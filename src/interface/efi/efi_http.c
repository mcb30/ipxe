/*
 * Copyright (C) 2024 Michael Brown <mbrown@fensystems.co.uk>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * You can also choose to distribute this program under the terms of
 * the Unmodified Binary Distribution Licence (as given in the file
 * COPYING.UBDL), provided that you have satisfied its requirements.
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

/** @file
 *
 * EFI HTTP downloads
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <libgen.h>
#include <ipxe/uri.h>
#include <ipxe/timer.h>
#include <ipxe/efi/efi.h>
#include <ipxe/efi/efi_path.h>
#include <ipxe/efi/efi_strings.h>
#include <ipxe/efi/efi_service.h>
#include <ipxe/efi/efi_http.h>
#include <ipxe/efi/Protocol/Http.h>
#include <ipxe/efi/Protocol/Ip4Config2.h>

/** Timeout for EFI HTTP messages */
#define EFIHTTP_TIMEOUT_TICKS ( 5 * TICKS_PER_SEC )

/** An EFI HTTP download */
struct efi_http {
	/** Reference count */
	struct refcnt refcnt;
	/** URI */
	struct uri *uri;
	/** URI as ASCII */
	char *uristring;
	/** URI as UCS-2 */
	wchar_t *wuristring;
	/** Name (for debug messages) */
	const char *name;

	/** Service binding handle */
	EFI_HANDLE service;
	/** EFI HTTP handle */
	EFI_HANDLE handle;
	/** EFI HTTP protocol instance */
	EFI_HTTP_PROTOCOL *http;
	/** EFI HTTP token */
	EFI_HTTP_TOKEN token;
	/** EFI HTTP event */
	EFI_EVENT event;
	/** EFI HTTP message has completed */
	int done;
};

/**
 * Free EFI HTTP download
 *
 * @v refcnt		Reference count
 */
static void efi_http_free ( struct refcnt *refcnt ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	struct efi_http *efihttp =
		container_of ( refcnt, struct efi_http, refcnt );

	/* Close event, if opened */
	if ( efihttp->event )
		bs->CloseEvent ( efihttp->event );

	/* Close HTTP protocol, if opened */
	if ( efihttp->http ) {
		bs->CloseProtocol ( efihttp->handle, &efi_http_protocol_guid,
				    efi_image_handle, efihttp->handle );
	}

	/* Delete HTTP protocol child, if created */
	if ( efihttp->handle ) {
		efi_service_del ( efihttp->service,
				  &efi_http_service_binding_protocol_guid,
				  efihttp->handle );
	}

	/* Free dynamically allocated data */
	uri_put ( efihttp->uri );
	free ( efihttp->uristring );
	free ( efihttp->wuristring );

	/* Free download structure */
	free ( efihttp );
}

/**
 * Record EFI HTTP message completion
 *
 * @v event		EFI event
 * @v context		Context
 */
static EFIAPI void efi_http_event ( EFI_EVENT event __unused, void *context ) {
	struct efi_http *efihttp = context;

	/* Record message as complete */
	efihttp->done = 1;
}

/**
 * Configure EFI HTTP download for IPv4
 *
 * @v efihttp		EFI HTTP download
 * @v netcfg		Network address configuration
 * @ret rc		Return status code
 */
static int efi_http_configure_ipv4 ( struct efi_http *efihttp,
				     struct efi_path_net_config *netcfg ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	IPv4_DEVICE_PATH *ipv4 = netcfg->ipv4;
	union {
		void *interface;
		EFI_IP4_CONFIG2_PROTOCOL *ip4cfg;
	} u;
	EFI_IP4_CONFIG2_MANUAL_ADDRESS address;
	EFI_STATUS efirc;
	int rc;

	/* Sanity check */
	assert ( ipv4 != NULL );

	/* Open IPv4 configuration protocol */
	if ( ( efirc = bs->OpenProtocol ( efihttp->service,
					  &efi_ip4_config2_protocol_guid,
					  &u.interface, efi_image_handle,
					  efihttp->service,
					  EFI_OPEN_PROTOCOL_GET_PROTOCOL ))!=0){
		rc = -EEFI ( efirc );
		DBGC ( efihttp, "EFIHTTP %s could not open IPv4 config: %s\n",
		       efihttp->name, strerror ( rc ) );
		goto err_open;
	}

	/* Configure address */
	memcpy ( &address.Address, &ipv4->LocalIpAddress,
		 sizeof ( address.Address ) );
	memcpy ( &address.SubnetMask, &ipv4->SubnetMask,
		 sizeof ( address.SubnetMask ) );
	if ( ( efirc = u.ip4cfg->SetData ( u.ip4cfg,
					   Ip4Config2DataTypeManualAddress,
					   sizeof ( address ),
					   &address ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( efihttp, "EFIHTTP %s could not set IPv4 address: %s\n",
		       efihttp->name, strerror ( rc ) );
		goto err_address;
	}

	/* Configure gateway */
	if ( ( efirc = u.ip4cfg->SetData ( u.ip4cfg,
					   Ip4Config2DataTypeGateway,
					   sizeof ( ipv4->GatewayIpAddress ),
					   &ipv4->GatewayIpAddress ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( efihttp, "EFIHTTP %s could not set IPv4 gateway: %s\n",
		       efihttp->name, strerror ( rc ) );
		goto err_gateway;
	}

	/* Configure DNS, if applicable */
	//
	DBGC_HDA ( efihttp, 0, netcfg->dns->DnsServerIp, netcfg->dns_len );
	if ( netcfg->dns && ( ! netcfg->dns->IsIPv6 ) &&
	     ( ( efirc = u.ip4cfg->SetData ( u.ip4cfg,
					     Ip4Config2DataTypeDnsServer,
					     netcfg->dns_len,
					     netcfg->dns->DnsServerIp ) ) !=0)){
		rc = -EEFI ( efirc );
		DBGC ( efihttp, "EFIHTTP %s could not set IPv4 DNS: %s\n",
		       efihttp->name, strerror ( rc ) );
		goto err_dns;
	}

	/* Success */
	rc = 0;

 err_dns:
 err_gateway:
 err_address:
	bs->CloseProtocol ( efihttp->service, &efi_ip4_config2_protocol_guid,
			    efi_image_handle, efihttp->service );
 err_open:
	return rc;
}

/**
 * Configure EFI HTTP download for IPv6
 *
 * @v efihttp		EFI HTTP download
 * @v netcfg		Network address configuration
 * @ret rc		Return status code
 */
static int efi_http_configure_ipv6 ( struct efi_http *efihttp,
				     struct efi_path_net_config *netcfg ) {

	( void ) efihttp;
	( void ) netcfg;
	return -ENOTSUP;
}

/**
 * Configure EFI HTTP download
 *
 * @v efihttp		EFI HTTP download
 * @v netcfg		Network address configuration
 * @ret rc		Return status code
 */
static int efi_http_configure ( struct efi_http *efihttp,
				struct efi_path_net_config *netcfg ) {
	struct {
		EFI_HTTP_CONFIG_DATA data;
		union {
			EFI_HTTPv4_ACCESS_POINT ipv4;
			EFI_HTTPv6_ACCESS_POINT ipv6;
		};
	} config;
	int ( * ipcfg ) ( struct efi_http *efihttp,
			  struct efi_path_net_config *netcfg );
	EFI_STATUS efirc;
	int rc;

	/* Configure relevant IP address family */
	ipcfg = ( netcfg->ipv6 ?
		  efi_http_configure_ipv6 : efi_http_configure_ipv4 );
	if ( ( rc = ipcfg ( efihttp, netcfg ) ) != 0 )
		return rc;

	/* Construct HTTP configuration */
	memset ( &config, 0, sizeof ( config ) );
	config.data.HttpVersion = HttpVersion11;
	if ( netcfg->ipv6 ) {
		config.data.LocalAddressIsIPv6 = TRUE;
		config.data.AccessPoint.IPv6Node = &config.ipv6;
	} else {
		config.ipv4.UseDefaultAddress = TRUE;
		config.data.AccessPoint.IPv4Node = &config.ipv4;
	}

	/* Configure HTTP protocol */
	if ( ( efirc = efihttp->http->Configure ( efihttp->http,
						  &config.data ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( efihttp, "EFIHTTP %s could not configure: %s\n",
		       efihttp->name, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Submit an EFI HTTP message and wait for completion
 *
 * @v efihttp		EFI HTTP download
 * @v submit		Submission method
 * @v msg		EFI HTTP message
 * @ret rc		Return status code
 */
static int efi_http_message ( struct efi_http *efihttp,
			      EFI_HTTP_REQUEST submit,
			      EFI_HTTP_MESSAGE *msg ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	unsigned long started;
	EFI_STATUS efirc;
	int rc;

	/* Prepare for submission */
	efihttp->done = 0;
	efihttp->token.Status = EFI_TIMEOUT;
	efihttp->token.Message = msg;

	/* Submit message (at EFI external TPL) */
	bs->RestoreTPL ( efi_external_tpl );
	efirc = submit ( efihttp->http, &efihttp->token );
	bs->RaiseTPL ( efi_internal_tpl );
	if ( efirc != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( efihttp, "EFIHTTP %s could not submit: %s\n",
		       efihttp->name, strerror ( rc ) );
		return rc;
	}

	/* Wait for completion or timeout */
	started = currticks();
	while ( ( currticks() - started ) < EFIHTTP_TIMEOUT_TICKS ) {

		/* Poll (at EFI external TPL) until completion or timeout */
		if ( ! efihttp->done ) {
			bs->RestoreTPL ( efi_external_tpl );
			efihttp->http->Poll ( efihttp->http );
			bs->RaiseTPL ( efi_internal_tpl );
			continue;
		}

		/* Check completion status */
		if ( ( efirc = efihttp->token.Status ) != 0 ) {
			rc = -EEFI ( efirc );
			DBGC ( efihttp, "EFIHTTP %s submission failed: %s\n",
			       efihttp->name, strerror ( rc ) );
			return rc;
		}
	}

	/* Cancel incomplete submission */
	efihttp->http->Cancel ( efihttp->http, &efihttp->token );
	efihttp->token.Message = NULL;

	DBGC ( efihttp, "EFIHTTP %s submission timed out\n", efihttp->name );
	return -ETIMEDOUT;
}

/**
 * Free EFI HTTP headers
 *
 * @v msg		EFI HTTP message
 *
 * Free an HTTP message header list as constructed by
 * EFI_HTTP_UTILITIES_PROTOCOL.Parse() (and hence as provided to us by
 * EFI_HTTP_PROTOCOL as part of a response message).
 *
 * The UEFI specification (as of version 2.10) says absolutely nothing
 * about who is responsible for freeing the memory allocated by
 * EFI_HTTP_UTILITIES_PROTOCOL.Parse(), or how it was allocated.  The
 * specification does say that it is the EFI_HTTP_PROTOCOL caller's
 * responsibility to free the response headers, but again fails to
 * give any indication of how this memory was allocated.
 *
 * Inspection of the reference implementation in EDK2 shows that the
 * caller is expected to use FreePool() on each header field name and
 * value separately, and then use FreePool() once more on the list
 * pointer itself.
 *
 * Though this behaviour is not specified, we will need to match it in
 * order to interoperate with the reference EDK2 implementation.
 * Given the under-specified nature of the protocols, it is unlikely
 * that any competing implementation will ever exist.
 */
static void efi_http_free_headers ( EFI_HTTP_MESSAGE *msg ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	unsigned int i;

	/* Free header key-value pairs.  Yes, DxeHttpLib really does
	 * allocate each of these separately, even though it would
	 * suffice to simply overwrite the separator colons and
	 * newlines with NULs.
	 */
	for ( i = 0 ; i < msg->HeaderCount ; i++ ) {
		bs->FreePool ( msg->Headers[i].FieldName );
		bs->FreePool ( msg->Headers[i].FieldValue );
	}
	msg->HeaderCount = 0;

	/* Free the header list */
	bs->FreePool ( msg->Headers );
	msg->Headers = NULL;
}

/**
 * Download file via EFI HTTP
 *
 * @v service		HTTP service binding handle
 * @v uri		URI
 * @v netcfg		Network address configuration
 * @ret rc		Return status code
 */
int efi_http_download ( EFI_HANDLE service, struct uri *uri,
			struct efi_path_net_config *netcfg ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	union {
		void *interface;
		EFI_HTTP_PROTOCOL *http;
	} u;
	struct efi_http *efihttp;
	EFI_STATUS efirc;
	int rc;

	/* Allocate and initialise structure */
	efihttp = zalloc ( sizeof ( *efihttp ) );
	if ( ! efihttp ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	ref_init ( &efihttp->refcnt, efi_http_free );
	efihttp->uri = uri_get ( uri );
	efihttp->service = service;
	efihttp->uristring = format_uri_alloc ( uri );
	if ( ! efihttp->uristring ) {
		rc = -ENOMEM;
		goto err_uristring;
	}
	if ( ( rc = efi_asprintf ( &efihttp->wuristring, "%s",
				   efihttp->uristring ) ) < 0 ) {
		goto err_wuristring;
	}
	efihttp->name = basename ( efihttp->uristring );
	DBGC ( efihttp, "EFIHTTP %s created from %s\n",
	       efihttp->name, efi_handle_name ( service ) );
	DBGC ( efihttp, "EFIHTTP %s downloading %ls\n",
	       efihttp->name, efihttp->wuristring );

	/* Create HTTP protocol instance */
	if ( ( rc = efi_service_add ( efihttp->service,
				      &efi_http_service_binding_protocol_guid,
				      &efihttp->handle ) ) != 0 ) {
		DBGC ( efihttp, "EFIHTTP %s could not create HTTP child: %s\n",
		       efihttp->name, strerror ( rc ) );
		goto err_service;
	}

	/* Open HTTP protocol */
	if ( ( efirc = bs->OpenProtocol ( efihttp->handle,
					  &efi_http_protocol_guid, &u.interface,
					  efi_image_handle, efihttp->handle,
					  EFI_OPEN_PROTOCOL_GET_PROTOCOL ))!=0){
		rc = -EEFI ( efirc );
		DBGC ( efihttp, "EFIHTTP %s could not open HTTP protocol: %s\n",
		       efihttp->name, strerror ( rc ) );
		goto err_open;
	}
	efihttp->http = u.http;

	/* Create completion event */
	if ( ( efirc = bs->CreateEvent ( EVT_NOTIFY_SIGNAL, TPL_NOTIFY,
					 efi_http_event, efihttp,
					 &efihttp->event ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( efihttp, "EFIHTTP %s could not create event: %s\n",
		       efihttp->name, strerror ( rc ) );
		goto err_event;
	}

	/* Configure HTTP */
	if ( ( rc = efi_http_configure ( efihttp, netcfg ) ) != 0 )
		goto err_configure;

	//
	{
		EFI_HTTP_REQUEST_DATA req;
		EFI_HTTP_MESSAGE msg;

		memset ( &msg, 0, sizeof ( msg ) );
		msg.Data.Request = &req;
		req.Method = HttpMethodGet;
		req.Url = efihttp->wuristring;

		efi_http_message ( efihttp, efihttp->http->Request, &msg );
	}
	( void ) efi_http_free_headers;

 err_configure:
 err_event:
 err_open:
 err_service:
 err_wuristring:
 err_uristring:
	ref_put ( &efihttp->refcnt );
 err_alloc:
	return rc;
}
