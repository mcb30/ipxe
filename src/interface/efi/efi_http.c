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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <ipxe/efi/efi.h>
#include <ipxe/efi/efi_service.h>
#include <ipxe/efi/efi_http.h>
#include <ipxe/efi/Protocol/Http.h>

/** An EFI HTTP download */
struct efi_http {
	/** Token */
	EFI_HTTP_TOKEN token;

	/** Progress event */
	EFI_EVENT progress;
	/** Progress producer counter */
	unsigned int prod;
	/** Progress consumer counter */
	unsigned int cons;
	/** Timeout event */
	EFI_EVENT timeout;
};

/**
 * Record EFI HTTP download progress
 *
 * @v event		EFI event
 * @v context		Context
 */
static EFIAPI void efi_http_progress ( EFI_EVENT event __unused,
				       void *context ) {
	struct efi_http *efihttp = context;

	/* Sanity check */
	assert ( efihttp->prod == efihttp->cons );

	/* Record progress */
	efihttp->prod++;
}

/**
 * Wait for EFI HTTP download to make progress
 *
 * @v efihttp		EFI HTTP download
 * @ret rc		Return status code
 */
static int efi_http_wait ( struct efi_http *efihttp ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	int rc;

	/* Set timeout timer.  Though EFI_HTTP_PROTOCOL requires us to
	 * provide a nominal timeout parameter TimeOutMillisec as part
	 * of EFI_HTTP_CONFIG_DATA, there is literally nothing in
	 * HttpDxe that ever acts upon the value of this parameter.
	 */
	if ( ( efirc = bs->SetTimer ( efihttp->event, TimerRelative,
				      EFIHTTP_TIMEOUT ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( efihttp, "EFIHTTP %p could not set timer: %s\n",
		       efihttp, strerror ( rc ) );
		goto err_set_timer;
	}

	/* Wait for progress */
	while ( efihttp->cons != efihttp->prod )


	bs->SetTimer ( efihttp->event, TimerCancel, 0 );
 err_set_timer:
	return rc;
}


/**
 * Download file via EFI HTTP
 *
 * @v service		HTTP service binding handle
 * @v uri		URI
 * @v family		IP address family
 * @ret rc		Return status code
 */
int efi_http_download ( EFI_HANDLE service, struct uri *uri,
			sa_family_t family ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	union {
		void *interface;
		EFI_HTTP_PROTOCOL *http;
	} u;
	struct {
		EFI_HTTP_CONFIG_DATA data;
		union {
			EFI_HTTPv4_ACCESS_POINT ipv4;
			EFI_HTTPv6_ACCESS_POINT ipv6;
		};
	} config;
	EFI_HANDLE handle = NULL;
	EFI_STATUS efirc;
	int rc;

	/* Create HTTP protocol instance */
	if ( ( rc = efi_service_add ( service,
				      &efi_http_service_binding_protocol_guid,
				      &handle ) ) != 0 ) {
		DBGC ( service, "EFIHTTP %s could not create HTTP child: %s\n",
		       efi_handle_name ( service ), strerror ( rc ) );
		goto err_service;
	}

	/* Open HTTP protocol */
	if ( ( efirc = bs->OpenProtocol ( handle, &efi_http_protocol_guid,
					  &u.interface, efi_image_handle,
					  handle,
					  EFI_OPEN_PROTOCOL_GET_PROTOCOL ))!=0){
		rc = -EEFI ( efirc );
		DBGC ( service, "EFIHTTP %s could not open HTTP protocol: %s\n",
		       efi_handle_name ( service ), strerror ( rc ) );
		goto err_open;
	}

	/* Construct configuration */
	memset ( &config, 0, sizeof ( config ) );
	config.data.HttpVersion = HttpVersion11;
	switch ( family ) {
	case AF_INET:
		config.ipv4.UseDefaultAddress = TRUE;
		config.data.AccessPoint.IPv4Node = &config.ipv4;
		break;
	case AF_INET6:
		config.data.LocalAddressIsIPv6 = TRUE;
		config.data.AccessPoint.IPv6Node = &config.ipv6;
		break;
	default:
		DBGC ( service, "EFIHTTP %s unsupported address family %d\n",
		       efi_handle_name ( service ), family );
		rc = -ENOTSUP;
		goto err_family;
	}

	/* Configure HTTP protocol */
	if ( ( efirc = u.http->Configure ( u.http, &config.data ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( service, "EFIHTTP %s could not configure: %s\n",
		       efi_handle_name ( service ), strerror ( rc ) );
		goto err_configure;
	}


	//
	( void ) uri;

 err_configure:
 err_family:
	bs->CloseProtocol ( handle, &efi_http_protocol_guid, efi_image_handle,
			    handle );
 err_open:
	efi_service_del ( service, &efi_http_service_binding_protocol_guid,
			  handle );
 err_service:
	return rc;
}
