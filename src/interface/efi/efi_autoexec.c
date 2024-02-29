/*
 * Copyright (C) 2021 Michael Brown <mbrown@fensystems.co.uk>.
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

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <ipxe/image.h>
#include <ipxe/init.h>
#include <ipxe/in.h>
#include <ipxe/efi/efi.h>
#include <ipxe/efi/efi_path.h>
#include <ipxe/efi/efi_utils.h>
#include <ipxe/efi/efi_autoexec.h>
#include <ipxe/efi/Protocol/PxeBaseCode.h>
#include <ipxe/efi/Protocol/SimpleFileSystem.h>
#include <ipxe/efi/Guid/FileInfo.h>

/** @file
 *
 * EFI autoexec script
 *
 */

/** Autoexec script filename */
static wchar_t efi_autoexec_wname[] = L"autoexec.ipxe";

/** Autoexec script image name */
static char efi_autoexec_name[] = "autoexec.ipxe";

/** Autoexec script relative URI */
static struct uri efi_autoexec_uri = { .epath = "autoexec.ipxe" };

/** Autoexec script (if any) */
static void *efi_autoexec;

/** Autoexec script length */
static size_t efi_autoexec_len;

/**
 * Resize autoexec script
 *
 * @v xferbuf		Data transfer buffer
 * @v len		New length (or zero to free buffer)
 * @ret rc		Return status code
 */
static int efi_autoexec_resize ( size_t len ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	void *data = NULL;
	size_t copy_len;
	EFI_STATUS efirc;

	/* Allocate new buffer if necessary */
	if ( ( len > 0 ) &&
	     ( ( efirc = bs->AllocatePool ( EfiBootServicesData, len,
					    &data ) ) != 0 ) ) {
		rc = -EEFI ( efirc );
		return rc;
	}

	/* Copy data from old to new buffer */
	copy_len =  ( ( len < efi_autoexec_len) ? len : efi_autoexec_len );
	memcpy ( data, efi_autoexec, copy_len );

	/* Free old buffer */
	bs->FreePool ( efi_autoexec );

	/* Record new buffer */
	efi_autoexec = data;
	efi_autoexec_len = len;

	return 0;
}

/**
 * Load autoexec script from path within filesystem
 *
 * @v device		Device handle
 * @v path		Relative path to image, or NULL to load from root
 * @ret rc		Return status code
 */
static int efi_autoexec_filesystem ( EFI_HANDLE device,
				     EFI_DEVICE_PATH_PROTOCOL *path ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	union {
		void *interface;
		EFI_SIMPLE_FILE_SYSTEM_PROTOCOL *fs;
	} u;
	struct {
		EFI_FILE_INFO info;
		CHAR16 name[ sizeof ( efi_autoexec_wname ) /
			     sizeof ( efi_autoexec_wname[0] ) ];
	} info;
	FILEPATH_DEVICE_PATH *filepath;
	EFI_FILE_PROTOCOL *root;
	EFI_FILE_PROTOCOL *file;
	UINTN size;
	unsigned int dirlen;
	size_t len;
	CHAR16 *wname;
	EFI_STATUS efirc;
	int rc;

	/* Identify directory */
	if ( path ) {

		/* Check relative device path is a file path */
		if ( ! ( ( path->Type == MEDIA_DEVICE_PATH ) &&
			 ( path->SubType == MEDIA_FILEPATH_DP ) ) ) {
			DBGC ( device, "EFI %s image path ",
			       efi_handle_name ( device ) );
			DBGC ( device, " \"%s\" is not a file path\n",
			       efi_devpath_text ( path ) );
			rc = -ENOTTY;
			goto err_not_filepath;
		}
		filepath = container_of ( path, FILEPATH_DEVICE_PATH, Header );

		/* Find length of containing directory */
		dirlen = ( ( ( ( path->Length[1] << 8 ) | path->Length[0] )
			     - offsetof ( typeof ( *filepath ), PathName ) )
			   / sizeof ( filepath->PathName[0] ) );
		for ( ; dirlen ; dirlen-- ) {
			if ( filepath->PathName[ dirlen - 1 ] == L'\\' )
				break;
		}

	} else {

		/* Use root directory */
		filepath = NULL;
		dirlen = 0;
	}

	/* Allocate filename */
	len = ( ( dirlen * sizeof ( wname[0] ) ) +
		sizeof ( efi_autoexec_wname ) );
	wname = malloc ( len );
	if ( ! wname ) {
		rc = -ENOMEM;
		goto err_wname;
	}
	memcpy ( wname, filepath->PathName, ( dirlen * sizeof ( wname[0] ) ) );
	memcpy ( &wname[dirlen], efi_autoexec_wname,
		 sizeof ( efi_autoexec_wname ) );

	/* Open simple file system protocol */
	if ( ( efirc = bs->OpenProtocol ( device,
					  &efi_simple_file_system_protocol_guid,
					  &u.interface, efi_image_handle,
					  device,
					  EFI_OPEN_PROTOCOL_GET_PROTOCOL ))!=0){
		rc = -EEFI ( efirc );
		DBGC ( device, "EFI %s has no filesystem instance: %s\n",
		       efi_handle_name ( device ), strerror ( rc ) );
		goto err_filesystem;
	}

	/* Open root directory */
	if ( ( efirc = u.fs->OpenVolume ( u.fs, &root ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( device, "EFI %s could not open volume: %s\n",
		       efi_handle_name ( device ), strerror ( rc ) );
		goto err_volume;
	}

	/* Open autoexec script */
	if ( ( efirc = root->Open ( root, &file, wname,
				    EFI_FILE_MODE_READ, 0 ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( device, "EFI %s has no %ls: %s\n",
		       efi_handle_name ( device ), wname, strerror ( rc ) );
		goto err_open;
	}

	/* Get file information */
	size = sizeof ( info );
	if ( ( efirc = file->GetInfo ( file, &efi_file_info_id, &size,
				       &info ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( device, "EFI %s could not get %ls info: %s\n",
		       efi_handle_name ( device ), wname, strerror ( rc ) );
		goto err_getinfo;
	}
	size = info.info.FileSize;

	/* Ignore zero-length files */
	if ( ! size ) {
		rc = -EINVAL;
		DBGC ( device, "EFI %s has zero-length %ls\n",
		       efi_handle_name ( device ), wname );
		goto err_empty;
	}

	/* Allocate temporary copy */
	if ( ( rc = efi_autoexec_resize ( size ) ) != 0 ) {
		DBGC ( device, "EFI %s could not allocate %ls: %s\n",
		       efi_handle_name ( device ), wname, strerror ( rc ) );
		goto err_alloc;
	}

	/* Read file */
	if ( ( efirc = file->Read ( file, &size, efi_autoexec ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( device, "EFI %s could not read %ls: %s\n",
		       efi_handle_name ( device ), wname, strerror ( rc ) );
		goto err_read;
	}

	/* Success */
	rc = 0;
	DBGC ( device, "EFI %s found %ls\n",
	       efi_handle_name ( device ), wname );

 err_read:
	if ( rc != 0 )
		efi_autoexec_resize ( 0 );
 err_alloc:
 err_empty:
 err_getinfo:
	file->Close ( file );
 err_open:
	root->Close ( root );
 err_volume:
	bs->CloseProtocol ( device, &efi_simple_file_system_protocol_guid,
			    efi_image_handle, device );
 err_filesystem:
	free ( wname );
 err_wname:
 err_not_filepath:
	return rc;
}

/**
 * Load autoexec script from HTTP server
 *
 * @v device		Device handle
 * @v path		Device path
 * @ret rc		Return status code
 */
static int efi_autoexec_http ( EFI_HANDLE device,
			       EFI_DEVICE_PATH_PROTOCOL *path ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	EFI_GUID *protocol = &efi_http_service_binding_protocol_guid;
	EFI_DEVICE_PATH_PROTOCOL *next;
	EFI_HANDLE httpsb;
	sa_family_t sa_family;
	struct uri *base;
	struct uri *uri;
	EFI_STATUS efirc;
	int rc;

	/* Parse device path */
	sa_family = AF_UNSPEC;
	base = NULL;
	for ( ; ( next = efi_path_next ( path ) ) ; path = next ) {
		if ( path->Type == MESSAGING_DEVICE_PATH ) {
			switch ( path->SubType ) {
			case MSG_IPv4_DP:
				sa_family = AF_INET;
				break;
			case MSG_IPv6_DP:
				sa_family = AF_INET6;
				break;
			case MSG_URI_DP:
				base = efi_path_uri ( path );
				break;
			}
		}
	}
	if ( sa_family == AF_UNSPEC ) {
		DBGC ( device, "EFI %s has no IPv4/IPv6 path\n",
		       efi_handle_name ( device ) );
		rc = -ENOTTY;
		goto err_no_family;
	}
	if ( ! base ) {
		DBGC ( device, "EFI %s has no URI path\n",
		       efi_handle_name ( device ) );
		rc = -ENOTTY;
		goto err_no_uri;
	}

	/* Resolve download URI */
	uri = resolve_uri ( base, efi_autoexec_uri );
	if ( ! uri ) {
		rc = -ENOMEM;
		goto err_resolve;
	}

	/* Locate parent HTTP service binding protocol handle */
	if ( ( rc = efi_locate_device ( device, protocol, &httpsb, 0 ) ) != 0){
		DBGC ( device, "EFI %s has no HTTP service binding: %s\n",
		       efi_handle_name ( device ), strerror ( rc ) );
		goto err_locate;
	}

	/* Create HTTP I/O protocol child handle */
	if ( ( rc = efi_service_create ( httpsb, protocol, &child ) ) != 0 ) {
		DBGC ( device, "EFI %s could not create HTTP I/O: %s\n",
		       efi_handle_name ( device ), strerror ( rc ) );
		goto err_create;
	}

	/* Download via HTTP I/O protocol */
	if ( ( rc = efi_http_download ( httpsb, uri, sa_family,
					) ) != 0 ) {
		DBGC ( device, "EFI %s could not download: %s\n",
		       efi_handle_name ( device ), strerror ( rc ) );
		goto err_download;
	}

	efi_service_destroy ( httpsb, protocol, child );
 err_create:
 err_locate:
	uri_put ( uri );
 err_resolve:
	uri_put ( base );
 err_no_uri:
 err_no_family:
	return rc;
}

/**
 * Load autoexec script from TFTP server
 *
 * @v device		Device handle
 * @ret rc		Return status code
 */
static int efi_autoexec_tftp ( EFI_HANDLE device ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	union {
		void *interface;
		EFI_PXE_BASE_CODE_PROTOCOL *pxe;
	} u;
	EFI_PXE_BASE_CODE_MODE *mode;
	EFI_PXE_BASE_CODE_PACKET *packet;
	union {
		struct in_addr in;
		EFI_IP_ADDRESS ip;
	} server;
	size_t filename_max;
	char *filename;
	char *sep;
	UINT64 size;
	EFI_STATUS efirc;
	int rc;

	/* Open PXE base code protocol */
	if ( ( efirc = bs->OpenProtocol ( device,
					  &efi_pxe_base_code_protocol_guid,
					  &u.interface, efi_image_handle,
					  device,
					  EFI_OPEN_PROTOCOL_GET_PROTOCOL ))!=0){
		rc = -EEFI ( efirc );
		DBGC ( device, "EFI %s has no PXE base code instance: %s\n",
		       efi_handle_name ( device ), strerror ( rc ) );
		goto err_pxe;
	}

	/* Do not attempt to parse DHCPv6 packets */
	mode = u.pxe->Mode;
	if ( mode->UsingIpv6 ) {
		rc = -ENOTSUP;
		DBGC ( device, "EFI %s has IPv6 PXE base code\n",
		       efi_handle_name ( device ) );
		goto err_ipv6;
	}

	/* Identify relevant reply packet */
	if ( mode->PxeReplyReceived &&
	     mode->PxeReply.Dhcpv4.BootpBootFile[0] ) {
		/* Use boot filename if present in PXE reply */
		DBGC ( device, "EFI %s using PXE reply filename\n",
		       efi_handle_name ( device ) );
		packet = &mode->PxeReply;
	} else if ( mode->DhcpAckReceived &&
		    mode->DhcpAck.Dhcpv4.BootpBootFile[0] ) {
		/* Otherwise, use boot filename if present in DHCPACK */
		DBGC ( device, "EFI %s using DHCPACK filename\n",
		       efi_handle_name ( device ) );
		packet = &mode->DhcpAck;
	} else if ( mode->ProxyOfferReceived &&
		    mode->ProxyOffer.Dhcpv4.BootpBootFile[0] ) {
		/* Otherwise, use boot filename if present in ProxyDHCPOFFER */
		DBGC ( device, "EFI %s using ProxyDHCPOFFER filename\n",
		       efi_handle_name ( device ) );
		packet = &mode->ProxyOffer;
	} else {
		/* No boot filename available */
		rc = -ENOENT;
		DBGC ( device, "EFI %s has no PXE boot filename\n",
		       efi_handle_name ( device ) );
		goto err_packet;
	}

	/* Allocate filename */
	filename_max = ( sizeof ( packet->Dhcpv4.BootpBootFile )
			 + ( sizeof ( efi_autoexec_name ) - 1 /* NUL */ )
			 + 1 /* NUL */ );
	filename = zalloc ( filename_max );
	if ( ! filename ) {
		rc = -ENOMEM;
		goto err_filename;
	}

	/* Extract next-server address and boot filename */
	memset ( &server, 0, sizeof ( server ) );
	memcpy ( &server.in, packet->Dhcpv4.BootpSiAddr,
		 sizeof ( server.in ) );
	memcpy ( filename, packet->Dhcpv4.BootpBootFile,
		 sizeof ( packet->Dhcpv4.BootpBootFile ) );

	/* Update filename to autoexec script name */
	sep = strrchr ( filename, '/' );
	if ( ! sep )
		sep = strrchr ( filename, '\\' );
	if ( ! sep )
		sep = ( filename - 1 );
	strcpy ( ( sep + 1 ), efi_autoexec_name );

	/* Get file size */
	if ( ( efirc = u.pxe->Mtftp ( u.pxe,
				      EFI_PXE_BASE_CODE_TFTP_GET_FILE_SIZE,
				      NULL, FALSE, &size, NULL, &server.ip,
				      ( ( UINT8 * ) filename ), NULL,
				      FALSE ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( device, "EFI %s could not get size of %s:%s: %s\n",
		       efi_handle_name ( device ), inet_ntoa ( server.in ),
		       filename, strerror ( rc ) );
		goto err_size;
	}

	/* Ignore zero-length files */
	if ( ! size ) {
		rc = -EINVAL;
		DBGC ( device, "EFI %s has zero-length %s:%s\n",
		       efi_handle_name ( device ), inet_ntoa ( server.in ),
		       filename );
		goto err_empty;
	}

	/* Allocate temporary copy */
	if ( ( rc = efi_autoexec_resize ( size ) ) != 0 ) {
		DBGC ( device, "EFI %s could not allocate %s:%s: %s\n",
		       efi_handle_name ( device ), inet_ntoa ( server.in ),
		       filename, strerror ( rc ) );
		goto err_alloc;
	}

	/* Download file */
	if ( ( efirc = u.pxe->Mtftp ( u.pxe, EFI_PXE_BASE_CODE_TFTP_READ_FILE,
				      efi_autoexec, FALSE, &size, NULL,
				      &server.ip, ( ( UINT8 * ) filename ),
				      NULL, FALSE ) ) != 0 ) {
		rc = -EEFI ( efirc );
		DBGC ( device, "EFI %s could not download %s:%s: %s\n",
		       efi_handle_name ( device ), inet_ntoa ( server.in ),
		       filename, strerror ( rc ) );
		goto err_download;
	}

	/* Success */
	rc = 0;
	DBGC ( device, "EFI %s found %s:%s\n", efi_handle_name ( device ),
	       inet_ntoa ( server.in ), filename );

 err_download:
	if ( rc != 0 )
		efi_autoexec_resize ( 0 );
 err_alloc:
 err_empty:
 err_size:
	free ( filename );
 err_filename:
 err_packet:
 err_ipv6:
	bs->CloseProtocol ( device, &efi_pxe_base_code_protocol_guid,
			    efi_image_handle, device );
 err_pxe:
	return rc;
}

/**
 * Load autoexec script
 *
 * @v device		Device handle
 * @v devpath		Device handle's device path
 * @v filepath		Image path within device handle
 * @ret rc		Return status code
 */
int efi_autoexec_load ( EFI_HANDLE device,
			EFI_DEVICE_PATH_PROTOCOL *devpath,
			EFI_DEVICE_PATH_PROTOCOL *filepath ) {
	int rc;

	/* Sanity check */
	assert ( efi_autoexec == NULL );
	assert ( efi_autoexec_len == 0 );

	/* Try loading from file system loaded image directory, if supported */
	if ( ( rc = efi_autoexec_filesystem ( device, filepath ) ) == 0 )
		return 0;

	/* Try loading from file system root directory, if supported */
	if ( ( rc = efi_autoexec_filesystem ( device, NULL ) ) == 0 )
		return 0;

	/* Try loading via HTTP, if supported */
	if ( ( rc = efi_autoexec_http ( device, devpath ) ) == 0 )
		return 0;

	/* Try loading via TFTP, if supported */
	if ( ( rc = efi_autoexec_tftp ( device ) ) == 0 )
		return 0;

	return -ENOENT;
}

/**
 * Register autoexec script
 *
 */
static void efi_autoexec_startup ( void ) {
	EFI_BOOT_SERVICES *bs = efi_systab->BootServices;
	EFI_HANDLE device = efi_loaded_image->DeviceHandle;
	struct image *image;

	/* Do nothing if we have no autoexec script */
	if ( ! efi_autoexec )
		return;

	/* Create autoexec image */
	image = image_memory ( efi_autoexec_name,
			       virt_to_user ( efi_autoexec ),
			       efi_autoexec_len );
	if ( ! image ) {
		DBGC ( device, "EFI %s could not create %s\n",
		       efi_handle_name ( device ), efi_autoexec_name );
		return;
	}
	DBGC ( device, "EFI %s registered %s\n",
	       efi_handle_name ( device ), efi_autoexec_name );

	/* Free temporary copy */
	efi_autoexec_resize ( 0 );
}

/** Autoexec script startup function */
struct startup_fn efi_autoexec_startup_fn __startup_fn ( STARTUP_NORMAL ) = {
	.name = "efi_autoexec",
	.startup = efi_autoexec_startup,
};
