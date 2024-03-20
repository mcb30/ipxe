#ifndef _IPXE_EFI_HTTP_H
#define _IPXE_EFI_HTTP_H

/** @file
 *
 * EFI HTTP downloads
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

#include <ipxe/socket.h>
#include <ipxe/efi/efi.h>

struct uri;

extern int efi_http_download ( EFI_HANDLE service, struct uri *uri,
			       sa_family_t family );

#endif /* _IPXE_EFI_HTTP_H */
