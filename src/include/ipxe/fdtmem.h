#ifndef _IPXE_FDTMEM_H
#define _IPXE_FDTMEM_H

/** @file
 *
 * Flattened Device Tree memory map
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

#include <stdint.h>
#include <ipxe/fdt.h>

extern physaddr_t fdt_relocate ( struct fdt_header *hdr, physaddr_t hdrphys,
				 physaddr_t old, physaddr_t max );

#endif /* _IPXE_FDTMEM_H */
