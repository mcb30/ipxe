/*
 * Copyright (C) 2025 Michael Brown <mbrown@fensystems.co.uk>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
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

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <byteswap.h>
#include <ipxe/fdt.h>
#include <ipxe/fdtmem.h>

/** @file
 *
 * Flattened Device Tree memory map
 *
 */

/** In-memory size of the iPXE image (defined by linker) */
extern size_t ABS_SYMBOL ( _memsz );
static size_t memsz = ABS_VALUE_INIT ( _memsz );

/** Relocation required alignment (defined by prefix or linker) */
extern physaddr_t ABS_SYMBOL ( _max_align );
static physaddr_t max_align = ABS_VALUE_INIT ( _max_align );

/** Colour for debug messages */
#define colour &memsz

/**
 * Avoid memory region
 *
 * @v top		Candidate top address
 * @v len		Candidate length
 * @v start		Start of region to avoid
 * @v size		Size of region to avoid
 * @v name		Region name (for debugging)
 * @ret top		Updated candidate top address
 */
static physaddr_t fdt_avoid ( physaddr_t top, size_t len,
			      uint64_t start, uint64_t size,
			      const char *name ) {
	physaddr_t below;
	size_t space;

	/* Check for overlap, avoiding integer underflow or overflow
	 * and minimising the use of unnecessary 64-bit arithmetic on
	 * 32-bit CPUs.
	 */
	DBGC2 ( colour, "...checking [%#llx,%#llx] %s",
		( ( unsigned long long ) start ),
		( ( unsigned long long ) ( start + size - 1 ) ), name );
	if ( top <= start ) {
		/* Candidate is already below the region to be avoided */
		DBGC2 ( colour, "\n" );
		return top;
	}
	below = ( start & ~( max_align - 1 ) );
	space = ( top - start );
	if ( space < size ) {
		/* End of candidate overlaps the region to be avoided */
		DBGC2 ( colour, " (overlaps)\n" );
		return below;
	}
	space -= size;
	if ( space < len ) {
		/* End of region to be avoided overlaps candidate */
		DBGC2 ( colour, " (overlaps)\n" );
		return below;
	}
	/* Candidate is fully above the region to be avoided */
	DBGC2 ( colour, "\n" );
	return top;
}

/**
 * Avoid memory reservations
 *
 * @v top		Candidate top address
 * @v len		Candidate length
 * @v fdt		Device tree
 * @ret top		Updated candidate top address
 */
static physaddr_t fdt_avoid_reservations ( physaddr_t top, size_t len,
					   struct fdt *fdt ) {
	const struct fdt_reservation *rsv;
	struct fdt_descriptor desc;
	struct fdt_reg_cells regs;
	unsigned int offset;
	uint64_t start;
	uint64_t size;
	int depth;
	int count;
	int index;
	int rc;

	/* Avoid all reservations in the memory reservations block */
	for_each_fdt_reservation ( rsv, fdt ) {
		top = fdt_avoid ( top, len, be64_to_cpu ( rsv->start ),
				  be64_to_cpu ( rsv->size ), "<rsv>" );
	}

	/* Locate reserved-memory node */
	if ( ( rc = fdt_path ( fdt, "/reserved-memory", &offset ) ) != 0 )
		goto err;

	/* Parse region cell sizes */
	fdt_reg_cells ( fdt, offset, &regs );

	/* Scan through reservations */
	for ( depth = -1 ; ; depth += desc.depth, offset = desc.next ) {

		/* Describe token */
		if ( ( rc = fdt_describe ( fdt, offset, &desc ) ) != 0 ) {
			DBGC ( colour, "FDTMEM has malformed node: %s\n",
			       strerror ( rc ) );
			goto err;
		}

		/* Terminate when we exit the reserved-memory node */
		if ( ( depth == 0 ) && ( desc.depth < 0 ) )
			break;

		/* Ignore any non-immediate child nodes */
		if ( ! ( ( depth == 0 ) && desc.name && ( ! desc.data ) ) )
			continue;

		/* Count regions */
		count = fdt_reg_count ( fdt, desc.offset, &regs );
		if ( count < 0 ) {
			rc = count;
			DBGC ( colour, "FDTMEM has malformed region %s: %s\n",
			       desc.name, strerror ( rc ) );
			continue;
		}

		/* Scan through this region */
		for ( index = 0 ; index < count ; index++ ) {

			/* Get region starting address and size */
			if ( ( rc = fdt_reg_address ( fdt, desc.offset, &regs,
						      index, &start ) ) != 0 ){
				DBGC ( colour, "FDTMEM %s region %d has "
				       "malformed start address: %s\n",
				       desc.name, index, strerror ( rc ) );
				break;
			}
			if ( ( rc = fdt_reg_size ( fdt, desc.offset, &regs,
						   index, &size ) ) != 0 ) {
				DBGC ( colour, "FDTMEM %s region %d has "
				       "malformed size: %s\n",
				       desc.name, index, strerror ( rc ) );
				break;
			}

			/* Avoid this region */
			top = fdt_avoid ( top, len, start, size, desc.name );
		}
	}

 err:
	return top;
}

/**
 * Avoid all relevant memory regions
 *
 * @v top		Candidate top address
 * @v len		Candidate length
 * @v fdt		Device tree
 * @v hdrphys		FDT header physical address
 * @v old		Current physical address of iPXE
 * @ret top		Updated candidate top address
 */
static physaddr_t fdt_avoid_all ( physaddr_t top, size_t len,
				  struct fdt *fdt, physaddr_t hdrphys,
				  physaddr_t old ) {
	physaddr_t prev;

	/* Avoid all regions */
	while ( 1 ) {

		/* Record previous top */
		prev = top;
		DBGC2 ( colour, "...trying [%#lx,%#lx]\n",
			( top - len ), ( top - 1 ) );

		/* Avoid current iPXE iamge */
		top = fdt_avoid ( top, len, old, memsz, "iPXE" );

		/* Avoid device tree */
		top = fdt_avoid ( top, len, hdrphys, fdt->len, "FDT" );

		/* Avoid all memory reservations */
		top = fdt_avoid_reservations ( top, len, fdt );

		/* Loop until all regions have been avoided */
		if ( prev == top )
			return top;
	}
}

/**
 * Find a relocation address for iPXE
 *
 * @v hdr		FDT header
 * @v hdrphys		FDT header physical address
 * @v old		Current physical address of iPXE
 * @v max		Maximum accessible physical address
 * @ret new		New physical address for relocation
 *
 * Find a suitably aligned address towards the top of existent memory
 * to which iPXE may be relocated, along with a copy of the system
 * device tree.
 *
 * This function may be called very early in initialisation, before
 * .data is writable or .bss has been zeroed.  Neither this function
 * nor any function that it calls may write to or rely upon the zero
 * initialisation of any static variables.
 *
 * Note that virt_to_phys() may rely upon variables in .data having
 * been written, and so we need to be passed both the virtual and
 * physical addresses of the FDT as parameters.
 */
physaddr_t fdt_relocate ( struct fdt_header *hdr, physaddr_t hdrphys,
			  physaddr_t old, physaddr_t max ) {
	struct fdt fdt;
	struct fdt_descriptor desc;
	struct fdt_reg_cells regs;
	const char *devtype;
	unsigned int offset;
	physaddr_t addr;
	physaddr_t top;
	physaddr_t new;
	uint64_t start;
	uint64_t size;
	uint64_t end;
	size_t len;
	int depth;
	int count;
	int index;
	int rc;

	/* Sanity check */
	assert ( ( max_align & ( max_align - 1 ) ) == 0 );

	/* Parse FDT */
	if ( ( rc = fdt_parse ( &fdt, hdr, -1UL ) ) != 0 ) {
		DBGC ( colour, "FDTMEM could not parse FDT: %s\n",
		       strerror ( rc ) );
		goto err;
	}

	/* Determine required length */
	assert ( memsz > 0 );
	assert ( ( memsz % FDT_MAX_ALIGN ) == 0 );
	assert ( ( fdt.len % FDT_MAX_ALIGN ) == 0 );
	len = ( memsz + fdt.len );
	len = ( ( len + max_align - 1 ) & ~( max_align - 1 ) );
	assert ( len > 0 );
	DBGC ( colour, "FDTMEM requires %#zx + %#zx => %#zx bytes for "
	       "relocation\n", memsz, fdt.len, len );

	/* Parse region cell sizes */
	fdt_reg_cells ( &fdt, 0, &regs );

	/* Scan through memory regions */
	new = old;
	for ( offset = 0, depth = -1 ; ;
	      depth += desc.depth, offset = desc.next ) {

		/* Describe token */
		if ( ( rc = fdt_describe ( &fdt, offset, &desc ) ) != 0 ) {
			DBGC ( colour, "FDTMEM has malformed node: %s\n",
			       strerror ( rc ) );
			goto err;
		}

		/* Terminate when we exit the root node */
		if ( ( depth == 0 ) && ( desc.depth < 0 ) )
			break;

		/* Ignore any non-immediate child nodes */
		if ( ! ( ( depth == 0 ) && desc.name && ( ! desc.data ) ) )
			continue;

		/* Ignore any non-memory nodes */
		devtype = fdt_string ( &fdt, desc.offset, "device_type" );
		if ( ( ! devtype ) || ( strcmp ( devtype, "memory" ) != 0 ) )
			continue;

		/* Count regions */
		count = fdt_reg_count ( &fdt, desc.offset, &regs );
		if ( count < 0 ) {
			rc = count;
			DBGC ( colour, "FDTMEM has malformed region %s: %s\n",
			       desc.name, strerror ( rc ) );
			continue;
		}

		/* Scan through this region */
		for ( index = 0 ; index < count ; index++ ) {

			/* Get region starting address and size */
			if ( ( rc = fdt_reg_address ( &fdt, desc.offset, &regs,
						      index, &start ) ) != 0 ){
				DBGC ( colour, "FDTMEM %s region %d has "
				       "malformed start address: %s\n",
				       desc.name, index, strerror ( rc ) );
				break;
			}
			if ( ( rc = fdt_reg_size ( &fdt, desc.offset, &regs,
						   index, &size ) ) != 0 ) {
				DBGC ( colour, "FDTMEM %s region %d has "
				       "malformed size: %s\n",
				       desc.name, index, strerror ( rc ) );
				break;
			}

			/* Calculate region end address (avoiding overflow) */
			end = ( start + size - 1 );
			DBGC2 ( colour, "FDTMEM found [%#llx,%#llx]\n",
				( ( unsigned long long ) start ),
				( ( unsigned long long ) end ) );
			if ( end < start ) {
				DBGC2 ( colour, "...invalid size\n" );
				continue;
			}

			/* Limit region to accessible memory */
			if ( start > max ) {
				DBGC2 ( colour, "...not accessible\n" );
				continue;
			}
			if ( end > max ) {
				end = max;
				DBGC2 ( colour, "...truncated to "
					"[%#llx,%#llx]\n",
					( ( unsigned long long ) start ),
					( ( unsigned long long ) end ) );
			}

			/* Calculate initial candidate top address */
			top = ( ( end + 1 ) & ~( max_align - 1 ) );

			/* Avoid any necessary regions */
			top = fdt_avoid_all ( top, len, &fdt, hdrphys, old );

			/* Ignore regions with insufficient length */
			if ( ( top < len ) || ( ( top - len ) < start ) ) {
				DBGC2 ( colour, "...not usable\n" );
				continue;
			}

			/* Use highest valid candidate address */
			addr = ( top - len );
			assert ( ( addr & ( max_align - 1 ) ) == 0 );
			if ( addr > old )
				new = addr;
		}
	}

	DBGC ( colour, "FDTMEM relocating %#llx => [%#llx,%#llx]\n",
	       ( ( unsigned long long ) old ), ( ( unsigned long long ) new ),
	       ( ( unsigned long long ) ( new + len ) ) );
	return new;

 err:
	/* Return current address on any error (i.e. no relocation) */
	return old;
}
