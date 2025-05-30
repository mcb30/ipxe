/*
 * Copyright (C) 2019 Michael Brown <mbrown@fensystems.co.uk>.
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

#include <string.h>
#include <errno.h>
#include <assert.h>
#include <byteswap.h>
#include <ipxe/netdevice.h>
#include <ipxe/image.h>
#include <ipxe/uaccess.h>
#include <ipxe/umalloc.h>
#include <ipxe/fdt.h>

/** @file
 *
 * Flattened Device Tree
 *
 */

/** The system flattened device tree (if present) */
struct fdt sysfdt;

/** The downloaded flattened device tree tag */
struct image_tag fdt_image __image_tag = {
	.name = "FDT",
};

/** Amount of free space to add whenever we have to reallocate a tree */
#define FDT_INSERT_PAD 1024

/**
 * Describe next device tree token
 *
 * @v desc		Device tree token descriptor
 * @ret rc		Return status code
 */
int fdt_next ( struct fdt_token *desc ) {
	struct fdt *fdt = desc->fdt;
	unsigned int offset = desc->next;
	const fdt_token_t *token;
	const void *data;
	const struct fdt_prop *prop;
	unsigned int name_off;
	size_t remaining;
	size_t len;

	/* Sanity checks */
	assert ( fdt != NULL );
	assert ( offset <= fdt->len );
	assert ( ( offset & ( FDT_STRUCTURE_ALIGN - 1 ) ) == 0 );

	/* Initialise descriptor */
	desc->offset = offset;
	desc->name = NULL;
	desc->data = NULL;
	desc->len = 0;

	/* Locate token and calculate remaining space */
	token = ( fdt->raw + fdt->structure + offset );
	remaining = ( fdt->len - offset );
	if ( remaining < sizeof ( *token ) ) {
		DBGC ( fdt, "FDT truncated tree at +%#04x\n", offset );
		return -EINVAL;
	}
	remaining -= sizeof ( *token );
	data = ( ( ( const void * ) token ) + sizeof ( *token ) );
	len = 0;

	/* Handle token */
	switch ( *token ) {

	case cpu_to_be32 ( FDT_BEGIN_NODE ):

		/* Start of node */
		desc->name = data;
		len = ( strnlen ( desc->name, remaining ) + 1 /* NUL */ );
		if ( remaining < len ) {
			DBGC ( fdt, "FDT unterminated node name at +%#04x\n",
			       offset );
			return -EINVAL;
		}
		desc->depth++;
		break;

	case cpu_to_be32 ( FDT_END_NODE ):

		/* End of node */
		desc->depth--;
		break;

	case cpu_to_be32 ( FDT_PROP ):

		/* Property */
		prop = data;
		if ( remaining < sizeof ( *prop ) ) {
			DBGC ( fdt, "FDT truncated property at +%#04x\n",
			       offset );
			return -EINVAL;
		}
		desc->data = ( ( ( const void * ) prop ) + sizeof ( *prop ) );
		desc->len = be32_to_cpu ( prop->len );
		len = ( sizeof ( *prop ) + desc->len );
		if ( remaining < len ) {
			DBGC ( fdt, "FDT overlength property at +%#04x\n",
			       offset );
			return -EINVAL;
		}
		name_off = be32_to_cpu ( prop->name_off );
		if ( name_off > fdt->strings_len ) {
			DBGC ( fdt, "FDT property name outside strings "
			       "block at +%#04x\n", offset );
			return -EINVAL;
		}
		desc->name = ( fdt->raw + fdt->strings + name_off );
		break;

	case cpu_to_be32 ( FDT_NOP ):

		/* Do nothing */
		break;

	default:

		/* Unrecognised or unexpected token */
		DBGC ( fdt, "FDT unexpected token %#08x at +%#04x\n",
		       be32_to_cpu ( *token ), offset );
		return -EINVAL;
	}

	/* Calculate offset to next token */
	len = ( ( len + FDT_STRUCTURE_ALIGN - 1 ) &
		~( FDT_STRUCTURE_ALIGN - 1 ) );
	offset += ( sizeof ( *token ) + len );
	desc->next = offset;

	/* Sanity checks */
	assert ( offset <= fdt->len );

	return 0;
}

/**
 * Find root node
 *
 * @v fdt		Device tree
 * @ret rc		Return status code
 */
static int fdt_root ( struct fdt *fdt ) {
	struct fdt_token *root = ( ( struct fdt_token * ) &fdt->root );
	int rc;

	/* Initialise root node */
	root->fdt = fdt;
	root->next = 0;
	root->depth = 0;

	/* Locate initial node */
	while ( root->depth == 0 ) {

		/* Describe token */
		if ( ( rc = fdt_next ( root ) ) != 0 ) {
			DBGC ( fdt, "FDT has malformed root node: %s\n",
			       strerror ( rc ) );
			return rc;
		}

		/* Check for root node */
		if ( fdt_is_node ( root ) ) {
			assert ( root->name != NULL );
			assert ( root->name[0] == '\0' );
			root->name = "";
			assert ( root->data == NULL );
			return 0;
		}
	}

	DBGC ( fdt, "FDT could not locate root node\n" );
	return -ENOENT;
}

/**
 * Find child node
 *
 * @v node		Node descriptor
 * @v name		Node name
 * @v child		Child node descriptor to fill in
 * @ret rc		Return status code
 */
static int fdt_child ( const struct fdt_token *node, const char *name,
		       struct fdt_token *child ) {
	struct fdt *fdt = node->fdt;
	unsigned int offset = node->offset;
	int depth = node->depth;
	const char *sep;
	size_t name_len;
	int rc;

	/* Determine length of name (may be terminated with NUL or '/') */
	sep = strchr ( name, '/' );
	name_len = ( sep ? ( ( size_t ) ( sep - name ) ) : strlen ( name ) );

	/* Find child node */
	for ( fdt_fork ( node, child ) ; child->depth >= depth ; ) {

		/* Describe token */
		if ( ( rc = fdt_next ( child ) ) != 0 ) {
			DBGC ( fdt, "FDT +%#04x has malformed node: %s\n",
			       offset, strerror ( rc ) );
			return rc;
		}

		/* Check for matching immediate child node */
		if ( fdt_is_child ( child, depth ) ) {
			DBGC2 ( fdt, "FDT +%#04x has child node \"%s\" at "
				"+%#04x\n", offset, child->name,
				child->offset );
			assert ( child->depth > depth );
			if ( ( strlen ( child->name ) == name_len ) &&
			     ( memcmp ( name, child->name, name_len ) == 0 ) )
				return 0;
		}
	}

	DBGC2 ( fdt, "FDT +%#04x has no child node \"%s\"\n", offset, name );
	return -ENOENT;
}

/**
 * Find end of node
 *
 * @v node		Node descriptor
 * @v end		End node descriptor to fill in
 * @ret rc		Return status code
 */
static int fdt_end ( const struct fdt_token *node, struct fdt_token *end ) {
	struct fdt *fdt = node->fdt;
	unsigned int offset = node->offset;
	int depth = node->depth;
	int rc;

	/* Find end of this node */
	for ( fdt_fork ( node, end ) ; end->depth >= depth ; ) {

		/* Describe token */
		if ( ( rc = fdt_next ( end ) ) != 0 ) {
			DBGC ( fdt, "FDT +%#04x has malformed node: %s\n",
			       offset, strerror ( rc ) );
			return rc;
		}
	}

	DBGC2 ( fdt, "FDT +%#04x has end at +%#04x\n", offset, end->offset );
	return 0;
}

/**
 * Find node by path
 *
 * @v fdt		Device tree
 * @v path		Node path
 * @v node		Node descriptor to fill in
 * @ret rc		Return status code
 */
int fdt_path ( struct fdt *fdt, const char *path, struct fdt_token *node ) {
	const char *tmp = path;
	int rc;

	/* Initialise offset */
	memcpy ( node, &fdt->root, sizeof ( *node ) );

	/* Traverse tree one path segment at a time */
	while ( 1 ) {

		/* Skip any leading '/' */
		while ( *tmp == '/' )
			tmp++;

		/* Terminate if there are no more path components */
		if ( ! *tmp )
			break;

		/* Find child */
		if ( ( rc = fdt_child ( node, tmp, node ) ) != 0 )
			return rc;

		/* Move to next path component, if any */
		tmp = strchr ( tmp, '/' );
		if ( ! tmp )
			break;
	}

	DBGC2 ( fdt, "FDT found path \"%s\" at +%#04x\n", path, node->offset );
	return 0;
}

/**
 * Find node by alias
 *
 * @v fdt		Device tree
 * @v name		Alias name
 * @v node		Node descriptor to fill in
 * @ret rc		Return status code
 */
int fdt_alias ( struct fdt *fdt, const char *name, struct fdt_token *node ) {
	const char *alias;
	int rc;

	/* Locate "/aliases" node */
	if ( ( rc = fdt_child ( &fdt->root, "aliases", node ) ) != 0 )
		return rc;

	/* Locate alias property */
	if ( ( alias = fdt_string ( node, name ) ) == NULL )
		return -ENOENT;
	DBGC ( fdt, "FDT alias \"%s\" is \"%s\"\n", name, alias );

	/* Locate aliased node */
	if ( ( rc = fdt_path ( fdt, alias, node ) ) != 0 )
		return rc;

	return 0;
}

/**
 * Find property
 *
 * @v node		Node descriptor
 * @v name		Property name
 * @v desc		Token descriptor to fill in
 * @ret rc		Return status code
 */
static int fdt_property ( const struct fdt_token *node, const char *name,
			  struct fdt_token *desc ) {
	struct fdt *fdt = node->fdt;
	unsigned int offset = node->offset;
	int depth = node->depth;
	int rc;

	/* Find property */
	for ( fdt_fork ( node, desc ) ; desc->depth == depth ; ) {

		/* Describe token */
		if ( ( rc = fdt_next ( desc ) ) != 0 ) {
			DBGC ( fdt, "FDT +%#04x has malformed node: %s\n",
			       offset, strerror ( rc ) );
			return rc;
		}

		/* Check for matching immediate child property */
		if ( desc->data ) {
			DBGC2 ( fdt, "FDT +%#04x has property \"%s\" at "
				"+%#04x len %#zx\n", offset, desc->name,
				desc->offset, desc->len );
			assert ( desc->depth == depth );
			if ( strcmp ( name, desc->name ) == 0 ) {
				DBGC2_HDA ( fdt, 0, desc->data, desc->len );
				return 0;
			}
		}
	}

	DBGC2 ( fdt, "FDT +%#04x has no property \"%s\"\n", offset, name );
	return -ENOENT;
}

/**
 * Find strings property
 *
 * @v node		Node descriptor
 * @v name		Property name
 * @v count		String count to fill in
 * @ret string		String property, or NULL on error
 */
const char * fdt_strings ( const struct fdt_token *node, const char *name,
			   unsigned int *count ) {
	struct fdt *fdt = node->fdt;
	struct fdt_token desc;
	const char *data;
	size_t len;
	int rc;

	/* Return a zero count on error */
	*count = 0;

	/* Find property */
	if ( ( rc = fdt_property ( node, name, &desc ) ) != 0 )
		return NULL;

	/* Check NUL termination */
	data = desc.data;
	if ( desc.len && ( data[ desc.len - 1 ] != '\0' ) ) {
		DBGC ( fdt, "FDT unterminated string property \"%s\"\n",
		       name );
		return NULL;
	}

	/* Count number of strings */
	for ( len = desc.len ; len-- ; ) {
		if ( data[len] == '\0' )
			(*count)++;
	}

	return data;
}

/**
 * Find string property
 *
 * @v node		Node descriptor
 * @v name		Property name
 * @ret string		String property, or NULL on error
 */
const char * fdt_string ( const struct fdt_token *node, const char *name ) {
	unsigned int count;

	/* Find strings property */
	return fdt_strings ( node, name, &count );
}

/**
 * Get integer property
 *
 * @v node		Node descriptor
 * @v name		Property name
 * @v index		Starting cell index
 * @v count		Number of cells (or 0 to read all remaining cells)
 * @v value		Integer value to fill in
 * @ret rc		Return status code
 */
int fdt_cells ( const struct fdt_token *node, const char *name,
		unsigned int index, unsigned int count, uint64_t *value ) {
	struct fdt *fdt = node->fdt;
	struct fdt_token desc;
	const uint32_t *cell;
	unsigned int total;
	int rc;

	/* Clear value */
	*value = 0;

	/* Find property */
	if ( ( rc = fdt_property ( node, name, &desc ) ) != 0 )
		return rc;
	cell = desc.data;

	/* Determine number of cells */
	total = ( desc.len / sizeof ( *cell ) );
	if ( ( index > total ) || ( count > ( total - index ) ) ) {
		DBGC ( fdt, "FDT truncated integer \"%s\"\n", name );
		return -ERANGE;
	}
	if ( ! count )
		count = ( total - index );
	if ( count > ( sizeof ( *value ) / sizeof ( *cell ) ) ) {
		DBGC ( fdt, "FDT overlength integer \"%s\"\n", name );
		return -ERANGE;
	}

	/* Read value */
	for ( cell += index ; count ; cell++, count-- ) {
		*value <<= 32;
		*value |= be32_to_cpu ( *cell );
	}

	return 0;
}

/**
 * Get 64-bit integer property
 *
 * @v node		Node descriptor
 * @v name		Property name
 * @v value		Integer value to fill in
 * @ret rc		Return status code
 */
int fdt_u64 ( const struct fdt_token *node, const char *name,
	      uint64_t *value ) {
	int rc;

	/* Read value */
	if ( ( rc = fdt_cells ( node, name, 0, 0, value ) ) != 0 )
		return rc;

	return 0;
}

/**
 * Get 32-bit integer property
 *
 * @v node		Node descriptor
 * @v name		Property name
 * @v value		Integer value to fill in
 * @ret rc		Return status code
 */
int fdt_u32 ( const struct fdt_token *node, const char *name,
	      uint32_t *value ) {
	struct fdt *fdt = node->fdt;
	uint64_t value64;
	int rc;

	/* Read value */
	if ( ( rc = fdt_u64 ( node, name, &value64 ) ) != 0 )
		return rc;

	/* Check range */
	*value = value64;
	if ( *value != value64 ) {
		DBGC ( fdt, "FDT overlength 32-bit integer \"%s\"\n", name );
		return -ERANGE;
	}

	return 0;
}

/**
 * Get region cell size specification
 *
 * @v node		Parent node descriptor
 * @v regs		Region cell size specification to fill in
 *
 * Note that #address-cells and #size-cells are defined on the
 * immediate parent node, rather than on the node with the "reg"
 * property itself.
 */
void fdt_reg_cells ( const struct fdt_token *node,
		     struct fdt_reg_cells *regs ) {
	int rc;

	/* Read #address-cells, if present */
	if ( ( rc = fdt_u32 ( node, "#address-cells",
			      &regs->address_cells ) ) != 0 ) {
		regs->address_cells = FDT_DEFAULT_ADDRESS_CELLS;
	}

	/* Read #size-cells, if present */
	if ( ( rc = fdt_u32 ( node, "#size-cells",
			      &regs->size_cells ) ) != 0 ) {
		regs->size_cells = FDT_DEFAULT_SIZE_CELLS;
	}

	/* Calculate stride */
	regs->stride = ( regs->address_cells + regs->size_cells );
}

/**
 * Get number of regions
 *
 * @v node		Node descriptor
 * @v regs		Region cell size specification
 * @ret count		Number of regions, or negative error
 */
int fdt_reg_count ( const struct fdt_token *node,
		    struct fdt_reg_cells *regs ) {
	struct fdt_token desc;
	const uint32_t *cell;
	unsigned int count;
	int rc;

	/* Find property */
	if ( ( rc = fdt_property ( node, "reg", &desc ) ) != 0 )
		return rc;

	/* Determine number of regions */
	count = ( desc.len / ( regs->stride * sizeof ( *cell ) ) );
	return count;
}

/**
 * Get region address
 *
 * @v node		Node descriptor
 * @v regs		Region cell size specification
 * @v index		Region index
 * @v address		Region starting address to fill in
 * @ret rc		Return status code
 */
int fdt_reg_address ( const struct fdt_token *node,
		      struct fdt_reg_cells *regs, unsigned int index,
		      uint64_t *address ) {
	unsigned int cell = ( index * regs->stride );
	int rc;

	/* Read relevant portion of region array */
	if ( ( rc = fdt_cells ( node, "reg", cell, regs->address_cells,
				address ) ) != 0 ) {
		return rc;
	}

	return 0;
}

/**
 * Get region size
 *
 * @v node		Node descriptor
 * @v regs		Region cell size specification
 * @v index		Region index
 * @v size		Region size to fill in
 * @ret rc		Return status code
 */
int fdt_reg_size ( const struct fdt_token *node, struct fdt_reg_cells *regs,
		   unsigned int index, uint64_t *size ) {
	unsigned int cell = ( ( index * regs->stride ) + regs->address_cells );
	int rc;

	/* Read relevant portion of region array */
	if ( ( rc = fdt_cells ( node, "reg", cell, regs->size_cells,
				size ) ) != 0 ) {
		return rc;
	}

	return 0;
}

/**
 * Get MAC address from property
 *
 * @v node		Node descriptor
 * @v netdev		Network device
 * @ret rc		Return status code
 */
int fdt_mac ( const struct fdt_token *node, struct net_device *netdev ) {
	struct fdt *fdt = node->fdt;
	struct fdt_token desc;
	size_t len;
	int rc;

	/* Find applicable MAC address property */
	if ( ( ( rc = fdt_property ( node, "mac-address", &desc ) ) != 0 ) &&
	     ( ( rc = fdt_property ( node, "local-mac-address",
				     &desc ) ) != 0 ) ) {
		return rc;
	}

	/* Check length */
	len = netdev->ll_protocol->hw_addr_len;
	if ( len != desc.len ) {
		DBGC ( fdt, "FDT malformed MAC address \"%s\":\n",
		       desc.name );
		DBGC_HDA ( fdt, 0, desc.data, desc.len );
		return -ERANGE;
	}

	/* Fill in MAC address */
	memcpy ( netdev->hw_addr, desc.data, len );

	return 0;
}

/**
 * Parse device tree
 *
 * @v fdt		Device tree
 * @v hdr		Device tree header
 * @v max_len		Maximum device tree length
 * @ret rc		Return status code
 */
int fdt_parse ( struct fdt *fdt, struct fdt_header *hdr, size_t max_len ) {
	struct fdt_token node;
	const uint8_t *nul;
	size_t end;
	int rc;

	/* Sanity check */
	if ( sizeof ( *hdr ) > max_len ) {
		DBGC ( fdt, "FDT length %#zx too short for header\n",
		       max_len );
		goto err;
	}

	/* Record device tree location */
	fdt->hdr = hdr;
	fdt->len = be32_to_cpu ( hdr->totalsize );
	fdt->used = sizeof ( *hdr );
	if ( fdt->len > max_len ) {
		DBGC ( fdt, "FDT has invalid length %#zx / %#zx\n",
		       fdt->len, max_len );
		goto err;
	}
	DBGC ( fdt, "FDT version %d at %p+%#04zx (phys %#08lx)\n",
	       be32_to_cpu ( hdr->version ), fdt->hdr, fdt->len,
	       virt_to_phys ( hdr ) );

	/* Check signature */
	if ( hdr->magic != cpu_to_be32 ( FDT_MAGIC ) ) {
		DBGC ( fdt, "FDT has invalid magic value %#08x\n",
		       be32_to_cpu ( hdr->magic ) );
		goto err;
	}

	/* Check version */
	if ( hdr->last_comp_version != cpu_to_be32 ( FDT_VERSION ) ) {
		DBGC ( fdt, "FDT unsupported version %d\n",
		       be32_to_cpu ( hdr->last_comp_version ) );
		goto err;
	}

	/* Record structure block location */
	fdt->structure = be32_to_cpu ( hdr->off_dt_struct );
	fdt->structure_len = be32_to_cpu ( hdr->size_dt_struct );
	DBGC ( fdt, "FDT structure block at +[%#04x,%#04zx)\n",
	       fdt->structure, ( fdt->structure + fdt->structure_len ) );
	if ( ( fdt->structure > fdt->len ) ||
	     ( fdt->structure_len > ( fdt->len - fdt->structure ) ) ) {
		DBGC ( fdt, "FDT structure block exceeds table\n" );
		goto err;
	}
	if ( ( fdt->structure | fdt->structure_len ) &
	     ( FDT_STRUCTURE_ALIGN - 1 ) ) {
		DBGC ( fdt, "FDT structure block is misaligned\n" );
		goto err;
	}
	end = ( fdt->structure + fdt->structure_len );
	if ( fdt->used < end )
		fdt->used = end;

	/* Record strings block location */
	fdt->strings = be32_to_cpu ( hdr->off_dt_strings );
	fdt->strings_len = be32_to_cpu ( hdr->size_dt_strings );
	DBGC ( fdt, "FDT strings block at +[%#04x,%#04zx)\n",
	       fdt->strings, ( fdt->strings + fdt->strings_len ) );
	if ( ( fdt->strings > fdt->len ) ||
	     ( fdt->strings_len > ( fdt->len - fdt->strings ) ) ) {
		DBGC ( fdt, "FDT strings block exceeds table\n" );
		goto err;
	}
	end = ( fdt->strings + fdt->strings_len );
	if ( fdt->used < end )
		fdt->used = end;

	/* Shrink strings block to ensure NUL termination safety */
	nul = ( fdt->raw + fdt->strings + fdt->strings_len );
	for ( ; fdt->strings_len ; fdt->strings_len-- ) {
		if ( *(--nul) == '\0' )
			break;
	}
	if ( fdt->strings_len != be32_to_cpu ( hdr->size_dt_strings ) ) {
		DBGC ( fdt, "FDT strings block shrunk to +[%#04x,%#04zx)\n",
		       fdt->strings, ( fdt->strings + fdt->strings_len ) );
	}

	/* Record memory reservation block location */
	fdt->reservations = be32_to_cpu ( hdr->off_mem_rsvmap );
	DBGC ( fdt, "FDT memory reservations at +[%#04x,...)\n",
	       fdt->reservations );
	if ( fdt->used <= fdt->reservations ) {
		/* No size field exists: assume whole table is used */
		fdt->used = fdt->len;
	}

	/* Identify free space (if any) */
	if ( fdt->used < fdt->len ) {
		DBGC ( fdt, "FDT free space at +[%#04zx,%#04zx)\n",
		       fdt->used, fdt->len );
	}

	/* Find root node */
	if ( ( rc = fdt_root ( fdt ) ) != 0 )
		goto err;

	/* Print model name and boot arguments (for debugging) */
	if ( DBG_LOG ) {
		DBGC ( fdt, "FDT model is \"%s\"\n",
		       fdt_string ( &fdt->root, "model" ) );
		if ( fdt_child ( &fdt->root, "chosen", &node ) == 0 ) {
			DBGC ( fdt, "FDT boot arguments \"%s\"\n",
			       fdt_string ( &node, "bootargs" ) );
		}
	}

	return 0;

 err:
	DBGC_HDA ( fdt, 0, hdr, sizeof ( *hdr ) );
	memset ( fdt, 0, sizeof ( *fdt ) );
	return -EINVAL;
}

/**
 * Parse device tree image
 *
 * @v fdt		Device tree
 * @v image		Image
 * @ret rc		Return status code
 */
static int fdt_parse_image ( struct fdt *fdt, struct image *image ) {
	int rc;

	/* Parse image */
	if ( ( rc = fdt_parse ( fdt, image->rwdata, image->len ) ) != 0 ) {
		DBGC ( fdt, "FDT image \"%s\" is invalid: %s\n",
		       image->name, strerror ( rc ) );
		return rc;
	}

	DBGC ( fdt, "FDT image is \"%s\"\n", image->name );
	return 0;
}

/**
 * Insert empty space
 *
 * @v fdt		Device tree
 * @v offset		Offset at which to insert space
 * @v len		Length to insert (must be a multiple of FDT_MAX_ALIGN)
 * @ret rc		Return status code
 */
static int fdt_insert ( struct fdt *fdt, unsigned int offset, size_t len ) {
	size_t free;
	size_t new;
	int rc;

	/* Sanity checks */
	assert ( offset <= fdt->used );
	assert ( fdt->used <= fdt->len );
	assert ( ( len % FDT_MAX_ALIGN ) == 0 );

	/* Reallocate tree if necessary */
	free = ( fdt->len - fdt->used );
	if ( free < len ) {
		if ( ! fdt->realloc ) {
			DBGC ( fdt, "FDT is not reallocatable\n" );
			return -ENOTSUP;
		}
		new = ( fdt->len + ( len - free ) + FDT_INSERT_PAD );
		if ( ( rc = fdt->realloc ( fdt, new ) ) != 0 )
			return rc;
	}
	assert ( ( fdt->used + len ) <= fdt->len );

	/* Insert empty space */
	memmove ( ( fdt->raw + offset + len ), ( fdt->raw + offset ),
		  ( fdt->used - offset ) );
	memset ( ( fdt->raw + offset ), 0, len );
	fdt->used += len;

	/* Update offsets
	 *
	 * We assume that we never need to legitimately insert data at
	 * the start of a block, and therefore can unambiguously
	 * determine which block offsets need to be updated.
	 *
	 * It is the caller's responsibility to update the length (and
	 * contents) of the block into which it has inserted space.
	 */
	if ( fdt->structure >= offset ) {
		fdt->structure += len;
		fdt->hdr->off_dt_struct = cpu_to_be32 ( fdt->structure );
		DBGC ( fdt, "FDT structure block now at +[%#04x,%#04zx)\n",
		       fdt->structure,
		       ( fdt->structure + fdt->structure_len ) );
	}
	if ( fdt->strings >= offset ) {
		fdt->strings += len;
		fdt->hdr->off_dt_strings = cpu_to_be32 ( fdt->strings );
		DBGC ( fdt, "FDT strings block now at +[%#04x,%#04zx)\n",
		       fdt->strings, ( fdt->strings + fdt->strings_len ) );
	}
	if ( fdt->reservations >= offset ) {
		fdt->reservations += len;
		fdt->hdr->off_mem_rsvmap = cpu_to_be32 ( fdt->reservations );
		DBGC ( fdt, "FDT memory reservations now at +[%#04x,...)\n",
		       fdt->reservations );
	}

	return 0;
}

/**
 * Fill space in structure block with FDT_NOP
 *
 * @v fdt		Device tree
 * @v offset		Starting offset
 * @v len		Length (must be a multiple of FDT_STRUCTURE_ALIGN)
 */
static void fdt_nop ( struct fdt *fdt, unsigned int offset, size_t len ) {
	fdt_token_t *token;
	unsigned int count;

	/* Sanity check */
	assert ( ( len % FDT_STRUCTURE_ALIGN ) == 0 );

	/* Fill with FDT_NOP */
	token = ( fdt->raw + fdt->structure + offset );
	count = ( len / sizeof ( *token ) );
	while ( count-- )
		*(token++) = cpu_to_be32 ( FDT_NOP );
}

/**
 * Insert FDT_NOP padded space in structure block
 *
 * @v fdt		Device tree
 * @v offset		Offset at which to insert space
 * @v len		Minimal length to insert
 * @ret rc		Return status code
 */
static int fdt_insert_nop ( struct fdt *fdt, unsigned int offset,
			    size_t len ) {
	int rc;

	/* Sanity check */
	assert ( ( offset % FDT_STRUCTURE_ALIGN ) == 0 );

	/* Round up inserted length to maximal alignment */
	len = ( ( len + FDT_MAX_ALIGN - 1 ) & ~( FDT_MAX_ALIGN - 1 ) );

	/* Insert empty space in structure block */
	if ( ( rc = fdt_insert ( fdt, ( fdt->structure + offset ),
				 len ) ) != 0 )
		return rc;

	/* Fill with NOPs */
	fdt_nop ( fdt, offset, len );

	/* Update structure block size */
	fdt->structure_len += len;
	fdt->hdr->size_dt_struct = cpu_to_be32 ( fdt->structure_len );
	DBGC ( fdt, "FDT structure block now at +[%#04x,%#04zx)\n",
	       fdt->structure, ( fdt->structure + fdt->structure_len ) );

	return 0;
}

/**
 * Insert string in strings block
 *
 * @v fdt		Device tree
 * @v string		String
 * @v offset		String offset to fill in
 * @ret rc		Return status code
 */
static int fdt_insert_string ( struct fdt *fdt, const char *string,
			       unsigned int *offset ) {
	size_t len = ( strlen ( string ) + 1 /* NUL */ );
	int rc;

	/* Round up inserted length to maximal alignment */
	len = ( ( len + FDT_MAX_ALIGN - 1 ) & ~( FDT_MAX_ALIGN - 1 ) );

	/* Insert space at end of strings block */
	if ( ( rc = fdt_insert ( fdt, ( fdt->strings + fdt->strings_len ),
				 len ) ) != 0 )
		return rc;

	/* Append string to strings block */
	*offset = fdt->strings_len;
	strcpy ( ( fdt->raw + fdt->strings + *offset ), string );

	/* Update strings block size */
	fdt->strings_len += len;
	fdt->hdr->size_dt_strings = cpu_to_be32 ( fdt->strings_len );
	DBGC ( fdt, "FDT strings block now at +[%#04x,%#04zx)\n",
		       fdt->strings, ( fdt->strings + fdt->strings_len ) );

	return 0;
}

/**
 * Ensure child node exists
 *
 * @v node		Node descriptor
 * @v name		New node name
 * @v child		Child node descriptor to fill in
 * @ret rc		Return status code
 */
static int fdt_ensure_child ( const struct fdt_token *node, const char *name,
			      struct fdt_token *child ) {
	struct fdt *fdt = node->fdt;
	size_t name_len = ( strlen ( name ) + 1 /* NUL */ );
	fdt_token_t *token;
	size_t len;
	int rc;

	/* Find existing child node, if any */
	if ( ( rc = fdt_child ( node, name, child ) ) == 0 )
		return 0;

	/* Find end of parent node */
	if ( ( rc = fdt_end ( node, child ) ) != 0 )
		return rc;

	/* Insert space for child node (with maximal alignment) */
	len = ( sizeof ( fdt_token_t ) /* BEGIN_NODE */ + name_len +
		sizeof ( fdt_token_t ) /* END_NODE */ );
	if ( ( rc = fdt_insert_nop ( fdt, child->offset, len ) ) != 0 )
		return rc;

	/* Construct node */
	token = ( fdt->raw + fdt->structure + child->offset );
	*(token++) = cpu_to_be32 ( FDT_BEGIN_NODE );
	child->name = ( ( const void * ) token );
	memcpy ( token, name, name_len );
	name_len = ( ( name_len + FDT_STRUCTURE_ALIGN - 1 ) &
		     ~( FDT_STRUCTURE_ALIGN - 1 ) );
	token = ( ( ( void * ) token ) + name_len );
	*(token++) = cpu_to_be32 ( FDT_END_NODE );
	child->next = ( child->offset + sizeof ( fdt_token_t ) + name_len );
	child->depth = ( node->depth + 1 );
	DBGC2 ( fdt, "FDT +%#04x created child \"%s\" at +%#04x\n",
		node->offset, name, child->offset );

	return 0;
}

/**
 * Set property value
 *
 * @v node		Device tree node
 * @v name		Property name
 * @v data		Property data, or NULL to delete property
 * @v len		Length of property data
 * @ret rc		Return status code
 */
static int fdt_set ( const struct fdt_token *node, const char *name,
		     const void *data, size_t len ) {
	struct fdt *fdt = node->fdt;
	unsigned int offset = node->offset;
	struct fdt_token desc;
	struct {
		fdt_token_t token;
		struct fdt_prop prop;
		uint8_t data[0];
	} __attribute__ (( packed )) *hdr;
	unsigned int string;
	size_t erase;
	size_t insert;
	int rc;

	/* Find and reuse existing property, if any */
	if ( ( rc = fdt_property ( node, name, &desc ) ) == 0 ) {

		/* Reuse existing name */
		hdr = ( fdt->raw + fdt->structure + desc.offset );
		assert ( hdr->token == be32_to_cpu ( FDT_PROP ) );
		string = be32_to_cpu ( hdr->prop.name_off );

		/* Erase existing property */
		erase = ( sizeof ( *hdr ) + desc.len );
		erase = ( ( erase + FDT_STRUCTURE_ALIGN - 1 ) &
			  ~( FDT_STRUCTURE_ALIGN - 1 ) );
		fdt_nop ( fdt, desc.offset, erase );
		DBGC2 ( fdt, "FDT +%#04x erased property \"%s\"\n",
			offset, name );

		/* Calculate insertion position and length */
		insert = ( ( desc.len < len ) ? ( len - desc.len ) : 0 );

	} else {

		/* Create name */
		if ( ( rc = fdt_insert_string ( fdt, name, &string ) ) != 0 )
			return rc;

		/* Calculate insertion position and length */
		desc.offset = node->next;
		insert = ( sizeof ( *hdr ) + len );
	}

	/* Leave property erased if applicable */
	if ( ! data )
		return 0;

	/* Insert space */
	if ( ( rc = fdt_insert_nop ( fdt, desc.offset, insert ) ) != 0 )
		return rc;

	/* Construct property */
	hdr = ( fdt->raw + fdt->structure + desc.offset );
	hdr->token = cpu_to_be32 ( FDT_PROP );
	hdr->prop.len = cpu_to_be32 ( len );
	hdr->prop.name_off = cpu_to_be32 ( string );
	memset ( hdr->data, 0, ( ( len + FDT_STRUCTURE_ALIGN - 1 ) &
				 ~( FDT_STRUCTURE_ALIGN - 1 ) ) );
	memcpy ( hdr->data, data, len );
	DBGC2 ( fdt, "FDT +%#04x created property \"%s\"\n", offset, name );
	DBGC2_HDA ( fdt, 0, hdr->data, len );

	return 0;
}

/**
 * Reallocate device tree via urealloc()
 *
 * @v fdt		Device tree
 * @v len		New total length
 * @ret rc		Return status code
 */
static int fdt_urealloc ( struct fdt *fdt, size_t len ) {
	void *new;

	/* Sanity check */
	assert ( len >= fdt->used );

	/* Attempt reallocation */
	new = urealloc ( fdt->raw, len );
	if ( ! new ) {
		DBGC ( fdt, "FDT could not reallocate from +%#04zx to "
		       "+%#04zx\n", fdt->len, len );
		return -ENOMEM;
	}
	DBGC ( fdt, "FDT reallocated from +%#04zx to +%#04zx\n",
	       fdt->len, len );

	/* Update device tree */
	fdt->raw = new;
	fdt->len = len;
	fdt->hdr->totalsize = cpu_to_be32 ( len );

	return 0;
}

/**
 * Populate device tree with boot arguments
 *
 * @v fdt		Device tree
 * @v cmdline		Command line, or NULL
 * @v initrd		Initial ramdisk address (or 0 for no initrd)
 * @v initrd_len	Initial ramdisk length (or 0 for no initrd)
 * @ret rc		Return status code
 */
static int fdt_bootargs ( struct fdt *fdt, const char *cmdline,
			  physaddr_t initrd, size_t initrd_len ) {
	struct fdt_token node;
	physaddr_t addr;
	const void *data;
	size_t len;
	int rc;

	/* Ensure "chosen" node exists */
	if ( ( rc = fdt_ensure_child ( &fdt->root, "chosen", &node ) ) != 0 )
		return rc;

	/* Set or clear "bootargs" property */
	len = ( cmdline ? ( strlen ( cmdline ) + 1 /* NUL */ ) : 0 );
	if ( ( rc = fdt_set ( &node, "bootargs", cmdline, len ) ) != 0 )
		return rc;

	/* Set or clear initrd properties */
	data = ( initrd_len ? &addr : NULL );
	len = ( initrd_len ? sizeof ( addr ) : 0 );
	addr = initrd;
	addr = ( ( sizeof ( addr ) == sizeof ( uint64_t ) ) ?
		 cpu_to_be64 ( addr ) : cpu_to_be32 ( addr ) );
	if ( ( rc = fdt_set ( &node, "linux,initrd-start", data, len ) ) != 0 )
		return rc;
	addr = ( initrd + initrd_len );
	addr = ( ( sizeof ( addr ) == sizeof ( uint64_t ) ) ?
		 cpu_to_be64 ( addr ) : cpu_to_be32 ( addr ) );
	if ( ( rc = fdt_set ( &node, "linux,initrd-end", data, len ) ) != 0 )
		return rc;

	return 0;
}

/**
 * Create device tree
 *
 * @v hdr		Device tree header to fill in (may be set to NULL)
 * @v cmdline		Command line, or NULL
 * @v initrd		Initial ramdisk address (or 0 for no initrd)
 * @v initrd_len	Initial ramdisk length (or 0 for no initrd)
 * @ret rc		Return status code
 */
int fdt_create ( struct fdt_header **hdr, const char *cmdline,
		 physaddr_t initrd, size_t initrd_len ) {
	struct fdt_token *root;
	struct image *image;
	struct fdt fdt;
	void *copy;
	int rc;

	/* Use system FDT as the base by default */
	memcpy ( &fdt, &sysfdt, sizeof ( fdt ) );
	root = ( ( struct fdt_token * ) &fdt.root );
	root->fdt = &fdt;

	/* If an FDT image exists, use this instead */
	image = find_image_tag ( &fdt_image );
	if ( image && ( ( rc = fdt_parse_image ( &fdt, image ) ) != 0 ) )
		goto err_image;

	/* Exit successfully if we have no base FDT */
	if ( ! fdt.len ) {
		DBGC ( &fdt, "FDT has no base tree\n" );
		goto no_fdt;
	}

	/* Create modifiable copy */
	copy = umalloc ( fdt.len );
	if ( ! copy ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	memcpy ( copy, fdt.raw, fdt.len );
	fdt.raw = copy;
	fdt.realloc = fdt_urealloc;

	/* Populate boot arguments */
	if ( ( rc = fdt_bootargs ( &fdt, cmdline, initrd, initrd_len ) ) != 0 )
		goto err_bootargs;

 no_fdt:
	*hdr = fdt.raw;
	return 0;

 err_bootargs:
	ufree ( fdt.raw );
 err_alloc:
 err_image:
	return rc;
}

/**
 * Remove device tree
 *
 * @v hdr		Device tree header, or NULL
 */
void fdt_remove ( struct fdt_header *hdr ) {

	/* Free modifiable copy */
	ufree ( hdr );
}

/* Drag in objects via fdt_next() */
REQUIRING_SYMBOL ( fdt_next );

/* Drag in device tree configuration */
REQUIRE_OBJECT ( config_fdt );
