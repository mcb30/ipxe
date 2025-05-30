#ifndef _IPXE_FDT_H
#define _IPXE_FDT_H

/** @file
 *
 * Flattened Device Tree
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

#include <stdint.h>
#include <ipxe/image.h>

struct net_device;

/** Device tree header */
struct fdt_header {
	/** Magic signature */
	uint32_t magic;
	/** Total size of device tree */
	uint32_t totalsize;
	/** Offset to structure block */
	uint32_t off_dt_struct;
	/** Offset to strings block */
	uint32_t off_dt_strings;
	/** Offset to memory reservation block */
	uint32_t off_mem_rsvmap;
	/** Version of this data structure */
	uint32_t version;
	/** Lowest version to which this structure is compatible */
	uint32_t last_comp_version;
	/** Physical ID of the boot CPU */
	uint32_t boot_cpuid_phys;
	/** Length of string block */
	uint32_t size_dt_strings;
	/** Length of structure block */
	uint32_t size_dt_struct;
} __attribute__ (( packed ));

/** Magic signature */
#define FDT_MAGIC 0xd00dfeed

/** Expected device tree version */
#define FDT_VERSION 16

/** Device tree token */
typedef uint32_t fdt_token_t;

/** Begin node token */
#define FDT_BEGIN_NODE 0x00000001

/** End node token */
#define FDT_END_NODE 0x00000002

/** Property token */
#define FDT_PROP 0x00000003

/** Property fragment */
struct fdt_prop {
	/** Data length */
	uint32_t len;
	/** Name offset */
	uint32_t name_off;
} __attribute__ (( packed ));

/** NOP token */
#define FDT_NOP 0x00000004

/** End of structure block */
#define FDT_END 0x00000009

/** Alignment of structure block */
#define FDT_STRUCTURE_ALIGN ( sizeof ( fdt_token_t ) )

/** Maximum alignment of any block */
#define FDT_MAX_ALIGN 8

/** A memory reservation */
struct fdt_reservation {
	/** Starting address */
	uint64_t start;
	/** Length of reservation */
	uint64_t size;
} __attribute__ (( packed ));

/** A device tree token descriptor */
struct fdt_token {
	/** Device tree */
	struct fdt *fdt;
	/** Offset within structure block */
	unsigned int offset;
	/** Next offset within structure block */
	unsigned int next;
	/** Node or property name (if applicable)
	 *
	 * Note that this pointer will be invalidated by any operation
	 * that modifies the device tree.
	 */
	const char *name;
	/** Property data (if applicable)
	 *
	 * Note that this pointer will be invalidated by any operation
	 * that modifies the device tree.
	 */
	const void *data;
	/** Length of property data (if applicable) */
	size_t len;
	/** Depth (after processing this token) */
	int depth;
};

/** A device tree */
struct fdt {
	/** Tree data */
	union {
		/** Tree header */
		struct fdt_header *hdr;
		/** Raw data */
		void *raw;
	};
	/** Length of tree */
	size_t len;
	/** Used length of tree */
	size_t used;
	/** Offset to structure block */
	unsigned int structure;
	/** Length of structure block */
	size_t structure_len;
	/** Offset to strings block */
	unsigned int strings;
	/** Length of strings block */
	size_t strings_len;
	/** Offset to memory reservation block */
	unsigned int reservations;
	/** Root node */
	const struct fdt_token root;
	/** Reallocate device tree
	 *
	 * @v fdt		Device tree
	 * @v len		New length
	 * @ret rc		Return status code
	 */
	int ( * realloc ) ( struct fdt *fdt, size_t len );
};

/** A device tree region cell size specification */
struct fdt_reg_cells {
	/** Number of address cells */
	uint32_t address_cells;
	/** Number of size cells */
	uint32_t size_cells;
	/** Number of address cells plus number of size cells */
	unsigned int stride;
};

/** Default number of address cells, if not specified */
#define FDT_DEFAULT_ADDRESS_CELLS 2

/** Default number of size cells, if not specified */
#define FDT_DEFAULT_SIZE_CELLS 1

extern struct image_tag fdt_image __image_tag;
extern struct fdt sysfdt;

/**
 * Get memory reservations
 *
 * @v fdt		Device tree
 * @ret rsv		Memory reservations
 */
static inline const struct fdt_reservation *
fdt_reservations ( struct fdt *fdt ) {

	return ( fdt->raw + fdt->reservations );
}

/** Iterate over memory reservations */
#define for_each_fdt_reservation( rsv, fdt )			\
	for ( rsv = fdt_reservations ( (fdt) ) ;		\
	      ( (rsv)->start || (rsv)->size ) ; rsv++ )

/**
 * Initialise a subordinate token descriptor for iteration
 *
 * @v node		Device tree node
 * @v desc		Subordinate token descriptor to fill in
 */
static inline void fdt_fork ( const struct fdt_token *node,
			      struct fdt_token *desc ) {

	/* Fill in fields required by fdt_next() */
	desc->fdt = node->fdt;
	desc->next = node->next;
	desc->depth = node->depth;
}

/**
 * Check if token is a node
 *
 * @v desc		Token descriptor
 * @ret is_node		Token represents the start of a node
 */
static inline int fdt_is_node ( const struct fdt_token *desc ) {

	return ( desc->name && ( ! desc->data ) );
}

/**
 * Check if token is a child node
 *
 * @v desc		Token descriptor
 * @v depth		Parent node depth
 * @ret is_child	Token represents a child node
 */
static inline int fdt_is_child ( const struct fdt_token *desc, int depth ) {

	return ( fdt_is_node ( desc ) && ( desc->depth == ( depth + 1 ) ) );
}

extern int fdt_next ( struct fdt_token *desc );
extern int fdt_path ( struct fdt *fdt, const char *path,
		      struct fdt_token *node );
extern int fdt_alias ( struct fdt *fdt, const char *name,
		       struct fdt_token *node );
extern const char * fdt_strings ( const struct fdt_token *node,
				  const char *name, unsigned int *count );
extern const char * fdt_string (  const struct fdt_token *node,
				  const char *name );
extern int fdt_cells ( const struct fdt_token *node, const char *name,
		       unsigned int index, unsigned int count,
		       uint64_t *value );
extern int fdt_u64 ( const struct fdt_token *node, const char *name,
		     uint64_t *value );
extern int fdt_u32 ( const struct fdt_token *node, const char *name,
		     uint32_t *value );
extern void fdt_reg_cells ( const struct fdt_token *node,
			    struct fdt_reg_cells *regs );
extern int fdt_reg_count ( const struct fdt_token *node,
			   struct fdt_reg_cells *regs );
extern int fdt_reg_address ( const struct fdt_token *node,
			     struct fdt_reg_cells *regs, unsigned int index,
			     uint64_t *address );
extern int fdt_reg_size ( const struct fdt_token *node,
			  struct fdt_reg_cells *regs, unsigned int index,
			  uint64_t *size );
extern int fdt_mac ( const struct fdt_token *node, struct net_device *netdev );
extern int fdt_parse ( struct fdt *fdt, struct fdt_header *hdr,
		       size_t max_len );
extern int fdt_create ( struct fdt_header **hdr, const char *cmdline,
			physaddr_t initrd, size_t initrd_len );
extern void fdt_remove ( struct fdt_header *hdr );

#endif /* _IPXE_FDT_H */
