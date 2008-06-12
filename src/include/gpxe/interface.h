#ifndef _GPXE_INTERFACE_H
#define _GPXE_INTERFACE_H

/** @file
 *
 * Object interfaces
 *
 */

#include <stddef.h>
#include <gpxe/refcnt.h>

/** An object interface operation */
struct interface_operation {
	/** Operation type */
	void *type;
	/** Implementing method */
	void *func;
};

/**
 * Define an object interface operation
 *
 * @v op_type		Operation type
 * @v object_type	Implementing method's expected object type
 * @v op_func		Implementing method
 * @ret op		Object interface operation
 */
#define INTF_OP( op_type, object_type, op_func ) {			      \
		.type = op_type,					      \
		.func = ( ( ( ( typeof ( op_func ) * ) NULL ) ==	      \
			    ( ( op_type ## _TYPE ( object_type ) * ) NULL ) ) \
			  ? op_func : op_func ),			      \
	}

/**
 * Get object interface operation method
 *
 * @v intf		Object interface
 * @v type		Operation type
 * @ret func		Implementing method, or NULL
 */
#define intf_op( intf, type ) \
	( ( type ## _TYPE ( void * ) * ) intf_op_untyped ( intf, type ) )

/** An object interface descriptor */
struct interface_descriptor {
	/** Offset of interface within containing object */
	size_t offset;
	/** Number of interface operations */
	unsigned int num_op;
	/** Object interface operations */
	struct interface_operation *op;
};

/**
 * Define an object interface descriptor
 *
 * @v object_type	Containing object data type
 * @v interface		Interface name (i.e. field within object data type)
 * @v operations	Object interface operations array
 * @ret desc		Object interface descriptor
 */
#define INTF_DESC( object_type, interface, operations ) {	\
		.offset = offsetof ( object_type, interface ),	\
		.op = operations,				\
		.num_op = ( sizeof ( operations ) /		\
			    sizeof ( operations[0] ) ),		\
	}

/** An object interface */
struct interface {
	/** Destination object interface
	 *
	 * When the containing object invokes an operation on this
	 * interface, it will be executed by the destination object.
	 *
	 * This pointer may never be NULL.  When the interface is
	 * unplugged, it should point to the null interface.
	 */
	struct interface *dest;
	/** Reference counter
	 *
	 * If this interface is not part of a reference-counted
	 * object, this field may be NULL.
	 */
	struct refcnt *refcnt;
	/** Interface descriptor */
	struct interface_descriptor *desc;
};

/** A null object interface */
struct null_interface {
	/** Object interface */
	struct interface intf;
};

extern void intf_plug ( struct interface *intf, struct interface *dest );
extern void intf_plug_plug ( struct interface *a, struct interface *b );
extern void intf_unplug ( struct interface *intf );
extern void intf_nullify ( struct interface *intf );
extern struct interface * intf_get ( struct interface *intf );
extern struct interface * intf_get_dest ( struct interface *intf );
extern void intf_put ( struct interface *intf );
extern void * __pure intf_object ( struct interface *intf );
extern void * intf_op_untyped ( struct interface *intf, void *type );

extern void intf_close ( struct interface *intf, int rc );
#define intf_close_TYPE( object_type ) \
	typeof ( void ( object_type, int rc ) )

extern void intf_shutdown ( struct interface *intf, int rc );
extern void intf_restart ( struct interface *intf, int rc );

extern struct interface_descriptor null_intf_desc;
extern struct null_interface null_intf;

/**
 * Initialise an object interface
 *
 * @v intf		Object interface
 * @v desc		Object interface descriptor
 * @v refcnt		Containing object reference counter, or NULL
 */
static inline void intf_init ( struct interface *intf,
			       struct interface_descriptor *desc,
			       struct refcnt *refcnt ) {
	intf->dest = &null_intf.intf;
	intf->refcnt = refcnt;
	intf->desc = desc;
}

/**
 * Initialise a static object interface
 *
 * @v descriptor	Object interface descriptor
 */
#define INTF_INIT( descriptor ) {		\
		.dest = &null_intf.intf,	\
		.refcnt = NULL,			\
		.desc = &(descriptor),		\
	}

/**
 * Find debugging colourisation for an object interface
 *
 * @v intf		Object interface
 * @ret col		Debugging colourisation
 *
 * Use as the first argument to DBGC() or equivalent macro.
 */
#define INTF_COL( intf ) intf_object ( intf )

/** printf() format string for INTF_DBG() */
#define INTF_FMT "%p+%zx"

/**
 * printf() arguments for representing an object interface
 *
 * @v intf		Object interface
 * @ret args		printf() argument list corresponding to INTF_FMT
 */
#define INTF_DBG( intf ) intf_object ( intf ), (intf)->desc->offset

/** printf() format string for INTF_INTF_DBG() */
#define INTF_INTF_FMT INTF_FMT "->" INTF_FMT

/**
 * printf() arguments for representing an object interface pair
 *
 * @v intf		Object interface
 * @v dest		Destination object interface
 * @ret args		printf() argument list corresponding to INTF_INTF_FMT
 */
#define INTF_INTF_DBG( intf, dest ) INTF_DBG ( intf ), INTF_DBG ( dest )

#endif /* _GPXE_INTERFACE_H */
