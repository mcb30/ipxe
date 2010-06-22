#ifndef _IPXE_REFCNT_H
#define _IPXE_REFCNT_H

/** @file
 *
 * Reference counting
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

/**
 * A reference counter
 *
 * This data structure is designed to be embedded within a
 * reference-counted object.
 *
 * Reference-counted objects are freed when their reference count
 * drops below zero.  This means that a freshly allocated-and-zeroed
 * reference-counted object will be freed on the first call to
 * ref_put().
 */
struct refcnt {
	/** Current reference count
	 *
	 * When this count is decremented below zero, the free()
	 * method will be called.
	 */
	int refcnt;
	/** Free containing object
	 *
	 * This method is called when the reference count is
	 * decremented below zero.
	 *
	 * If this method is left NULL, the standard library free()
	 * function will be called.  The upshot of this is that you
	 * may omit the free() method if the @c refcnt object is the
	 * first element of your reference-counted struct.
	 */
	void ( * free ) ( struct refcnt *refcnt );
};

/**
 * Initialise a reference counter
 *
 * @v refcnt		Reference counter
 * @v free		Freeing function
 */
static inline __attribute__ (( always_inline )) void
ref_init ( struct refcnt *refcnt,
	   void ( * free ) ( struct refcnt *refcnt ) ) {
	refcnt->free = free;
}

/**
 * Initialise a reference counter
 *
 * @v refcnt		Reference counter
 * @v free		Free containing object
 */
#define ref_init( refcnt, free ) do {					\
	if ( __builtin_constant_p ( (free) ) && ( (free) == NULL ) ) {	\
		/* Skip common case of no initialisation required */	\
	} else {							\
		ref_init ( (refcnt), (free) );				\
	}								\
	} while ( 0 )

/**
 * Initialise a static reference counter
 *
 * @v free		Free containing object
 */
#define REF_INIT( free ) {						\
		.free = free,						\
	}

extern struct refcnt * ref_get ( struct refcnt *refcnt );
extern void ref_put ( struct refcnt *refcnt );

#endif /* _IPXE_REFCNT_H */
