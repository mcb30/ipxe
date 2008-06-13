#ifndef _GPXE_RESOLV_H
#define _GPXE_RESOLV_H

/** @file
 *
 * Name resolution
 *
 */

#include <gpxe/interface.h>
#include <gpxe/tables.h>

struct sockaddr;

/** A name resolver */
struct resolver {
	/** Name of this resolver (e.g. "DNS") */
	const char *name;
	/** Start name resolution
	 *
	 * @v resolv		Name resolution interface
	 * @v name		Name to resolve
	 * @v sa		Socket address to complete
	 * @ret rc		Return status code
	 */
	int ( * resolv ) ( struct interface *resolv, const char *name,
			   struct sockaddr *sa );
};

/** Numeric resolver priority */
#define RESOLV_NUMERIC 01

/** Normal resolver priority */
#define RESOLV_NORMAL 02

/** Register as a name resolver */
#define __resolver( resolv_order ) \
	__table ( struct resolver, resolvers, resolv_order )

extern void resolv_done ( struct interface *intf, struct sockaddr *sa );
#define resolv_done_TYPE( object_type ) \
	typeof ( void ( object_type, struct sockaddr *sa ) )

extern int resolv ( struct interface *resolv, const char *name,
		    struct sockaddr *sa );

#endif /* _GPXE_RESOLV_H */
