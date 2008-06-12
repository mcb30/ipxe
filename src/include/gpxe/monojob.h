#ifndef _GPXE_MONOJOB_H
#define _GPXE_MONOJOB_H

/** @file
 *
 * Single foreground job
 *
 */

#include <gpxe/interface.h>

struct monojob {
	struct interface intf;
	int rc;
};

extern struct monojob monojob;

extern int monojob_wait ( const char *string );

#endif /* _GPXE_MONOJOB_H */
