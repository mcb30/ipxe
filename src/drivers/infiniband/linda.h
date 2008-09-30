#ifndef _LINDA_H
#define _LINDA_H

/**
 * @file
 *
 * QLogic Linda Infiniband HCA
 *
 */

#define BITOPS_LE64
#include <gpxe/bitops.h>
#include "qib_7220_regs.h"

struct ib_device;

/** Linda memory BAR size */
#define LINDA_BAR0_SIZE 0x400000

/** Number of ports per Linda card */
#define LINDA_NUM_PORTS 1

/** Base port number */
#define LINDA_PORT_BASE 1

/** A Linda HCA */
struct linda {
	/** Registers */
	void *regs;
	/** Infiniband devices */
	struct ib_device *ibdev[LINDA_NUM_PORTS];
};

#endif /* _LINDA_H */
