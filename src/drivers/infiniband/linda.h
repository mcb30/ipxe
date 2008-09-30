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

/** A Linda GPIO register */
struct QIB_7220_GPIO_pb {
	pseudo_bit_t GPIO[16];
	pseudo_bit_t Reserved[48];
};
struct QIB_7220_GPIO {
	PSEUDO_BIT_STRUCT ( struct QIB_7220_GPIO_pb );
};

/** Linda memory BAR size */
#define LINDA_BAR0_SIZE 0x400000

/** Number of ports per Linda card */
#define LINDA_NUM_PORTS 1

/** Base port number */
#define LINDA_PORT_BASE 1

/** Linda I2C SCL line GPIO number */
#define LINDA_GPIO_SCL 0

/** Linda I2C SDA line GPIO number */
#define LINDA_GPIO_SDA 1

#endif /* _LINDA_H */
