#ifndef _LINDA_H
#define _LINDA_H

/**
 * @file
 *
 * QLogic Linda Infiniband HCA
 *
 */

#define BITOPS_LITTLE_ENDIAN
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

/** GUID offset within EEPROM */
#define LINDA_EEPROM_GUID_OFFSET 3

/** GUID size within EEPROM */
#define LINDA_EEPROM_GUID_SIZE 8

/** Board serial number offset within EEPROM */
#define LINDA_EEPROM_SERIAL_OFFSET 12

/** Board serial number size within EEPROM */
#define LINDA_EEPROM_SERIAL_SIZE 12

/** Maximum time for wait for external parallel bus request, in us */
#define LINDA_EPB_REQUEST_MAX_WAIT_US 500

/** Maximum time for wait for external parallel bus transaction, in us */
#define LINDA_EPB_XACT_MAX_WAIT_US 500

/** Linda external parallel bus chip selects */
#define LINDA_EPB_CS_SERDES 1
#define LINDA_EPB_CS_8051 2

/** Linda external parallel bus read/write operations */
#define LINDA_EPB_WRITE 0
#define LINDA_EPB_READ 1

/** Linda external parallel bus microcontroller registers and values */
#define LINDA_EPB_LOC( _chn, _elt, _reg)				\
	( ( (_elt) & 0xf ) | ( ( (_chn) & 7 ) << 4 ) |			\
	  ( ( (_reg) & 0x3f ) << 9 ) )

#define LINDA_EPB_UC_CTL	LINDA_EPB_LOC ( 6, 0, 0 )
#define LINDA_EPB_UC_CTL_WRITE	1
#define LINDA_EPB_UC_CTL_READ	2
#define LINDA_EPB_UC_ADDR_LO	LINDA_EPB_LOC ( 6, 0, 2 )
#define LINDA_EPB_UC_ADDR_HI	LINDA_EPB_LOC ( 6, 0, 3 )
#define LINDA_EPB_UC_DATA	LINDA_EPB_LOC ( 6, 0, 4 )

#define LINDA_EPB_UC_CHUNK_SIZE	64

extern uint8_t linda_ib_fw[8192];

/** Maximum time to wait for "trim done" signal, in ms */
#define LINDA_TRIM_DONE_MAX_WAIT_MS 1000

/** Linda link states */
enum linda_link_state {
	LINDA_LINK_STATE_DOWN = 0,
	LINDA_LINK_STATE_INIT = 1,
	LINDA_LINK_STATE_ARM = 2,
	LINDA_LINK_STATE_ACTIVE = 3,
	LINDA_LINK_STATE_ACT_DEFER = 4,
};

#endif /* _LINDA_H */
