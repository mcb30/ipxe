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

/** A Linda general scalar register */
struct QIB_7220_scalar_pb {
	pseudo_bit_t Value[64];
};
struct QIB_7220_scalar {
	PSEUDO_BIT_STRUCT ( struct QIB_7220_scalar_pb );
};

/** A Linda eager receive descriptor */
struct QIB_7220_RcvEgr_pb {
	pseudo_bit_t Addr[37];
	pseudo_bit_t BufSize[3];
	pseudo_bit_t Reserved[24];
};
struct QIB_7220_RcvEgr {
	PSEUDO_BIT_STRUCT ( struct QIB_7220_RcvEgr_pb );
};

/** Linda memory BAR size */
#define LINDA_BAR0_SIZE 0x400000

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

/** Number of contexts (including kernel context) */
#define LINDA_NUM_CONTEXTS 17

/** PortCfg values for different numbers of contexts */
enum linda_portcfg {
	LINDA_PORTCFG_5CTX = 0,
	LINDA_PORTCFG_9CTX = 1,
	LINDA_PORTCFG_17CTX = 2,
};

/** PortCfg values for different numbers of contexts */
#define LINDA_EAGER_ARRAY_SIZE_5CTX_0 2048
#define LINDA_EAGER_ARRAY_SIZE_5CTX_OTHER 4096
#define LINDA_EAGER_ARRAY_SIZE_9CTX_0 2048
#define LINDA_EAGER_ARRAY_SIZE_9CTX_OTHER 2048
#define LINDA_EAGER_ARRAY_SIZE_17CTX_0 2048
#define LINDA_EAGER_ARRAY_SIZE_17CTX_OTHER 1024

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

/** Linda external parallel bus register addresses */
#define LINDA_EPB_ADDRESS( _channel, _element, _reg ) \
	( (_element) | ( (_channel) << 4 ) | ( (_reg) << 9 ) )
#define LINDA_EPB_ADDRESS_CHANNEL( _address )	( ( (_address) >> 4 ) & 0x1f )
#define LINDA_EPB_ADDRESS_ELEMENT( _address )	( ( (_address) >> 0 ) & 0x0f )
#define LINDA_EPB_ADDRESS_REG( _address )	( ( (_address) >> 9 ) & 0x3f )

/** Linda external parallel bus locations
 *
 * The location is used by the driver to encode both the chip select
 * and the EPB address.
 */
#define LINDA_EPB_LOC( _cs, _channel, _element, _reg) \
	( ( (_cs) << 16 ) | LINDA_EPB_ADDRESS ( _channel, _element, _reg ) )
#define LINDA_EPB_LOC_ADDRESS( _loc )	( (_loc) & 0xffff )
#define LINDA_EPB_LOC_CS( _loc )	( (_loc) >> 16 )

/** Linda external parallel bus 8051 microcontroller register addresses */
#define LINDA_EPB_UC_CHANNEL 6
#define LINDA_EPB_UC_LOC( _reg ) \
	LINDA_EPB_LOC ( LINDA_EPB_CS_8051, LINDA_EPB_UC_CHANNEL, 0, (_reg) )
#define LINDA_EPB_UC_CTL	LINDA_EPB_UC_LOC ( 0 )
#define LINDA_EPB_UC_CTL_WRITE	1
#define LINDA_EPB_UC_CTL_READ	2
#define LINDA_EPB_UC_ADDR_LO	LINDA_EPB_UC_LOC ( 2 )
#define LINDA_EPB_UC_ADDR_HI	LINDA_EPB_UC_LOC ( 3 )
#define LINDA_EPB_UC_DATA	LINDA_EPB_UC_LOC ( 4 )
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
