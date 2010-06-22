#ifndef _IPXE_FCOE_H
#define _IPXE_FCOE_H

/**
 * @file
 *
 * Fibre Channel over Ethernet
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

#include <stdint.h>

/** An FCoE header */
struct fcoe_header {
	/** FCoE frame version */
	uint8_t version;
	/** Reserved */
	uint8_t reserved[12];
	/** Start of Frame marker */
	uint8_t sof;
} __attribute__ (( packed ));

/** FCoE frame version */
#define FCOE_FRAME_VER 0x00

/** Start of Frame marker values */
enum fcoe_sof {
	FCOE_SOF_F = 0x28,	/**< Start of Frame Class F */
	FCOE_SOF_I2 = 0x2d,	/**< Start of Frame Initiate Class 2 */
	FCOE_SOF_N2 = 0x35,	/**< Start of Frame Normal Class 2 */
	FCOE_SOF_I3 = 0x2e,	/**< Start of Frame Initiate Class 3 */
	FCOE_SOF_N3 = 0x36,	/**< Start of Frame Normal Class 3 */
};

/** An FCoE footer */
struct fcoe_footer {
	/** CRC */
	uint32_t crc;
	/** End of frame marker */
	uint8_t eof;
	/** Reserved */
	uint8_t reserved[3];
} __attribute__ (( packed ));

/** End of Frame marker value */
enum fcoe_eof {
	FCOE_EOF_N = 0x41,	/**< End of Frame Normal */
	FCOE_EOF_T = 0x42,	/**< End of Frame Terminate */
	FCOE_EOF_NI = 0x49,	/**< End of Frame Invalid */
	FCOE_EOF_A = 0x50,	/**< End of Frame Abort */
};

#endif /* _IPXE_FCOE_H */
