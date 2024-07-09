#ifndef _GVE_H
#define _GVE_H

/** @file
 *
 * Google Virtual Ethernet network driver
 *
 * The Google Virtual Ethernet NIC (GVE or gVNIC) is found only in
 * Google Cloud instances.  There is essentially zero documentation
 * available beyond the mostly uncommented source code in the Linux
 * kernel.
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

#include <stdint.h>
#include <ipxe/pci.h>
#include <ipxe/if_ether.h>

/** Configuration BAR */
#define GVE_CFG_BAR PCI_BASE_ADDRESS_0

/** Configuration BAR size
 *
 * All registers within the configuration BAR are big-endian.
 */
#define GVE_CFG_SIZE 0x1000

/** Device status */
#define GVE_CFG_DEVSTAT 0x0000
#define GVE_CFG_DEVSTAT_RESET 0x00000010UL	/**< Device is reset */

/** Driver status */
#define GVE_CFG_DRVSTAT 0x0004
#define GVE_CFG_DRVSTAT_RUN 0x00000001UL	/**< Run admin queue */
#define GVE_CFG_DRVSTAT_RESET 0x00000002UL	/**< Reset device */

/** Maximum time to wait for reset */
#define GVE_RESET_MAX_WAIT_MS 5000

/** Admin queue page frame number (for older devices) */
#define GVE_CFG_AQ_PFN 0x0010

/** Admin queue doorbell */
#define GVE_CFG_AQ_DB 0x0014

/** Admin queue event counter */
#define GVE_CFG_AQ_EVT 0x0018

/** Driver version (8-bit register) */
#define GVE_CFG_VERSION 0x001f

/** Admin queue base address high 32 bits */
#define GVE_CFG_AQ_BASE_HI 0x0020

/** Admin queue base address low 32 bits */
#define GVE_CFG_AQ_BASE_LO 0x0024

/** Admin queue base address length (16-bit register) */
#define GVE_CFG_AQ_LEN 0x0028

/** Doorbell BAR */
#define GVE_DB_BAR PCI_BASE_ADDRESS_2

/** Doorbell BAR size */
#define GVE_DB_SIZE 0x100000

/** Admin queue alignment */
#define GVE_AQ_ALIGN 0x1000

/** Admin queue length
 *
 * This is theoretically a policy decision.  However, older revisions
 * of the hardware seem to have only the "admin queue page frame
 * number" register and no "admin queue length" register, with the
 * implication that the admin queue must be exactly one page in
 * length.
 *
 * Choose to use a one page (4kB) admin queue for both older and newer
 * versions of the hardware, to minimise variability.
 */
#define GVE_AQ_LEN 0x1000

/** Admin queue entry header
 *
 * All values within admin queue entries are big-endian.
 */
struct gve_aq_header {
	/** Opcode */
	uint32_t opcode;
	/** Status */
	uint32_t status;
} __attribute__ (( packed ));

/** Command succeeded */
#define GVE_AQ_STATUS_OK 0x00000001

/** Describe device command */
#define GVE_AQ_DESCRIBE 0x0001

/** Describe device command */
struct gve_aq_describe {
	/** Header */
	struct gve_aq_header hdr;
	/** Descriptor buffer address */
	uint64_t addr;
	/** Descriptor version */
	uint32_t ver;
	/** Descriptor maximum length */
	uint32_t len;
} __attribute__ (( packed ));

/** Device descriptor version */
#define GVE_AQ_DESCRIBE_VER 1

/** Device descriptor */
struct gve_device_descriptor {
	/** Reserved */
	uint8_t reserved_a[16];
	/** Maximum transmit unit */
	uint16_t mtu;
	/** Reserved */
	uint8_t reserved_b[6];
	/** MAC address */
	uint8_t mac[ETH_ALEN];
	/** Reserved */
	uint8_t reserved_c[10];
} __attribute__ (( packed ));

/** An admin queue command */
union gve_aq_command {
	/** Header */
	struct gve_aq_header hdr;
	/** Describe device */
	struct gve_aq_describe describe;
	/** Padding */
	uint8_t pad[64];
};

/** Number of admin queue commands */
#define GVE_AQ_COUNT ( GVE_AQ_LEN / sizeof ( union gve_aq_command ) )

/** Admin queue */
struct gve_aq {
	/** Commands */
	union gve_aq_command *cmd;
	/** Producer counter */
	uint32_t prod;
};

/** A Google Virtual Ethernet NIC */
struct gve_nic {
	/** Configuration registers */
	void *cfg;
	/** PCI revision */
	uint8_t revision;
	/** Admin queue */
	struct gve_aq aq;
	/** Device descriptor */
	struct gve_device_descriptor desc;
};

/** Maximum time to wait for admin queue commands */
#define GVE_AQ_MAX_WAIT_MS 1000

#endif /* _GVE_H */
