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
#include <ipxe/dma.h>
#include <ipxe/pci.h>
#include <ipxe/in.h>

/** A Google Cloud MAC address
 *
 * Google Cloud locally assigned MAC addresses encode the local IPv4
 * address in the trailing 32 bits, presumably as a performance
 * optimisation to allow ARP resolution to be skipped by a suitably
 * aware network stack.
 */
struct google_mac {
	/** Reserved */
	uint8_t reserved[2];
	/** Local IPv4 address */
	struct in_addr in;
} __attribute__ (( packed ));

/** General alignment constraint
 *
 * Several data structures have unstated alignment requirements.
 * Conservatively choose to use page alignment to meet these
 * undocumented requirements.
 */
#define GVE_ALIGN 0x1000

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
#define GVE_CFG_ADMIN_PFN 0x0010

/** Admin queue doorbell */
#define GVE_CFG_ADMIN_DB 0x0014

/** Admin queue event counter */
#define GVE_CFG_ADMIN_EVT 0x0018

/** Driver version (8-bit register) */
#define GVE_CFG_VERSION 0x001f

/** Admin queue base address high 32 bits */
#define GVE_CFG_ADMIN_BASE_HI 0x0020

/** Admin queue base address low 32 bits */
#define GVE_CFG_ADMIN_BASE_LO 0x0024

/** Admin queue base address length (16-bit register) */
#define GVE_CFG_ADMIN_LEN 0x0028

/** Doorbell BAR */
#define GVE_DB_BAR PCI_BASE_ADDRESS_2

/** Doorbell BAR size */
#define GVE_DB_SIZE 0x100000

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
#define GVE_ADMIN_LEN 0x1000

/** Admin queue entry header
 *
 * All values within admin queue entries are big-endian.
 */
struct gve_admin_header {
	/** Opcode */
	uint32_t opcode;
	/** Status */
	uint32_t status;
} __attribute__ (( packed ));

/** Command succeeded */
#define GVE_ADMIN_STATUS_OK 0x00000001

/** Describe device command */
#define GVE_ADMIN_DESCRIBE 0x0001

/** Describe device command */
struct gve_admin_describe {
	/** Header */
	struct gve_admin_header hdr;
	/** Descriptor buffer address */
	uint64_t addr;
	/** Descriptor version */
	uint32_t ver;
	/** Descriptor maximum length */
	uint32_t len;
} __attribute__ (( packed ));

/** Device descriptor version */
#define GVE_ADMIN_DESCRIBE_VER 1

/** Device descriptor */
struct gve_device_descriptor {
	/** Reserved */
	uint8_t reserved_a[16];
	/** Maximum transmit unit */
	uint16_t mtu;
	/** Reserved */
	uint8_t reserved_b[6];
	/** MAC address */
	struct google_mac mac;
	/** Reserved */
	uint8_t reserved_c[10];
} __attribute__ (( packed ));

/** Configure device resources command */
#define GVE_ADMIN_CONFIGURE 0x0002

/** Configure device resources command */
struct gve_admin_configure {
	/** Header */
	struct gve_admin_header hdr;
	/** Event counter array */
	uint64_t counters;
	/** IRQ doorbell address */
	uint64_t doorbells;
	/** Number of event counters */
	uint32_t num_counters;
	/** Number of IRQ doorbells */
	uint32_t num_doorbells;
	/** IRQ doorbell stride */
	uint32_t stride;
} __attribute__ (( packed ));

/** Register page list command */
#define GVE_ADMIN_REGISTER 0x0003

/** Register page list command */
struct gve_admin_register {
	/** Header */
	struct gve_admin_header hdr;
	/** Page list ID */
	uint32_t id;
	/** Number of pages */
	uint32_t count;
	/** Address list address */
	uint64_t addr;
	/** Page size */
	uint64_t size;
} __attribute__ (( packed ));

/** Page list ID */
#define GVE_ADMIN_REGISTER_ID 0x69505845UL

/** Unregister page list command */
#define GVE_ADMIN_UNREGISTER 0x0004

/** Unregister page list command */
struct gve_admin_unregister {
	/** Header */
	struct gve_admin_header hdr;
	/** Page list ID */
	uint32_t id;
} __attribute__ (( packed ));

/** Deconfigure device resources command */
#define GVE_ADMIN_DECONFIGURE 0x0009

/** An admin queue command */
union gve_admin_command {
	/** Header */
	struct gve_admin_header hdr;
	/** Describe device */
	struct gve_admin_describe desc;
	/** Configure device resources */
	struct gve_admin_configure conf;
	/** Register page list */
	struct gve_admin_register reg;
	/** Unregister page list */
	struct gve_admin_unregister unreg;
	/** Padding */
	uint8_t pad[64];
};

/** Number of admin queue commands */
#define GVE_ADMIN_COUNT ( GVE_ADMIN_LEN / sizeof ( union gve_admin_command ) )

/** Admin queue */
struct gve_admin {
	/** Commands */
	union gve_admin_command *cmd;
	/** Producer counter */
	uint32_t prod;
	/** DMA mapping */
	struct dma_mapping map;
};

/** Scratch buffer */
struct gve_scratch {
	/** Buffer contents */
	union {
		/** Device descriptor */
		struct gve_device_descriptor desc;
	} *buf;
	/** DMA mapping */
	struct dma_mapping map;
};

/** Event counter */
struct gve_event {
	/** Event counter */
	uint32_t *counter;
	/** DMA mapping */
	struct dma_mapping map;
};

/** A Google Virtual Ethernet NIC */
struct gve_nic {
	/** Configuration registers */
	void *cfg;
	/** PCI revision */
	uint8_t revision;
	/** DMA device */
	struct dma_device *dma;
	/** Scratch buffer */
	struct gve_scratch scratch;
	/** Admin queue */
	struct gve_admin admin;
	/** Event counter */
	struct gve_event event;
};

/** Maximum time to wait for admin queue commands */
#define GVE_ADMIN_MAX_WAIT_MS 1000

#endif /* _GVE_H */
