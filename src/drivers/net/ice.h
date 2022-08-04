#ifndef _ICE_H
#define _ICE_H

/** @file
 *
 * Intel 100 Gigabit Ethernet network card driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

#include "intelxl.h"

/** BAR size */
#define ICE_BAR_SIZE 0x800000

/******************************************************************************
 *
 * Admin queue
 *
 ******************************************************************************
 */

/** Admin queue version number */
struct ice_admin_version {
	/** Branch identifier */
	uint8_t branch;
	/** Major version number */
	uint8_t major;
	/** Minor version number */
	uint8_t minor;
	/** Patch level */
	uint8_t patch;
} __attribute__ (( packed ));

/** Admin queue Get Version command parameters */
struct ice_admin_version_params {
	/** ROM version */
	uint32_t rom;
	/** Firmware build ID */
	uint32_t build;
	/** Firmware version */
	struct ice_admin_version firmware;
	/** API version */
	struct ice_admin_version api;
} __attribute__ (( packed ));

/** MAC Address description */
struct ice_admin_mac_read_address {
	/** Port number */
	uint8_t port;
	/** Address type */
	uint8_t type;
	/** MAC address */
	uint8_t mac[ETH_ALEN];
} __attribute__ (( packed ));

/** LAN MAC address type */
#define ICE_ADMIN_MAC_READ_TYPE_LAN 0

/** Admin queue Manage MAC Address Read data buffer */
struct ice_admin_mac_read_buffer {
	/** MAC addresses */
	struct ice_admin_mac_read_address mac[4];
} __attribute__ (( packed ));

/** Switching element configuration */
struct ice_admin_switch_config {
	/** Switching element ID and flags */
	uint16_t seid;
	/** Uplink switching element ID */
	uint16_t uplink;
	/** PF/VF number */
	uint16_t func;
} __attribute__ (( packed ));

/** Switching element ID type mask */
#define ICE_ADMIN_SWITCH_TYPE_MASK 0xc000

/** Virtual Station Interface element type */
#define ICE_ADMIN_SWITCH_TYPE_VSI 0x8000

/** Admin queue Get Switch Configuration data buffer */
struct ice_admin_switch_buffer {
	/** Switch configuration */
	struct ice_admin_switch_config cfg[1];
};

/** Admin queue Query Default Scheduling Tree Topology command */
#define ICE_ADMIN_SCHEDULE 0x0400

/** Admin queue Query Default Scheduling Tree Topology command parameters */
struct ice_admin_schedule_params {
	/** Reserved */
	uint8_t reserved_a;
	/** Total branches */
	uint8_t branches;
	/** Reserved */
	uint8_t reserved_b[6];
} __attribute__ (( packed ));

/** Transmit scheduler configuration */
struct ice_schedule_config {
	/** Node type */
	uint8_t type;
	/** Valid sections */
	uint8_t sections;
	/** Generic information */
	uint8_t generic;
	/** Flags */
	uint8_t flags;
	/** Committed bandwidth profile ID */
	uint16_t commit_id;
	/** Committeed bandwidth weight */
	uint16_t commit_weight;
	/** Excess bandwidth profile ID */
	uint16_t excess_id;
	/** Excess bandwidth weight */
	uint16_t excess_weight;
	/** Shared rate limit profile ID */
	uint16_t shared;
	/** Reserved */
	uint16_t reserved;
} __attribute__ (( packed ));

/** Transmit scheduler configuration generic section is valid */
#define ICE_SCHEDULE_GENERIC 0x01

/** Transmit scheduler configuration committed bandwidth section is valid */
#define ICE_SCHEDULE_COMMIT 0x02

/** Transmit scheduler configuration excess bandwidth section is valid */
#define ICE_SCHEDULE_EXCESS 0x04

/** Transmit scheduler configuration default weight */
#define ICE_SCHEDULE_WEIGHT 0x0004

/** Admin queue Query Default Scheduling Tree Topology node */
struct ice_admin_schedule_node {
	/** Parent TEID */
	uint32_t parent;
	/** Node TEID */
	uint32_t teid;
	/** Scheduler configuration */
	struct ice_schedule_config config;
} __attribute__ (( packed ));

/** Admin queue Query Default Scheduling Tree Topology branch */
struct ice_admin_schedule_branch {
	/** Reserved */
	uint8_t reserved_a[4];
	/** Number of nodes */
	uint16_t count;
	/** Reserved */
	uint8_t reserved_b[2];
	/** Nodes */
	struct ice_admin_schedule_node node[0];
} __attribute__ (( packed ));

/** Admin queue Query Default Scheduling Tree Topology data buffer */
union ice_admin_schedule_buffer {
	/** Branches */
	struct ice_admin_schedule_branch branch[0];
	/** Padding */
	uint8_t pad[INTELXL_ALIGN];
} __attribute__ (( packed ));

/** Admin queue Get Link Status command parameters */
struct ice_admin_link_params {
	/** Logical port number */
	uint8_t port;
	/** Reserved */
	uint8_t reserved_a;
	/** Link status notification */
	uint8_t notify;
	/** Reserved */
	uint8_t reserved_b[13];
} __attribute__ (( packed ));

/** Admin queue Get Link Status data buffer */
struct ice_admin_link_buffer {
	/** Topology conflicts */
	uint8_t conflict;
	/** Configuration errors */
	uint8_t error;
	/** Link status */
	uint8_t status;
	/** Reserved */
	uint8_t reserved_a[3];
	/** Maximum frame size */
	uint16_t mfs;
	/** Reserved */
	uint8_t reserved_b[2];
	/** Link speed */
	uint16_t speed;
	/** Reserved */
	uint8_t reserved_c[20];
} __attribute__ (( packed ));

/** Admin queue Add Transmit Queues command */
#define ICE_ADMIN_ADD_TXQ 0x0c30

/** Admin queue Add Transmit Queues command parameters */
struct ice_admin_add_txq_params {
	/** Number of queue groups */
	uint8_t count;
	/** Reserved */
	uint8_t reserved[7];
} __attribute__ (( packed ));

/** Admin queue Add Transmit Queues data buffer */
struct ice_admin_add_txq_buffer {
	/** Parent TEID */
	uint32_t parent;
	/** Number of queues */
	uint8_t count;
	/** Reserved */
	uint8_t reserved_a[3];
	/** Transmit queue ID */
	uint16_t id;
	/** Reserved */
	uint8_t reserved_b[2];
	/** Queue TEID */
	uint32_t teid;
	/** Base address */
	uint64_t base_port;
	/** PF number and queue type */
	uint16_t pf_type;
	/** Source VSI */
	uint16_t vsi;
	/** Reserved */
	uint8_t reserved_c[5];
	/** Queue length */
	uint16_t len;
	/** Flags */
	uint16_t flags;
	/** Reserved */
	uint8_t reserved_d[3];
	/** Scheduler configuration */
	struct ice_schedule_config config;
} __attribute__ (( packed ));

/** Transmit queue base address and port number */
#define ICE_TXQ_BASE_PORT( addr, port ) \
	( ( (addr) >> 7 ) | ( ( ( uint64_t ) (port) ) << 57 ) )

/** Transmit queue PF number */
#define ICE_TXQ_PF_TYPE( pf ) ( ( (pf) << 1 ) | ( 0x2 << 14 ) )

/** Transmit queue length */
#define ICE_TXQ_LEN( count ) ( (count) >> 1 )

/** Transmit queue uses TSO */
#define ICE_TXQ_FL_TSO 0x0001

/** Transmit queue uses legacy mode*/
#define ICE_TXQ_FL_LEGACY 0x1000

/** Admin queue Disable Transmit Queues command */
#define ICE_ADMIN_DISABLE_TXQ 0x0c31

/** Admin queue Disable Transmit Queues command parameters */
struct ice_admin_disable_txq_params {
	/** Flags */
	uint8_t flags;
	/** Number of queue groups */
	uint8_t count;
	/** Reserved */
	uint8_t reserved_a;
	/** Timeout */
	uint8_t timeout;
	/** Reserved */
	uint8_t reserved_b[4];
} __attribute__ (( packed ));

/** Disable queue and flush pipe */
#define ICE_TXQ_FL_FLUSH 0x08

/** Disable queue timeout */
#define ICE_TXQ_TIMEOUT 0xc8

/** Admin queue Disable Transmit Queues data buffer */
struct ice_admin_disable_txq_buffer {
	/** Parent TEID */
	uint32_t parent;
	/** Number of queues */
	uint8_t count;
	/** Reserved */
	uint8_t reserved;
	/** Transmit queue ID */
	uint16_t id;
} __attribute__ (( packed ));

/** Admin queue command parameters */
union ice_admin_params {
	/** Additional data buffer command parameters */
	struct intelxl_admin_buffer_params buffer;
	/** Get Version command parameters */
	struct ice_admin_version_params version;
	/** Query Default Scheduling Tree Topology command parameters */
	struct ice_admin_schedule_params sched;
	/** Get Link Status command parameters */
	struct ice_admin_link_params link;
	/** Add Transmit Queue command parameters */
	struct ice_admin_add_txq_params add_txq;
	/** Disable Transmit Queue command parameters */
	struct ice_admin_disable_txq_params disable_txq;
} __attribute__ (( packed ));

/** Admin queue data buffer */
union ice_admin_buffer {
	/** Manage MAC Address Read data buffer */
	struct ice_admin_mac_read_buffer mac_read;
	/** Get Switch Configuration data buffer */
	struct ice_admin_switch_buffer sw;
	/** Query Default Scheduling Tree Topology data buffer */
	union ice_admin_schedule_buffer sched;
	/** Get Link Status data buffer */
	struct ice_admin_link_buffer link;
	/** Add Transmit Queue data buffer */
	struct ice_admin_add_txq_buffer add_txq;
	/** Disable Transmit Queue data buffer */
	struct ice_admin_disable_txq_buffer disable_txq;
	/** Alignment padding */
	uint8_t pad[INTELXL_ALIGN];
} __attribute__ (( packed ));

/******************************************************************************
 *
 * Transmit and receive datapaths
 *
 ******************************************************************************
 */

/** Global Receive Queue Control Register */
#define ICE_QRX_CTRL 0x120000

/** Receive Queue Context Registers */
#define ICE_QRX_CONTEXT(x) ( 0x280000 + ( 0x2000 * (x) ) )

/** Receive Queue Tail Register */
#define ICE_QRX_TAIL 0x290000

/** Transmit Comm Scheduler Queue Doorbell Register */
#define ICE_QTX_COMM_DBELL 0x2c0000

/** Queue Context Flex Extension Register */
#define ICE_QRX_FLXP_CNTXT 0x480000
#define ICE_QRX_FLXP_CNTXT_RXDID_IDX(x) \
	( (x) << 0 )					/**< RX profile */
#define ICE_QRX_FLXP_CNTXT_RXDID_IDX_LEGACY_32 \
	ICE_QRX_FLXP_CNTXT_RXDID_IDX ( 1 )		/**< 32-byte legacy */
#define ICE_QRX_FLXP_CNTXT_RXDID_PRIO(x) \
	( (x) << 8 )					/**< Priority */
#define ICE_QRX_FLXP_CNTXT_RXDID_PRIO_MAX \
	ICE_QRX_FLXP_CNTXT_RXDID_PRIO ( 7 )		/**< Maximum priority */

/**
 * Initialise descriptor ring
 *
 * @v ring		Descriptor ring
 * @v count		Number of descriptors
 * @v len		Length of a single descriptor
 */
static inline __attribute__ (( always_inline)) void
ice_init_ring ( struct intelxl_ring *ring, unsigned int count, size_t len ) {

	ring->len = ( count * len );
}

/******************************************************************************
 *
 * Top level
 *
 ******************************************************************************
 */

/** Function Requester ID Information Register */
#define ICE_PFFUNC_RID 0x09e880
#define ICE_PFFUNC_RID_FUNC_NUM(x) \
	( ( (x) >> 0 ) & 0x7 )				/**< Function number */

/** PF LAN Port Number Register */
#define ICE_PFGEN_PORTNUM 0x1d2400
#define ICE_PFGEN_PORTNUM_PORT_NUM(x) \
	( ( (x) >> 0 ) & 0x7 )				/**< Port number */

/** Transmit Queue Interrupt Cause Control Register */
#define ICE_QINT_TQCTL 0x140000
#define ICE_QINT_TQCTL_ITR_INDX(x) ( (x) << 11 )	/**< Throttling */
#define ICE_QINT_TQCTL_ITR_INDX_NONE \
	ICE_QINT_TQCTL_ITR_INDX ( 0x3 )			/**< No throttling */
#define ICE_QINT_TQCTL_CAUSE_ENA	0x40000000UL	/**< Enable */

/** Receive Queue Interrupt Cause Control Register */
#define ICE_QINT_RQCTL 0x150000
#define ICE_QINT_RQCTL_ITR_INDX(x) ( (x) << 11 )	/**< Throttling */
#define ICE_QINT_RQCTL_ITR_INDX_NONE \
	ICE_QINT_RQCTL_ITR_INDX ( 0x3 )			/**< No throttling */
#define ICE_QINT_RQCTL_CAUSE_ENA	0x40000000UL	/**< Enable */

/** Global Interrupt Dynamic Control Register */
#define ICE_GLINT_DYN_CTL 0x160000

#endif /* _ICE_H */
