#ifndef _HERMON_H
#define _HERMON_H

/** @file
 *
 * Mellanox Hermon Infiniband HCA driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

#include <stdint.h>
#include <gpxe/uaccess.h>
#include "mlx_bitops.h"
#include "MT25408_PRM.h"

/*
 * Hardware constants
 *
 */

/* Ports in existence */
#define HERMON_NUM_PORTS		2
#define HERMON_PORT_BASE		1

/* PCI BARs */
#define HERMON_PCI_CONFIG_BAR		PCI_BASE_ADDRESS_0
#define HERMON_PCI_CONFIG_BAR_SIZE	0x100000
#define HERMON_PCI_UAR_BAR		PCI_BASE_ADDRESS_2

/* Work queue entry and completion queue entry opcodes */
#define HERMON_OPCODE_SEND		0x0a
#define HERMON_OPCODE_RECV_ERROR	0xfe
#define HERMON_OPCODE_SEND_ERROR	0xff

/* HCA command register opcodes */
#define HERMON_HCR_QUERY_DEV_CAP	0x0003
#define HERMON_HCR_QUERY_FW		0x0004
#define HERMON_HCR_INIT_HCA		0x0007
#define HERMON_HCR_CLOSE_HCA		0x0008
#define HERMON_HCR_INIT_PORT		0x0009
#define HERMON_HCR_CLOSE_PORT		0x000a
#define HERMON_HCR_SW2HW_MPT		0x000d
#define HERMON_HCR_WRITE_MTT		0x0011
#define HERMON_HCR_MAP_EQ		0x0012
#define HERMON_HCR_SW2HW_EQ		0x0013
#define HERMON_HCR_HW2SW_EQ		0x0014
#define HERMON_HCR_QUERY_EQ		0x0015
#define HERMON_HCR_SW2HW_CQ		0x0016
#define HERMON_HCR_HW2SW_CQ		0x0017
#define HERMON_HCR_RST2INIT_QP		0x0019
#define HERMON_HCR_INIT2RTR_QP		0x001a
#define HERMON_HCR_RTR2RTS_QP		0x001b
#define HERMON_HCR_RTS2RTS_QP		0x001c
#define HERMON_HCR_2RST_QP		0x0021
#define HERMON_HCR_MAD_IFC		0x0024
#define HERMON_HCR_READ_MCG		0x0025
#define HERMON_HCR_WRITE_MCG		0x0026
#define HERMON_HCR_MGID_HASH		0x0027
#define HERMON_HCR_RUN_FW		0x0ff6
#define HERMON_HCR_DISABLE_LAM		0x0ff7
#define HERMON_HCR_ENABLE_LAM		0x0ff8
#define HERMON_HCR_UNMAP_ICM		0x0ff9
#define HERMON_HCR_MAP_ICM		0x0ffa
#define HERMON_HCR_UNMAP_ICM_AUX	0x0ffb
#define HERMON_HCR_MAP_ICM_AUX		0x0ffc
#define HERMON_HCR_SET_ICM_SIZE		0x0ffd
#define HERMON_HCR_UNMAP_FA		0x0ffe
#define HERMON_HCR_MAP_FA		0x0fff

/* Service types */
#define HERMON_ST_UD			0x03

/* MTUs */
#define HERMON_MTU_2048			0x04

#define HERMON_INVALID_LKEY		0x00000100UL

#define HERMON_PAGE_SIZE		4096

#define HERMON_DB_POST_SND_OFFSET	0x14
#define HERMON_DB_EQ_OFFSET(_eqn)	\
	( 0x800 + HERMON_PAGE_SIZE * ( (_eqn) / 4 ) + 0x08 * ( (_eqn) % 4 ) )

#define HERMON_QP_OPT_PARAM_QKEY	0x00000020UL

#define HERMON_MAP_EQ			( 0UL << 31 )
#define HERMON_UNMAP_EQ			( 1UL << 31 )

#define HERMON_EV_PORT_STATE_CHANGE	0x09

/*
 * Datatypes that seem to be missing from the autogenerated documentation
 *
 */
struct hermonprm_mgm_hash_st {
	pseudo_bit_t reserved0[0x00020];
/* -------------- */
	pseudo_bit_t hash[0x00010];
	pseudo_bit_t reserved1[0x00010];
} __attribute__ (( packed ));

struct hermonprm_mcg_entry_st {
	struct hermonprm_mcg_hdr_st hdr;
	struct hermonprm_mcg_qp_dw_st qp[8];
} __attribute__ (( packed ));

struct hermonprm_cq_db_record_st {
	pseudo_bit_t update_ci[0x00018];
	pseudo_bit_t reserved0[0x00008];
/* -------------- */
	pseudo_bit_t arm_ci[0x00018];
	pseudo_bit_t cmd[0x00003];
	pseudo_bit_t reserved1[0x00001];
	pseudo_bit_t cmd_sn[0x00002];
	pseudo_bit_t reserved2[0x00002];
} __attribute__ (( packed ));

struct hermonprm_send_db_register_st {
	pseudo_bit_t reserved[0x00008];
	pseudo_bit_t qn[0x00018];
} __attribute__ (( packed ));

struct hermonprm_event_db_register_st {
	pseudo_bit_t ci[0x00018];
	pseudo_bit_t reserver[0x00007];
	pseudo_bit_t a[0x00001];
} __attribute__ (( packed ));

struct hermonprm_scalar_parameter_st {
	pseudo_bit_t value_hi[0x00020];
/* -------------- */
	pseudo_bit_t value[0x00020];
} __attribute__ (( packed ));

struct hermonprm_event_mask_st {
	pseudo_bit_t reserved0[0x00020];
/* -------------- */
	pseudo_bit_t completion[0x00001];
	pseudo_bit_t reserved1[0x0008];
	pseudo_bit_t port_state_change[0x00001];
	pseudo_bit_t reserved2[0x00016];
} __attribute__ (( packed ));

struct hermonprm_port_state_change_event_st {
	pseudo_bit_t reserved[0x00020];
	struct hermonprm_port_state_change_st data;
} __attribute__ (( packed ));

/*
 * Wrapper structures for hardware datatypes
 *
 */

struct MLX_DECLARE_STRUCT ( hermonprm_completion_queue_context );
struct MLX_DECLARE_STRUCT ( hermonprm_completion_queue_entry );
struct MLX_DECLARE_STRUCT ( hermonprm_completion_with_error );
struct MLX_DECLARE_STRUCT ( hermonprm_cq_db_record );
struct MLX_DECLARE_STRUCT ( hermonprm_eqc );
struct MLX_DECLARE_STRUCT ( hermonprm_event_db_register );
struct MLX_DECLARE_STRUCT ( hermonprm_event_mask );
struct MLX_DECLARE_STRUCT ( hermonprm_event_queue_entry );
struct MLX_DECLARE_STRUCT ( hermonprm_hca_command_register );
struct MLX_DECLARE_STRUCT ( hermonprm_init_hca );
struct MLX_DECLARE_STRUCT ( hermonprm_init_port );
struct MLX_DECLARE_STRUCT ( hermonprm_mad_ifc );
struct MLX_DECLARE_STRUCT ( hermonprm_mcg_entry );
struct MLX_DECLARE_STRUCT ( hermonprm_mgm_hash );
struct MLX_DECLARE_STRUCT ( hermonprm_mpt );
struct MLX_DECLARE_STRUCT ( hermonprm_mtt );
struct MLX_DECLARE_STRUCT ( hermonprm_port_state_change_event );
struct MLX_DECLARE_STRUCT ( hermonprm_qp_db_record );
struct MLX_DECLARE_STRUCT ( hermonprm_qp_ee_state_transitions );
struct MLX_DECLARE_STRUCT ( hermonprm_query_dev_cap );
struct MLX_DECLARE_STRUCT ( hermonprm_query_fw );
struct MLX_DECLARE_STRUCT ( hermonprm_queue_pair_ee_context_entry );
struct MLX_DECLARE_STRUCT ( hermonprm_scalar_parameter );
struct MLX_DECLARE_STRUCT ( hermonprm_send_db_register );
struct MLX_DECLARE_STRUCT ( hermonprm_ud_address_vector );
struct MLX_DECLARE_STRUCT ( hermonprm_virtual_physical_mapping );
struct MLX_DECLARE_STRUCT ( hermonprm_wqe_segment_ctrl_send );
struct MLX_DECLARE_STRUCT ( hermonprm_wqe_segment_data_ptr );
struct MLX_DECLARE_STRUCT ( hermonprm_wqe_segment_ud );

/*
 * Composite hardware datatypes
 *
 */

struct hermonprm_write_mtt {
	struct hermonprm_scalar_parameter mtt_base_addr;
	struct hermonprm_scalar_parameter reserved;
	struct hermonprm_mtt mtt;
} __attribute__ (( packed ));

#define HERMON_MAX_GATHER 1

struct hermonprm_ud_send_wqe {
	struct hermonprm_wqe_segment_ctrl_send ctrl;
	struct hermonprm_wqe_segment_ud ud;
	struct hermonprm_wqe_segment_data_ptr data[HERMON_MAX_GATHER];
} __attribute__ (( packed ));

#define HERMON_MAX_SCATTER 1

struct hermonprm_recv_wqe {
	struct hermonprm_wqe_segment_data_ptr data[HERMON_MAX_SCATTER];
} __attribute__ (( packed ));

union hermonprm_completion_entry {
	struct hermonprm_completion_queue_entry normal;
	struct hermonprm_completion_with_error error;
} __attribute__ (( packed ));

union hermonprm_event_entry {
	struct hermonprm_event_queue_entry generic;
	struct hermonprm_port_state_change_event port_state_change;
} __attribute__ (( packed ));

union hermonprm_doorbell_register {
	struct hermonprm_send_db_register send;
	struct hermonprm_event_db_register event;
	uint32_t dword[1];
} __attribute__ (( packed ));

union hermonprm_mad {
	struct hermonprm_mad_ifc ifc;
	union ib_mad mad;
} __attribute__ (( packed ));

/*
 * gPXE-specific definitions
 *
 */

/** Hermon device capabilitiess */
struct hermon_dev_cap {
	/** CMPT entry size */
	size_t cmpt_entry_size;
	/** Number of reserved QPs */
	unsigned int reserved_qps;
	/** QP context entry size */
	size_t qpc_entry_size;
	/** Alternate path context entry size */
	size_t altc_entry_size;
	/** Auxiliary context entry size */
	size_t auxc_entry_size;
	/** Number of reserved SRQs */
	unsigned int reserved_srqs;
	/** SRQ context entry size */
	size_t srqc_entry_size;
	/** Number of reserved CQs */
	unsigned int reserved_cqs;
	/** CQ context entry size */
	size_t cqc_entry_size;
	/** Number of reserved EQs */
	unsigned int reserved_eqs;
	/** EQ context entry size */
	size_t eqc_entry_size;
	/** Number of reserved MTTs */
	unsigned int reserved_mtts;
	/** MTT entry size */
	size_t mtt_entry_size;
	/** Number of reserved MRWs */
	unsigned int reserved_mrws;
	/** DMPT entry size */
	size_t dmpt_entry_size;
	/** Number of reserved UARs */
	unsigned int reserved_uars;
};

/** Number of cMPT entries of each type */
#define HERMON_CMPT_MAX_ENTRIES ( 1 << 24 )

/** Hermon ICM memory map entry */
struct hermon_icm_map {
	/** Offset (virtual address within ICM) */
	uint64_t offset;
	/** Length */
	size_t len;
};

/** Discontiguous regions within Hermon ICM */
enum hermon_icm_map_regions {
	HERMON_ICM_QP_CMPT = 0,
	HERMON_ICM_SRQ_CMPT,
	HERMON_ICM_CQ_CMPT,
	HERMON_ICM_EQ_CMPT,
	HERMON_ICM_OTHER,
	HERMON_ICM_NUM_REGIONS
};

/** UAR page for doorbell accesses
 *
 * Pages 0-127 are reserved for event queue doorbells only, so we use
 * page 128.
 */
#define HERMON_UAR_NON_EQ_PAGE	128

/** Maximum number of allocatable MTT entries
 *
 * This is a policy decision, not a device limit.
 */
#define HERMON_MAX_MTTS		64

/** A Hermon MTT descriptor */
struct hermon_mtt {
	/** MTT offset */
	unsigned int mtt_offset;
	/** Number of pages */
	unsigned int num_pages;
	/** MTT base address */
	unsigned int mtt_base_addr;
	/** Offset within page */
	unsigned int page_offset;
};

/** Alignment of Hermon send work queue entries */
#define HERMON_SEND_WQE_ALIGN 128

/** A Hermon send work queue entry */
union hermon_send_wqe {
	struct hermonprm_ud_send_wqe ud;
	uint8_t force_align[HERMON_SEND_WQE_ALIGN];
} __attribute__ (( packed ));

/** A Hermon send work queue */
struct hermon_send_work_queue {
	/** Number of work queue entries, including headroom
	 *
	 * Hermon requires us to leave unused space within the send
	 * WQ, so we create a send WQ with more entries than are
	 * requested in the create_qp() call.
	 */
	unsigned int num_wqes;
	/** Work queue entries */
	union hermon_send_wqe *wqe;
	/** Size of work queue */
	size_t wqe_size;
	/** Doorbell register */
	void *doorbell;
};

/** Alignment of Hermon receive work queue entries */
#define HERMON_RECV_WQE_ALIGN 16

/** A Hermon receive work queue entry */
union hermon_recv_wqe {
	struct hermonprm_recv_wqe recv;
	uint8_t force_align[HERMON_RECV_WQE_ALIGN];
} __attribute__ (( packed ));

/** A Hermon receive work queue */
struct hermon_recv_work_queue {
	/** Work queue entries */
	union hermon_recv_wqe *wqe;
	/** Size of work queue */
	size_t wqe_size;
	/** Doorbell */
	struct hermonprm_qp_db_record doorbell __attribute__ (( aligned (4) ));
};

/** Maximum number of allocatable queue pairs
 *
 * This is a policy decision, not a device limit.
 */
#define HERMON_MAX_QPS		8

/** Base queue pair number */
#define HERMON_QPN_BASE 0x550000

/** A Hermon queue pair */
struct hermon_queue_pair {
	/** Work queue buffer */
	void *wqe;
	/** Size of work queue buffer */
	size_t wqe_size;
	/** MTT descriptor */
	struct hermon_mtt mtt;
	/** Send work queue */
	struct hermon_send_work_queue send;
	/** Receive work queue */
	struct hermon_recv_work_queue recv;
};

/** Maximum number of allocatable completion queues
 *
 * This is a policy decision, not a device limit.
 */
#define HERMON_MAX_CQS		8

/** A Hermon completion queue */
struct hermon_completion_queue {
	/** Completion queue entries */
	union hermonprm_completion_entry *cqe;
	/** Size of completion queue */
	size_t cqe_size;
	/** MTT descriptor */
	struct hermon_mtt mtt;
	/** Doorbell */
	struct hermonprm_cq_db_record doorbell __attribute__ (( aligned (8) ));
};

/** Maximum number of allocatable event queues
 *
 * This is a policy decision, not a device limit.
 */
#define HERMON_MAX_EQS		8

/** A Hermon event queue */
struct hermon_event_queue {
	/** Event queue entries */
	union hermonprm_event_entry *eqe;
	/** Size of event queue */
	size_t eqe_size;
	/** MTT descriptor */
	struct hermon_mtt mtt;
	/** Event queue number */
	unsigned long eqn;
	/** Next event queue entry index */
	unsigned long next_idx;
	/** Doorbell register */
	void *doorbell;
};

/** Number of event queue entries
 *
 * This is a policy decision.
 */
#define HERMON_NUM_EQES		4

/** A Hermon resource bitmask */
typedef uint32_t hermon_bitmask_t;

/** Size of a hermon resource bitmask */
#define HERMON_BITMASK_SIZE(max_entries)				     \
	( ( (max_entries) + ( 8 * sizeof ( hermon_bitmask_t ) ) - 1 ) /	     \
	  ( 8 * sizeof ( hermon_bitmask_t ) ) )

/** A Hermon device */
struct hermon {
	/** PCI configuration registers */
	void *config;
	/** PCI user Access Region */
	void *uar;

	/** Command toggle */
	unsigned int toggle;
	/** Command input mailbox */
	void *mailbox_in;
	/** Command output mailbox */
	void *mailbox_out;

	/** Firmware area in external memory */
	userptr_t firmware_area;
	/** ICM map */
	struct hermon_icm_map icm_map[HERMON_ICM_NUM_REGIONS];
	/** ICM area */
	userptr_t icm;

	/** Event queue */
	struct hermon_event_queue eq;
	/** Reserved LKey
	 *
	 * Used to get unrestricted memory access.
	 */
	unsigned long reserved_lkey;

	/** Completion queue in-use bitmask */
	hermon_bitmask_t cq_inuse[ HERMON_BITMASK_SIZE ( HERMON_MAX_CQS ) ];
	/** Queue pair in-use bitmask */
	hermon_bitmask_t qp_inuse[ HERMON_BITMASK_SIZE ( HERMON_MAX_QPS ) ];
	/** MTT entry in-use bitmask */
	hermon_bitmask_t mtt_inuse[ HERMON_BITMASK_SIZE ( HERMON_MAX_MTTS ) ];

	/** Device capabilities */
	struct hermon_dev_cap cap;

	/** Infiniband devices */
	struct ib_device *ibdev[HERMON_NUM_PORTS];
};

/** Global protection domain */
#define HERMON_GLOBAL_PD		0x123456

/** Memory key prefix */
#define HERMON_MKEY_PREFIX		0x77000000UL

/*
 * HCA commands
 *
 */

#define HERMON_HCR_BASE			0x80680
#define HERMON_HCR_REG(x)		( HERMON_HCR_BASE + 4 * (x) )
#define HERMON_HCR_MAX_WAIT_MS		2000
#define HERMON_MBOX_ALIGN		4096
#define HERMON_MBOX_SIZE		512

/* HCA command is split into
 *
 * bits  11:0	Opcode
 * bit     12	Input uses mailbox
 * bit     13	Output uses mailbox
 * bits 22:14	Input parameter length (in dwords)
 * bits 31:23	Output parameter length (in dwords)
 *
 * Encoding the information in this way allows us to cut out several
 * parameters to the hermon_command() call.
 */
#define HERMON_HCR_IN_MBOX		0x00001000UL
#define HERMON_HCR_OUT_MBOX		0x00002000UL
#define HERMON_HCR_OPCODE( _command )	( (_command) & 0xfff )
#define HERMON_HCR_IN_LEN( _command )	( ( (_command) >> 12 ) & 0x7fc )
#define HERMON_HCR_OUT_LEN( _command )	( ( (_command) >> 21 ) & 0x7fc )

/** Build HCR command from component parts */
#define HERMON_HCR_INOUT_CMD( _opcode, _in_mbox, _in_len,		     \
			     _out_mbox, _out_len )			     \
	( (_opcode) |							     \
	  ( (_in_mbox) ? HERMON_HCR_IN_MBOX : 0 ) |			     \
	  ( ( (_in_len) / 4 ) << 14 ) |					     \
	  ( (_out_mbox) ? HERMON_HCR_OUT_MBOX : 0 ) |			     \
	  ( ( (_out_len) / 4 ) << 23 ) )

#define HERMON_HCR_IN_CMD( _opcode, _in_mbox, _in_len )			     \
	HERMON_HCR_INOUT_CMD ( _opcode, _in_mbox, _in_len, 0, 0 )

#define HERMON_HCR_OUT_CMD( _opcode, _out_mbox, _out_len )		     \
	HERMON_HCR_INOUT_CMD ( _opcode, 0, 0, _out_mbox, _out_len )

#define HERMON_HCR_VOID_CMD( _opcode )					     \
	HERMON_HCR_INOUT_CMD ( _opcode, 0, 0, 0, 0 )

#endif /* _HERMON_H */
