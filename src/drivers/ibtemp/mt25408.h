#ifndef __mt25408_h__
#define __mt25408_h__

#include "MT25408_PRM.h"

#include "ib_mad.h"

#define F_MASK 0xFFFFFFFFULL
#define NUM_RESOURCES_MTTS	7
#define TAVOR_DEVICE_ID		0x5a44
#define TAVOR_BRIDGE_DEVICE_ID 0x5a46
#define ARTAVOR_DEVICE_ID	0x6278
#define HERMON_SDR_ID       0x6340
#define HERMON_DDR_ID       0x634a

#define HERMON_RESET_OFFSET 0xF0010

#define INVALID_WQE_LKEY 0x00000100

/* Command Interface */
/* Global devices commands */
  /* initialization and general commands */
#define XDEV_CMD_INIT_HCA			0x7
#define XDEV_CMD_CLOSE_HCA			0x8
#define XDEV_CMD_INIT_IB		    	0x9
#define XDEV_CMD_CLOSE_IB			0xa
#define XDEV_CMD_INIT_PORT			0xc

  /* TPT commands */
#define XDEV_CMD_SW2HW_MPT			0xd
#define	XDEV_CMD_HW2SW_MPT			0xf
#define	XDEV_CMD_WRITE_MTT			0x11

  /* EQ commands */
#define XDEV_CMD_MAP_EQ				0x12
#define XDEV_CMD_SW2HW_EQ			0x13
#define XDEV_CMD_HW2SW_EQ			0x14

  /* CQ commands */
#define XDEV_CMD_SW2HW_CQ			0x16
#define	XDEV_CMD_HW2SW_CQ			0x17

  /* QP/EE commands */
#define	XDEV_CMD_RST2INIT_QPEE			0x19
#define XDEV_CMD_INIT2RTR_QPEE			0x1a
#define XDEV_CMD_RTR2RTS_QPEE			0x1b
#define XDEV_CMD_2ERR_QPEE			0x1e
#define XDEV_CMD_ERR2RST_QPEE			0x21

  /* special QPs and management commands */
#define XDEV_CMD_MAD_IFC			0x24

  /* multicast commands */
#define XDEV_CMD_READ_MGM			0x25
#define XDEV_CMD_MGID_HASH			0x27

#define XDEV_CMD_POST_DOORBELL			0x41


/* Hermon Commands */

#define HERMON_CMD_QUERY_ADAPTER		0x006
#define HERMON_CMD_WRITE_MCG			0x026 
#define HERMON_CMD_MOD_STAT_CFG	    		0x034
#define HERMON_CMD_QUERY_FW			0x004

#define HERMON_CMD_MAP_FA			0xfff
#define HERMON_CMD_UNMAP_FA			0xffe
#define HERMON_CMD_RUN_FW			0xff6
#define HERMON_CMD_SET_ICM_SIZE	0xffd
#define HERMON_CMD_MAP_ICM_AUX			0xffc
#define HERMON_CMD_UNMAP_ICM_AUX		0xffb
#define HERMON_CMD_MAP_ICM			0xffa
#define HERMON_CMD_UNMAP_ICM			0xff9
#define HERMON_CMD_QUERY_DEV_CAP   		0x003



/*
 * EQ doorbel commands
 */
#define EQ_DBELL_CMD_INC_CONS_IDX 1	/* increment Consumer_indx by one */
#define EQ_DBELL_CMD_ARM_EQ       2	/* Request notifcation for next event (Arm EQ) */
#define EQ_DBELL_CMD_DISARM_CQ    3	/* Disarm CQ (CQ number is specified in EQ_param) */
#define EQ_DBELL_CMD_SET_CONS_IDX 4	/* set Consumer_indx to value of EQ_param */
#define EQ_DBELL_CMD_ALWAYS_ARM   5	/* move EQ to Always Armed state */

/*
 * CQ doorbel commands
 */
#define CQ_DBELL_CMD_INC_CONS_IDX 1
#define CQ_DBELL_CMD_REQ_NOTIF_SOL_UNSOL 2
#define CQ_DBELL_CMD_REQ_NOTIF_SOL 3
#define CQ_DBELL_CMD_SET_CONS_IDX 4
#define CQ_DBELL_CMD_REQ_NOTIF_MULT 5

#define INPRM_BUF_SZ 4096
#define INPRM_BUF_ALIGN 4096
#define OUTPRM_BUF_SZ 4096
#define OUTPRM_BUF_ALIGN 4096
#define PAGE_SIZE 0x1000

/*
 *  sizes of parameter blocks used in certain
 *	commands.
 *  TODO: replace them with sizeof
 *  operators of the appropriate structs
 */
#define SW2HW_MPT_IBUF_SZ	 MT_STRUCT_SIZE(hermonprm_mpt_st)
#define WRITE_MTT_IBUF_SZ	 MT_STRUCT_SIZE(hermonprm_mtt_st)
#define SW2HW_EQ_IBUF_SZ	 MT_STRUCT_SIZE(hermonprm_eqc_st)
#define SET_PORT_IBUF_SZ	 MT_STRUCT_SIZE(hermonprm_init_port_st)
#define INIT_IB_IBUF_SZ		 MT_STRUCT_SIZE(hermonprm_init_ib_st)
#define SW2HW_CQ_IBUF_SZ	 MT_STRUCT_SIZE(hermonprm_completion_queue_context_st)
#define QPCTX_IBUF_SZ		 MT_STRUCT_SIZE(hermonprm_queue_pair_ee_context_entry_st)

#define EQN 0
#define UAR_IDX 128

#define QPC_OFFSET 0
#define CQC_OFFSET (QPC_OFFSET + 0x100000)
#define EQPC_OFFSET (CQC_OFFSET + 0x100000)
#define EQC_OFFSET (EQPC_OFFSET + 0x100000)
#define MC_BASE_OFFSET (EQC_OFFSET + 0x100000)
#define MPT_BASE_OFFSET (MC_BASE_OFFSET + 0x100000)
#define MTT_BASE_OFFSET (MPT_BASE_OFFSET + 0x100000)
#define EQ0_OFFSET 0x800

#define LOG2_QPS 7
#define LOG2_CQS 8
#define LOG2_EQS 6
#define LOG2_MC_ENTRY 6		/* 8 QPs per group */
#define LOG2_MC_GROUPS 3	/* 8 groups */
#define LOG2_MPT_ENTRIES 5

/* new */
#define CMPT_ENTRIES_PER_RESOURCE (1 << 24)

/* size checked */
#define LOG2_EQ_SZ 5
#define LOG2_CQ_SZ 5

#define NUM_PORTS 2

#define EQE_OWNER_OFFSET 31
#define EQE_OWNER_VAL_HW 0x80

#define CQE_OWNER_OFFSET 31
#define CQE_OWNER_VAL_HW 0x80

#define POST_RCV_OFFSET 0x18
#define POST_SND_OFFSET 0x10
#define CQ_DBELL_OFFSET 0x20
#define EQ_DBELL_OFFSET 0x28

#define CQE_ERROR_OPCODE 0xfe

#define OWNER_HW 1
#define OWNER_SW 0

#define MAX_GATHER 1		/* max gather entries used in send */
#define MAX_SCATTER 2

#define LOG2_MADS_SND_CQ_SZ LOG2_CQ_SZ
#define LOG2_MADS_RCV_CQ_SZ LOG2_CQ_SZ
#define LOG2_IPOIB_SND_CQ_SZ LOG2_CQ_SZ
#define LOG2_IPOIB_RCV_CQ_SZ LOG2_CQ_SZ

/* TODO: */
#define NUM_MADS_SND_CQES (1<<LOG2_MADS_SND_CQ_SZ)
#define NUM_MADS_RCV_CQES (1<<LOG2_MADS_RCV_CQ_SZ)
#define NUM_IPOIB_SND_CQES (1<<LOG2_IPOIB_SND_CQ_SZ)
#define NUM_IPOIB_RCV_CQES (1<<LOG2_IPOIB_RCV_CQ_SZ)

/* work queues must be 2^n size with n=0.. */
#define NUM_MADS_RCV_WQES (1<<1)
#define NUM_IPOIB_RCV_WQES (1<<1)

#if NUM_MADS_RCV_WQES > NUM_IPOIB_RCV_WQES
#define MAX_RCV_WQES NUM_MADS_RCV_WQES
#else
#define MAX_RCV_WQES NUM_IPOIB_RCV_WQES
#endif

#define NUM_MADS_SND_WQES 1
#define NUM_IPOIB_SND_WQES 1

#if NUM_MADS_SND_WQES > NUM_IPOIB_SND_WQES
#define MAX_SND_WQES NUM_MADS_SND_WQES
#else
#define MAX_SND_WQES NUM_IPOIB_SND_WQES
#endif

/* uar context indexes */
enum {
	MADS_RCV_CQ_ARM_DB_IDX,
	MADS_SND_CQ_ARM_DB_IDX,
	IPOIB_RCV_CQ_ARM_DB_IDX,
	IPOIB_SND_CQ_ARM_DB_IDX,
	MADS_SND_QP_DB_IDX,
	IPOIB_SND_QP_DB_IDX,
	GROUP_SEP_IDX,
	START_UNMAPPED_DB_IDX,
	/* --------------------------
	   unmapped doorbell records
	   -------------------------- */
	END_UNMAPPED_DB_IDX = 505,
	MADS_RCV_QP_DB_IDX = 506,
	IPOIB_RCV_QP_DB_IDX = 507,
	MADS_RCV_CQ_CI_DB_IDX = 508,
	MADS_SND_CQ_CI_DB_IDX = 509,
	IPOIB_RCV_CQ_CI_DB_IDX = 510,
	IPOIB_SND_CQ_CI_DB_IDX = 511
};

/* uar resources types */
enum {
	UAR_RES_INVALID = 0x0,	/* Invalid (not allocated) DoorBell record */
	UAR_RES_CQ_SET_CI = 0x1,	/* CQ SET_CI DoorBell record */
	UAR_RES_CQ_ARM = 0x2,	/* CQ ARM DoorBell record */
	UAR_RES_SQ_DBELL = 0x3,	/* Send Queue DoorBell record */
	UAR_RES_RQ_DBELL = 0x4,	/* Receive Queue DoorBell record */
	UAR_RES_SRQ_DBELL = 0x5,	/* Shared Receive Queue DoorBell record */
	UAR_RES_GROUP_SEP = 0x7	/* Group Separator record */
};

enum {
	TS_RC,
	TS_UC,
	TS_RD,
	TS_UD,
	TS_MLX
};

enum {
	PM_STATE_ARMED = 0,
	PM_STATE_REARM = 1,
	PM_STATE_MIGRATED = 3
};

enum {
	DOORBEL_RES_SQ = 3,
	DOORBEL_RES_RQ = 4,
	DOORBEL_RES_SRQ = 5
};

struct ib_buffers_st {
	__u8 send_mad_buf[NUM_MADS_SND_WQES][MAD_BUF_SZ];
	__u8 rcv_mad_buf[NUM_MADS_RCV_WQES][MAD_BUF_SZ + GRH_SIZE];
	__u8 ipoib_rcv_buf[NUM_IPOIB_RCV_WQES][IPOIB_RCV_BUF_SZ + GRH_SIZE];
	__u8 ipoib_rcv_grh_buf[NUM_IPOIB_RCV_WQES][IPOIB_RCV_BUF_SZ];
	__u8 send_ipoib_buf[NUM_IPOIB_SND_WQES][IPOIB_SND_BUF_SZ];
};

struct pcidev {
	unsigned long bar[6];
	__u32 dev_config_space[64];
	struct pci_device *dev;
	__u8 bus;
	__u8 devfn;
};

struct dev_pci_struct {
	struct pcidev dev;
	struct pcidev br;
	void *cr_space;
	void *uar;
};


struct cq_event_type_st {
	__u32	cqn;
	__u32	rsvd[5];
};

union event_data_u {
	u32	raw[6];
	struct cq_event_type_st cq;
};

struct eqe_t {
	__u8 rsvd1;
	__u8 event_type;
	__u8 rsvd2;
	__u8 event_sub_type;
	union event_data_u edata;
	__u8 rsvd3[3];
	__u8 owner;
} __attribute__ ((packed));

struct eq_st {
	__u8 eqn;
	__u32 cons_counter;
	__u32 eq_size;
	__u32 *eq_doorbell;
	struct eqe_t *eq_buf;
};

enum qp_state_e {
	QP_STATE_RST = 0,
	QP_STATE_INIT = 1,
	QP_STATE_RTR = 2,
	QP_STATE_RTS = 3,
	QP_STATE_SQEr = 4,
	QP_STATE_SQD = 5,
	QP_STATE_ERR = 6,
	QP_STATE_SQDING = 7,
	QP_STATE_SUSPEND = 9
};

struct memory_pointer_st {
	__u32 byte_count;
	__u32 lkey;
	__u32 local_addr_h;
	__u32 local_addr_l;
} __attribute__ ((packed));

/* receive wqe descriptor */
struct recv_wqe_st {
	/* part referenced by hardware */
	struct memory_pointer_st mpointer[MAX_SCATTER];
} __attribute__ ((packed));

struct recv_wqe_cont_st {
	struct recv_wqe_st wqe;

	struct udqp_st *qp;	/* qp this wqe is used with */
} __attribute__ ((packed));

#define RECV_WQE_U_ALIGN 64
union recv_wqe_u {
	struct recv_wqe_st wqe;
} __attribute__ ((packed));

struct send_doorbell_st {
	__u8 raw[MT_STRUCT_SIZE(hermonprm_send_doorbell_st)];
} __attribute__ ((packed));

struct next_control_seg_st {
	__u32 owner_opcode;
	__u32 ds;
	__u32 flags;
	__u32 immediate;
} __attribute__ ((packed));

struct address_vector_st {
	__u32	fl_portn_pd;
	__u8    rsvd0;
	__u8	g_mlid;
	__u16	rlid;
	__u8    rsvd1;
	__u8	mgid_index;
	__u8	stat_rate;
	__u8	hop_limit;
	__u32	sl_tclass_flow_label;
	__u32	rgid[4];
} __attribute__ ((packed));

struct ud_seg_st {
	struct address_vector_st	av;
	__u32				dest_qp;
	__u32				qkey;
	__u32				rsvd[2];
} __attribute__ ((packed));

struct ud_send_wqe_st {
	struct next_control_seg_st next;	/* 16 bytes */
	struct ud_seg_st udseg;	/* 48 bytes */
	struct memory_pointer_st mpointer[MAX_GATHER];	/* 16 * MAX_GATHER bytes */
} __attribute__ ((packed));

struct ude_send_wqe_cont_st {
	struct ud_send_wqe_st wqe;
} __attribute__ ((packed));

#define UD_SEND_WQE_U_ALIGN 128
union ud_send_wqe_u {
	u8	align[UD_SEND_WQE_U_ALIGN];
	struct ude_send_wqe_cont_st wqe_cont;
} __attribute__ ((packed));

struct ud_av_st {
	struct address_vector_st av;
	__u32 dest_qp;		/* destination qpn */
	__u32 qkey;
	__u8 next_free;
} __attribute__ ((packed));

union ud_av_u {
	struct ud_av_st ud_av;
} __attribute__ ((packed));

struct udav_st {
	union ud_av_u *av_array;
	__u8 udav_next_free;
};

struct good_cqe_st {
	__u32 flags_qpn;
	__u32 imm_rss_invkey;
	__u32  g_mlpath_rqpn;
	__u16  sl_vid;
	__u16  slid;
	__u32  status;
	__u32  byte_count;
	__u16  wqe_counter;
	__u16  checksum;
	__u8   rsvd[3];
	__u8   owner_opcode;
} __attribute__ ((packed));

struct error_cqe_st {
	__u32	qpn;
	__u32	rsvd1[5];
	__u16	wqe_counter;
	__u8    vendor_error_syndrome;
	__u8	syndrome;
	__u8	rsvd2[3];
	__u8   owner_opcode;
} __attribute__ ((packed));

union cqe_st {
	struct good_cqe_st good_cqe;
	struct error_cqe_st error_cqe;
} __attribute__ ((packed));

struct qp_ctx_t {
	__u32 raw[62];
} __attribute__ ((packed));

struct qp_state_tarnisition_st {
	__u32 opt_param_mask;
	__u32 r1;
	struct qp_ctx_t ctx;
	__u32 r2[64];
} __attribute__ ((packed));

struct cq_dbell_st {
	__u8 raw[MT_STRUCT_SIZE(hermonprm_cq_cmd_doorbell_st)];
} __attribute__ ((packed));

struct mad_ifc_inprm_st {
	union mad_u mad;
} __attribute__ ((packed));

struct wqe_buf_st {
	struct ud_send_wqe_st *sndq;
	struct recv_wqe_st *rcvq;
};

struct mad_buffer_st {
	void *buf;		/* pointer to a 256 byte buffer */
	__u8 owner;		/* sw or hw ownership BUF_OWNER_SW or BUF_OWNER_HW */
};

struct rcv_buf_st {
	void *buf;
	__u8 busy;
};

struct ib_eqe_st {
	__u8 event_type;
	__u32 cqn;
};

struct cq_st {
	__u32 cqn;
	union cqe_st *cq_buf;
	__u32 cons_counter;	/* consuner counter */
	__u8 num_cqes;
	/* keep these two adjacent */
	__u8  db_rec_buf[15];
	__u32 *set_ci_db_record;
	__u32 *arm_db_record;
};

struct udqp_st {
	/* cq used by this QP */
	struct cq_st snd_cq;
	struct cq_st rcv_cq;

	/* QP related data */
	__u32 qpn;		/* QP number */

	__u32 qkey;

	__u8 recv_wqe_cur_free;
	__u8 recv_wqe_alloc_idx;
	__u8 max_recv_wqes;
	void *rcv_bufs[MAX_RCV_WQES];
	union recv_wqe_u *rcv_wq;	/* receive work queue */

	__u8 snd_wqe_cur_free;
	__u8 snd_wqe_alloc_idx;
	__u8 max_snd_wqes;
	void *snd_bufs[MAX_SND_WQES];
	__u16 send_buf_sz;
	__u16 rcv_buf_sz;
	union ud_send_wqe_u *snd_wq;	/* send work queue */
	/* pointers to uar context entries */
	__u32 rcv_uar_context;
	__u16 post_rcv_counter;
	__u32 send_counter;
	__u32 *post_send_dbell;
};


struct mtt_alloc_st {
	unsigned mtt_entry_sz;
	unsigned alloc_ptr;
};

#define MAX_ICM_OBJS 6

struct unmap_icm_st {
	__u64 	va;
	int	npages;
};

struct device_ib_data_st {
	__u32 mkey;
	__u32 pd;
	__u8 port;
	__u8 sl;
	__u32 qkey;
	struct eq_st eq;
	struct udav_st udav;
	struct udqp_st mads_qp;
	struct udqp_st ipoib_qp;
	void *clr_int_addr;
	__u32 clr_int_data;
	__u32 uar_idx;
	void *uar_context_base;
	__u32 error_buf_addr;
	__u32 error_buf_size;
	struct mtt_alloc_st mtt_alloc;


	int num_mappings;
	struct unmap_icm_st icm_arr[MAX_ICM_OBJS];
};


struct query_fw_st {
	__u16 fw_rev_major;
	__u16 fw_rev_minor;
	__u16 fw_rev_subminor;
	__u32 error_buf_offset_h;
	__u32 error_buf_offset_l;
	__u32 error_buf_size;
	__u8 error_buf_bar; 
	__u32 fw_pages;
	__u32 clear_int_base_offset_h; 
	__u32 clear_int_base_offset_l; 
	__u8 clear_int_bar; 
};

struct query_adapter_st {
	__u8 intapin;
};

struct vpm_entry_st {
	__u32 va_h;
	__u32 va_l;
	__u32 pa_h;
	__u32 pa_l;
	__u8 log2_size;
};

#define MAX_VPM_PER_CALL 1

struct map_icm_st {
	__u32 num_vpm;
	struct vpm_entry_st vpm_arr[MAX_VPM_PER_CALL];
};

struct init_hca_st {
	__u32 qpc_base_addr_h;
	__u32 qpc_base_addr_l;
	__u8 log_num_of_qp;

	__u32 srqc_base_addr_h;
	__u32 srqc_base_addr_l;
	__u8 log_num_of_srq;

	__u32 cqc_base_addr_h;
	__u32 cqc_base_addr_l;
	__u8 log_num_of_cq;

	__u32 altc_base_addr_l;
	__u32 altc_base_addr_h;
	__u32 auxc_base_addr_l;
	__u32 auxc_base_addr_h;


	__u32 eqc_base_addr_h;
	__u32 eqc_base_addr_l;
	__u8 log_num_of_eq;

	__u32 rdmardc_base_addr_l;
	__u32 rdmardc_base_addr_h;
	__u8 log_num_rd;  

	__u32 mc_base_addr_h;
	__u32 mc_base_addr_l;
	__u16 log_mc_table_entry_sz; 
	__u8 log_mc_table_hash_sz; 
	__u8 log_mc_table_sz; 

	__u32 cmpt_base_addr_h;
	__u32 cmpt_base_addr_l;
	__u32 dmpt_base_addr_h;
	__u32 dmpt_base_addr_l;
	__u8 log_dmpt_sz;

	__u32 mtt_base_addr_h;
	__u32 mtt_base_addr_l;
	__u8 log_max_uars;
};


struct dev_cap_st {
	__u8 log2_rsvd_qps;
	__u16 qpc_entry_sz;

	__u8 log2_rsvd_srqs;
	__u16 srq_entry_sz;

	__u8 num_rsvd_scqs;

	__u8 log2_rsvd_cqs;
	__u16 cqc_entry_sz;

	__u8 num_rsvd_eqs;
	__u16 eqc_entry_sz;

	__u8 log2_rsvd_mtts;
	__u16 mtt_entry_sz;

	__u8 log2_rsvd_mrws;
	__u16 d_mpt_entry_sz; 
	__u16 c_mpt_entry_sz; 

	/* __u16 rdmardc_entry_sz; */
	/* __u8 log_max_ra_res_qp; */

	__u16 altc_entry_size;
	__u16 auxc_entry_size;

	__u32 max_icm_size_l;
	__u32 max_icm_size_h;

	__u8 uar_sz;
	__u8 num_rsvd_uars;
};

/* Command structure */
typedef enum {
	TRANS_NA,
	TRANS_IMMEDIATE,
	TRANS_MAILBOX
} trans_type_t;

typedef struct {
	__u32 *in_param;	/* holds the virtually contigious buffer of the parameter block passed */
	unsigned int in_param_size;
	trans_type_t in_trans;

	__u32 input_modifier;

	__u32 *out_param;	/* holds the virtually contigious buffer of the parameter block passed */
	unsigned int out_param_size;
	trans_type_t out_trans;

	__u32 opcode;
	__u8 opcode_modifier;
} command_fields_t;

typedef int XHH_cmd_status_t;

/* Functions */

/* Command structure */
static XHH_cmd_status_t cmd_invoke(command_fields_t *cmd_prms);
static int wait_cmdif_free(int *last_toggle);
static int cmdif_is_free(int *is_free);
static void edit_hcr(command_fields_t *cmd_prms, __u32 *buf);
static int get_toggle(int *curr_toggle);


/* Command interface */
static int cmd_init_hca(__u32 *inprm, __u32 in_prm_size);
static int cmd_close_hca(int panic);
static int cmd_sw2hw_eq(__u8 eqn, __u32 inprm_sz);
static int cmd_hw2sw_eq(__u8 eqn);
static int cmd_map_eq(__u8 eqn, __u32 mask, int map);
static int cmd_sw2hw_mpt(__u32 *lkey, __u32 in_key, __u32 *inprm,
			 __u32 inprm_sz);
static int cmd_hw2sw_mpt(__u32 key);
static int cmd_init_ib(__u32 port, void *inprm, __u32 inprm_sz);
static int cmd_close_ib(__u32 port);
static int cmd_sw2hw_cq(__u32 cqn, __u32 *inprm, __u32 inprm_sz);
static int cmd_hw2sw_cq(__u32 cqn);
static int cmd_rst2init_qpee(__u32 qpn, __u32 *inprm, __u32 inprm_sz);
static int cmd_init2rtr_qpee(__u32 qpn, __u32 *inprm, __u32 inprm_sz);
static int cmd_rtr2rts_qpee(__u32 qpn, __u32 *inprm, __u32 inprm_sz);
static int cmd_2rst_qpee(__u32 qpn);
static int cmd_2err_qpee(__u32 qpn);
static int cmd_post_doorbell(void *inprm, __u32 offset);
static int cmd_mad_ifc(void *inprm, struct ib_mad_st *mad, __u8 port);
static int cmd_write_mcg( /*struct mg_member_layout_st */ void *mg,
			 __u16 index);
static int cmd_mgid_hash(__u8 *gid, __u16 *mgid_hash_p);
static int cmd_write_mtt(void *inprm, int num_mtt);  
static int cmd_mod_stat_cfg(void);
static int cmd_query_fw(struct query_fw_st *qfw);
static int cmd_query_adapter(struct query_adapter_st *qa);
static int cmd_map_fa(struct map_icm_st *map_fa_p);
static int cmd_unmap_fa(void);
static int cmd_run_fw(void);
static int cmd_set_icm_size(unsigned long long icm_size, __u32 *aux_pages_p);
static int cmd_map_icm_aux(struct map_icm_st *map_icm_aux_p);
static int cmd_unmap_icm_aux(void);
static int cmd_map_icm(struct map_icm_st *map_icm_p);
static int cmd_unmap_icm(__u64 va, int npages);
static int cmd_query_dev_cap(struct dev_cap_st *dev_cap_p);


/* Other Functions */
static int create_udqp(struct udqp_st *qp);
static int destroy_udqp(struct udqp_st *qp);
static void *get_send_wqe_buf(void *wqe, __u8 index);
static void *get_rcv_wqe_buf(void *wqe, __u8 index);

static struct recv_wqe_st *alloc_rcv_wqe(struct udqp_st *qp);
static int free_wqe(void *wqe);
static int poll_cq(void *cqh, union cqe_st *cqe_p, __u8 *num_cqes);
static int poll_eq(struct ib_eqe_st *ib_eqe_p, __u8 *num_eqes);
static int post_rcv_buf(struct udqp_st *qp);
static __u32 dev_get_qpn(void *qph);




#endif				/* __mt25408_h__ */
