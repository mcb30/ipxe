/*
  This software is available to you under a choice of one of two
  licenses.  You may choose to be licensed under the terms of the GNU
  General Public License (GPL) Version 2, available at
  <http://www.fsf.org/copyleft/gpl.html>, or the OpenIB.org BSD
  license, available in the LICENSE.TXT file accompanying this
  software.  These details are also available at
  <http://openib.org/license.html>.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
  ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  Copyright (c) 2004 Mellanox Technologies Ltd.  All rights reserved.
*/

#include "mt25408.h"
#include "ib_driver.h"
//#include "pci.h"

#define MOD_INC(counter, max_count) (counter) = ((counter)+1) & ((max_count) - 1)

#define MAX(a, b) (a > b ? a : b)


#define breakpoint {volatile __u32 *p=(__u32 *)0x1234;printf("breakpoint\n");do {} while((*p) != 0x1234);}

#define WRITE_BYTE_VOL(addr, off, val) \
    do { \
        (*((volatile __u8 *)(((volatile __u8 *)(addr)) + off))) = (val); \
    } while(0)

#define WRITE_WORD_VOL(addr, off, val) \
    do { \
        (*((volatile __u16 *)(((volatile __u8 *)(addr)) + off))) = (val); \
    } while(0)

struct device_buffers_st {
	/* inprm and outprm do not have alignnemet constraint since that
	   is acheived programatically */
	u8 inprm_buf[INPRM_BUF_SZ];
	u8 outprm_buf[OUTPRM_BUF_SZ];

	/* mads QP - send is larger so is first */
	union ud_send_wqe_u mads_qp_snd_queue[NUM_MADS_SND_WQES]
	    __attribute__ ((aligned(UD_SEND_WQE_U_ALIGN)));
        union recv_wqe_u mads_qp_rcv_queue[NUM_MADS_RCV_WQES]
	    __attribute__ ((aligned(RECV_WQE_U_ALIGN)));

	union ud_send_wqe_u ipoib_qp_snd_queue[NUM_IPOIB_SND_WQES]
	    __attribute__ ((aligned(UD_SEND_WQE_U_ALIGN)));
	union recv_wqe_u ipoib_qp_rcv_queue[NUM_IPOIB_RCV_WQES]
	    __attribute__ ((aligned(RECV_WQE_U_ALIGN)));

	struct eqe_t eq_buf[1 << LOG2_EQ_SZ]
	    __attribute__ ((aligned(sizeof(struct eqe_t))));
	union cqe_st mads_snd_cq_buf[NUM_MADS_SND_CQES]
	    __attribute__ ((aligned(sizeof(union cqe_st))));
	union cqe_st ipoib_snd_cq_buf[NUM_IPOIB_SND_CQES]
	    __attribute__ ((aligned(sizeof(union cqe_st))));
	union cqe_st mads_rcv_cq_buf[NUM_MADS_RCV_CQES]
	    __attribute__ ((aligned(sizeof(union cqe_st))));
	union cqe_st ipoib_rcv_cq_buf[NUM_IPOIB_RCV_CQES]
	    __attribute__ ((aligned(sizeof(union cqe_st))));
	union ud_av_u av_array[NUM_AVS];
} __attribute__ ((packed));

#define STRUCT_ALIGN_SZ 4096
#define SRC_BUF_SZ (sizeof(struct device_buffers_st) + STRUCT_ALIGN_SZ - 1)

/* the following must be kept in this order
   for the memory region to cover the buffers */
static u8 src_buf[SRC_BUF_SZ];
static struct ib_buffers_st ib_buffers;
static __u32 memreg_size;
static __u32 cmpt_resource_size;

/* end of order constraint */

struct phys_mem_desc_st {
	unsigned long base;
	unsigned long offset;
};

static struct phys_mem_desc_st phys_mem;

static struct dev_pci_struct hermon_pci_dev;
static struct device_buffers_st *dev_buffers_p;
static struct device_ib_data_st dev_ib_data;


static int gw_write_cr(__u32 addr, __u32 data)
{
	writel(htonl(data), hermon_pci_dev.cr_space + addr);
	return 0;
}

static int gw_read_cr(__u32 addr, __u32 * result)
{
	*result = ntohl(readl(hermon_pci_dev.cr_space + addr));
	return 0;
}

static int reset_hca(void)
{
        /* TODO
          make sure the device recovers from reset (read vendor id) */
	return gw_write_cr(HERMON_RESET_OFFSET, 1);
}

static int ib_device_init(struct pci_device *dev)
{
	int i;
	int rc;

	memset(&dev_ib_data, 0, sizeof dev_ib_data);

	/* save bars */
	for (i = 0; i < 6; ++i) {
		hermon_pci_dev.dev.bar[i] =
		    pci_bar_start(dev, PCI_BASE_ADDRESS_0 + (i << 2));
		if ((i % 2) && hermon_pci_dev.dev.bar[i]) {
			eprintf("");
			return -1;
		}
	}

	/* save config space */
	for (i = 0; i < 64; ++i) {
		rc = pci_read_config_dword(dev, i << 2,
					   &hermon_pci_dev.dev.
					   dev_config_space[i]);
		if (rc) {
			eprintf("");
			return rc;
		}
	}

	hermon_pci_dev.dev.dev = dev;

	/* map cr-space */
	hermon_pci_dev.cr_space =
	    ioremap(hermon_pci_dev.dev.bar[0], 0x100000);
	if (!hermon_pci_dev.cr_space) {
		eprintf("");
		return -1;
	}

	/* map uar */
	hermon_pci_dev.uar =
	    ioremap(hermon_pci_dev.dev.bar[2] + UAR_IDX * 0x1000, 0x1000);
	if (!hermon_pci_dev.uar) {
		eprintf("");
		return -1;
	}

	return 0;
}

/*
 * my_log2()
 * - Getting the log of a number 
 */
static int my_log2(unsigned long arg)
{
	int i;
	__u32 tmp;

	if (arg == 0) {
		return INT_MIN;	/* log2(0) = -infinity */
	}

	tmp = 1;
	i = 0;
	while (tmp < arg) {
		tmp = tmp << 1;
		++i;
	}

	return i;
}

static inline unsigned long lalign(unsigned long buf, unsigned long align)
{
	return (unsigned long)((buf + align - 1) &
			       (~(((unsigned long)align) - 1)));
}

#include <gpxe/umalloc.h>
static int init_dev_data(void)
{
	unsigned long tmp;
	unsigned long reserve_size = 32 * 1024 * 1024;

	tmp = lalign(virt_to_bus(src_buf), STRUCT_ALIGN_SZ);

	dev_buffers_p = bus_to_virt(tmp);
	memreg_size = (__u32) (&memreg_size) - (__u32) dev_buffers_p;

	//	phys_mem.base =
	//	    (virt_to_phys(_text) - reserve_size) & (~(reserve_size - 1));
	phys_mem.base = ( ( user_to_phys ( umalloc ( reserve_size * 2 ), 0 ) +
			    ( reserve_size - 1 ) ) & ~( reserve_size - 1 ) );

	phys_mem.offset = 0;

        dev_ib_data.num_mappings = 0;

	return 0;
}

static int map_icm_wrapper(unsigned long long icm_virtual_start, unsigned long entry_size)
{
	int rc;

	struct map_icm_st tmp_map_obj;
	memset(&tmp_map_obj, 0, sizeof tmp_map_obj);
	tmp_map_obj.num_vpm = 1;
	tmp_map_obj.vpm_arr[0].va_l = icm_virtual_start & F_MASK;
	tmp_map_obj.vpm_arr[0].va_h = icm_virtual_start >> 32;
	tmp_map_obj.vpm_arr[0].pa_l = phys_mem.base + phys_mem.offset;
	tmp_map_obj.vpm_arr[0].log2_size = my_log2((entry_size + 4095) >> 12);
        rc = cmd_map_icm(&tmp_map_obj);
	return rc;
}


static int restore_config(void)
{
	int i;
	int rc;

	for (i = 0; i < 64; ++i) {
		if (i != 22 && i != 23) {
			rc = pci_write_config_dword(hermon_pci_dev.dev.dev,
						    i << 2,
						    hermon_pci_dev.dev.
						    dev_config_space[i]);
			if (rc) {
				return rc;
			}
		}
	}
	return 0;
}


static void prep_init_hca_buf(struct init_hca_st *init_hca_p, void *buf)
{
	unsigned long ptr;
	__u8 shift;

	memset(buf, 0, MT_STRUCT_SIZE(hermonprm_init_hca_st));

	ptr = (unsigned long)buf;
	INS_FLD(2, ptr, hermonprm_init_hca_st, version);
	INS_FLD(1, ptr, hermonprm_init_hca_st, udp);

	ptr = (unsigned long)buf +
	    MT_BYTE_OFFSET(hermonprm_init_hca_st,
			   qpc_eec_cqc_eqc_rdb_parameters); 

	shift = 32 - MT_BIT_SIZE(hermonprm_qpcbaseaddr_st, qpc_base_addr_l);
	INS_FLD(init_hca_p->qpc_base_addr_h, ptr, hermonprm_qpcbaseaddr_st,
		qpc_base_addr_h);
	INS_FLD(init_hca_p->qpc_base_addr_l >> shift, ptr,
		hermonprm_qpcbaseaddr_st, qpc_base_addr_l);
	INS_FLD(init_hca_p->log_num_of_qp, ptr, hermonprm_qpcbaseaddr_st,
		log_num_of_qp);

	shift = 32 - MT_BIT_SIZE(hermonprm_qpcbaseaddr_st, srqc_base_addr_l);
	INS_FLD(init_hca_p->srqc_base_addr_h, ptr, hermonprm_qpcbaseaddr_st,
		srqc_base_addr_h);
	INS_FLD(init_hca_p->srqc_base_addr_l >> shift, ptr,
		hermonprm_qpcbaseaddr_st, srqc_base_addr_l);
	INS_FLD(init_hca_p->log_num_of_srq, ptr, hermonprm_qpcbaseaddr_st,
		log_num_of_srq);

	shift = 32 - MT_BIT_SIZE(hermonprm_qpcbaseaddr_st, cqc_base_addr_l);
	INS_FLD(init_hca_p->cqc_base_addr_h, ptr, hermonprm_qpcbaseaddr_st,
		cqc_base_addr_h);
	INS_FLD(init_hca_p->cqc_base_addr_l >> shift, ptr,
		hermonprm_qpcbaseaddr_st, cqc_base_addr_l);
	INS_FLD(init_hca_p->log_num_of_cq, ptr, hermonprm_qpcbaseaddr_st,
		log_num_of_cq);

	INS_FLD(init_hca_p->altc_base_addr_h, ptr, hermonprm_qpcbaseaddr_st, 
		altc_base_addr_h);
	INS_FLD(init_hca_p->altc_base_addr_l, ptr, hermonprm_qpcbaseaddr_st, 
		altc_base_addr_l);

	INS_FLD(init_hca_p->auxc_base_addr_h, ptr, hermonprm_qpcbaseaddr_st, 
		auxc_base_addr_h);
	INS_FLD(init_hca_p->auxc_base_addr_l, ptr, hermonprm_qpcbaseaddr_st, 
		auxc_base_addr_l);

    
	shift = 32 - MT_BIT_SIZE(hermonprm_qpcbaseaddr_st, eqc_base_addr_l);
	INS_FLD(init_hca_p->eqc_base_addr_h, ptr, hermonprm_qpcbaseaddr_st,
		eqc_base_addr_h);
	INS_FLD(init_hca_p->eqc_base_addr_l >> shift, ptr,
		hermonprm_qpcbaseaddr_st, eqc_base_addr_l);
	INS_FLD(init_hca_p->log_num_of_eq, ptr, hermonprm_qpcbaseaddr_st,
		log_num_of_eq);

	
	shift = 32 - MT_BIT_SIZE(hermonprm_qpcbaseaddr_st, rdmardc_base_addr_l);
	INS_FLD(init_hca_p->rdmardc_base_addr_h, ptr, hermonprm_qpcbaseaddr_st,
		rdmardc_base_addr_h);
	INS_FLD(init_hca_p->rdmardc_base_addr_l >> shift, ptr, 
		hermonprm_qpcbaseaddr_st, rdmardc_base_addr_l);
	
	ptr = (unsigned long)buf +
		MT_BYTE_OFFSET(hermonprm_init_hca_st, multicast_parameters);

	INS_FLD(init_hca_p->mc_base_addr_h, ptr, hermonprm_multicastparam_st,
		mc_base_addr_h);
	INS_FLD(init_hca_p->mc_base_addr_l, ptr, hermonprm_multicastparam_st,
		mc_base_addr_l);
	INS_FLD(init_hca_p->log_mc_table_entry_sz, ptr,
		hermonprm_multicastparam_st, log_mc_table_entry_sz);
	INS_FLD(init_hca_p->log_mc_table_hash_sz, ptr, hermonprm_multicastparam_st,
		log_mc_table_hash_sz);
	INS_FLD(init_hca_p->log_mc_table_sz, ptr, hermonprm_multicastparam_st,
		log_mc_table_sz);

	ptr = (unsigned long)buf +
	    MT_BYTE_OFFSET(hermonprm_init_hca_st, tpt_parameters);

	INS_FLD(init_hca_p->dmpt_base_addr_h, ptr, hermonprm_tptparams_st,
		dmpt_base_adr_h);
	INS_FLD(init_hca_p->dmpt_base_addr_l, ptr, hermonprm_tptparams_st,
		dmpt_base_adr_l);
	INS_FLD(init_hca_p->log_dmpt_sz, ptr, hermonprm_tptparams_st, log_dmpt_sz);

	INS_FLD(init_hca_p->mtt_base_addr_h, ptr, hermonprm_tptparams_st,
		mtt_base_addr_h);
	INS_FLD(init_hca_p->mtt_base_addr_l, ptr, hermonprm_tptparams_st,
		mtt_base_addr_l);

	INS_FLD(init_hca_p->cmpt_base_addr_h, ptr, hermonprm_tptparams_st,
		cmpt_base_adr_h);
	INS_FLD(init_hca_p->cmpt_base_addr_l, ptr, hermonprm_tptparams_st,
		cmpt_base_adr_l);
    
	ptr = (unsigned long)buf +
	    MT_BYTE_OFFSET(hermonprm_init_hca_st, uar_parameters);

	INS_FLD(init_hca_p->log_max_uars, ptr, hermonprm_uar_params_st,
		log_max_uars);
        
}

static __u32 get_page_offset(void *address)
{
	return virt_to_bus(address) & 0xfff;
}


static int get_num_pages(__u32 size, void *address)
{
	unsigned long ba = virt_to_bus(address);

	return (ba + size + 4095) / 4096 - ba / 4096;
}


static void prep_write_mtt_buf(void *inprm, __u32 mtt_offset, void *va, unsigned num_pages)
{
	__u32 *ptr;
	int size;
	__u32 base_ba;
	unsigned int i;

	base_ba = virt_to_bus(va) & (~4095);

	size = 16 + 8 * num_pages;
        memset(inprm, 0, size);
	ptr = (__u32 *)inprm;
	ptr[1] = mtt_offset;
	for (i = 0; i < num_pages; ++i) {
		ptr[2 * i + 5] = base_ba | 1;
		base_ba += PAGE_SIZE;
	}
}

static void prep_sw2hw_mpt_buf(void *buf, __u32 mkey)
{
	__u32 *ptr = buf;

	memset(ptr, 0, 80);

	ptr[0] = 1 << 8  |	/* MR */
		 1 << 9  |      /* no translation */
		 1 << 10 |      /* local read */
		 1 << 11;	/* local write */

	ptr[2] = mkey;
	ptr[3] = GLOBAL_PD;
	ptr[5] = virt_to_bus(dev_buffers_p);
	ptr[7] = memreg_size;
}

static void prep_sw2hw_eq_buf(void *buf, __u32 page_offset, __u32 mtt_offset)
{
	memset(buf, 0, MT_STRUCT_SIZE(hermonprm_eqc_st));

	if (page_offset & 0x1f)
		eprintf("page_offset = 0x%x\n", page_offset);

	INS_FLD(0xa, buf, hermonprm_eqc_st, st);	/* fired */
	INS_FLD(LOG2_EQ_SZ, buf, hermonprm_eqc_st, log_eq_size);
	INS_FLD(mtt_offset >> 3, buf, hermonprm_eqc_st, mtt_base_addr_l);
	INS_FLD(page_offset >> 5, buf, hermonprm_eqc_st, page_offset);
}

static void prep_set_port(void *buf)
{
    
	memset(buf, 0, MT_STRUCT_SIZE(hermonprm_init_port_st));

	INS_FLD(MTU_2048, buf, hermonprm_init_port_st, mtu);
	INS_FLD(3, buf, hermonprm_init_port_st, port_width_cap);
	INS_FLD(1, buf, hermonprm_init_port_st, vl_cap);
	INS_FLD(1, buf, hermonprm_init_port_st, max_gid);
	INS_FLD(64, buf, hermonprm_init_port_st, max_pkey);
}

static void prep_sw2hw_cq_buf(void *buf, __u8 eqn, __u32 mtt_offset,
			      __u32 page_offset, __u32 *cq_db_record)
{
	memset(buf, 0, MT_STRUCT_SIZE(hermonprm_completion_queue_context_st));

	INS_FLD(0xA, buf, hermonprm_completion_queue_context_st, st);
	INS_FLD(page_offset >> 5, buf, hermonprm_completion_queue_context_st, page_offset);
	INS_FLD(LOG2_CQ_SZ, buf, hermonprm_completion_queue_context_st, log_cq_size);
	INS_FLD(dev_ib_data.uar_idx, buf, hermonprm_completion_queue_context_st, usr_page);
	INS_FLD(eqn, buf, hermonprm_completion_queue_context_st, c_eqn);
        INS_FLD(mtt_offset >> 3, buf, hermonprm_completion_queue_context_st, mtt_base_addr_l);
	INS_FLD(virt_to_bus(cq_db_record) >> 3, buf, 
		hermonprm_completion_queue_context_st, db_record_addr_l);
}

static void prep_rst2init_qpee_buf(void *buf,
				   __u32 snd_cqn,
				   __u32 rcv_cqn,
				   __u32 qkey,
				   __u32 log_rq_size,
				   __u32 log_rq_stride,
				   __u32 log_sq_size,
				   __u32 log_sq_stride,
				   __u64 db_record_addr,
				   __u32 mtt_base_offset,
				   __u32 page_offset)
{
	__u32 *tmp;
	struct qp_state_tarnisition_st *prm = buf;

	memset(prm, 0, sizeof *prm);

	tmp = (__u32 *)&prm->ctx;
	tmp[0] |= 3 << 16; /* UD */
	tmp[0] |= 3 << 11; /* migrated */
	tmp[3] |= dev_ib_data.uar_idx;
	tmp[1] |= dev_ib_data.pd;
	tmp[31] |= snd_cqn;
	tmp[39] |= rcv_cqn;
	tmp[42] |= qkey;
	tmp[2] |= (log_sq_size & 0xf) << 11 |
		  (log_rq_size & 0xf) << 19 |
		  ((log_sq_stride - 4) & 0x7) << 8 |
		  ((log_rq_stride - 4)  & 0x7) << 16;

	tmp[51] |= mtt_base_offset & 0xfffffff8;
	tmp[40] |= (__u32)(db_record_addr >> 32);
	tmp[41] |= (__u32)db_record_addr & 0xfffffffc;

	if (page_offset & 0x3f)
		eprintf("buffer not properly aligned");

	tmp[36] = page_offset;
}

static void prep_init2rtr_qpee_buf(void *buf, __u8 sl)
{
	struct qp_state_tarnisition_st *prm;
	__u32 *tmp;

	prm = (struct qp_state_tarnisition_st *)buf;

	memset(prm, 0, sizeof *prm);

	tmp = (__u32 *)&prm->ctx;
	tmp[2] |= MTU_2048 << 29 | 11 << 24;

	tmp[14] = (0x83 | (dev_ib_data.port - 1) << 6 | sl << 2) << 24;
}

static void init_av_array(void)
{
	int i;

	dev_ib_data.udav.av_array = dev_buffers_p->av_array;
	dev_ib_data.udav.udav_next_free = FL_EOL;
	for (i = 0; i < NUM_AVS; ++i) {
		dev_ib_data.udav.av_array[i].ud_av.next_free =
		    dev_ib_data.udav.udav_next_free;
		dev_ib_data.udav.udav_next_free = i;
	}
}


/*
 * get_req_icm_pages
 */
static unsigned long get_req_icm_pages(unsigned long log2_reserved,
				       unsigned long app_rsrc,
				       unsigned long entry_size,
				       unsigned long *log2_entries_p)
{
	unsigned long size;
	unsigned long log2_entries;

	log2_entries = my_log2((1 << log2_reserved) + app_rsrc);
	*log2_entries_p = log2_entries;
	size = (1 << log2_entries) * entry_size;

	return (size + 4095) >> 12;
}



void mtt_alloc_init(int num_reserved, int entry_sz)
{
	
	dev_ib_data.mtt_alloc.mtt_entry_sz = entry_sz;
	dev_ib_data.mtt_alloc.alloc_ptr = num_reserved * entry_sz;
}

unsigned mtt_alloc(int num)
{
	unsigned new_alloc = dev_ib_data.mtt_alloc.alloc_ptr;

	dev_ib_data.mtt_alloc.alloc_ptr += num * dev_ib_data.mtt_alloc.mtt_entry_sz;

	return new_alloc;
}


void add_icm_obj(__u64 icm_va, __u32 size)
{
	int i;

	i = dev_ib_data.num_mappings;
	dev_ib_data.icm_arr[i].npages = size >> 12;
	dev_ib_data.icm_arr[i].va = icm_va;

	++dev_ib_data.num_mappings;
}

void unamp_icm_array()
{
	int i;

	while (dev_ib_data.num_mappings) {
		i = --dev_ib_data.num_mappings;
		cmd_unmap_icm(dev_ib_data.icm_arr[i].va, dev_ib_data.icm_arr[i].npages);
	}
}

/* 
    One of the issues in setup_hca is mapping the ICM.
    ICM mapping explanation:
    -----------------------
    1. Mapping the cMPT table:
        for each of the 4 resources (qp,srq,cq,eq): 
            a. get the physical needed size: e.g. num_of_req_ qp's * cMPT_entry_sz
            b. allocate 1GB (CMPT_ENTRIES_PER_RESOURCE * dev_cap.c_mpt_entry_sz)
               of virtual memory.
        Total mapping of cMPT takes 4GB.
    2. Mapping virtually the rest of needed resources.
    3. Getting the aux needed pages of all virtual memory.
    4. Mapping all needed physical resources (5 allocations):
        4 of them for each resource in '1a' and one for rest the in '2'.
*/


static int setup_hca(__u8 port, void **eq_p)
{
	int ret;
	int rc;
	struct query_fw_st qfw;
	struct map_icm_st map_obj;
	struct dev_cap_st dev_cap;
	struct init_hca_st init_hca;
        __u8 log2_pages;
	__u32 tmp;
	__u64 icm_map_start, icm_start;
	__u32 qp_cmpt_size, srq_cmpt_size, cq_cmpt_size, eq_cmpt_size;
	__u32 mtt_offset;
        __u32 log2_entries;
	__u32 aux_pages;
	__u32 mem_key, key;
	__u32 mkey_idx;
	__u8 eqn;
	__u32 uar_eqn;
	__u32 num_of_pages;
	void *uar_eq;
	__u32 event_mask;
	struct eqe_t *eq_buf;
	void *inprm;
	__u32 bus_addr;
	struct query_adapter_st qa;
	__u8 log_max_uars = 8;

	init_dev_data();
	inprm = get_inprm_buf();


#if 0
	rc = reset_hca();
	if (rc) {
		eprintf("");
		return rc;
	} 
	  


	mdelay(1000);		/* wait for 1 sec */

	rc = restore_config();
	if (rc) {
		eprintf("");
		return rc;
	}
#endif

	dev_ib_data.pd = GLOBAL_PD;
	dev_ib_data.port = port;
	dev_ib_data.qkey = GLOBAL_QKEY;

	rc = cmd_query_fw(&qfw);
	if (rc) {
		eprintf("");
		return rc;
	}
        if (print_info) {
		printf("FW ver = %d.%d.%d\n",
		       qfw.fw_rev_major,
		       qfw.fw_rev_minor,
		       qfw.fw_rev_subminor);
	}


	if (qfw.error_buf_offset_h) {
		eprintf("error buf offset too high");
		return -1;
	}

	if (qfw.error_buf_bar != 0) {
		eprintf("invalid error buf bar");
		return -1;
	}

	dev_ib_data.error_buf_addr= qfw.error_buf_offset_l;
	dev_ib_data.error_buf_size= qfw.error_buf_size;
	if (!dev_ib_data.error_buf_addr) {
		eprintf("");
		return -1;
	}


	bus_addr =
	    ((unsigned long)((u64) qfw.clear_int_base_offset_h << 32) | qfw.
	     clear_int_base_offset_l);
	dev_ib_data.clr_int_addr = bus_to_virt(bus_addr);

	log2_pages = my_log2(qfw.fw_pages);

	memset(&map_obj, 0, sizeof map_obj);
	map_obj.num_vpm = 1;
	map_obj.vpm_arr[0].log2_size = log2_pages;
	map_obj.vpm_arr[0].pa_l = phys_mem.base + phys_mem.offset;
	rc = cmd_map_fa(&map_obj);
	if (rc) {
                eprintf("");
		return -1;
	}
	phys_mem.offset += 1 << (log2_pages + 12);

        rc = cmd_run_fw();
	if (rc) {
		eprintf("");
		goto undo_map_fa;
	}

	rc = cmd_mod_stat_cfg();
	if (rc) {
		eprintf("");
		goto undo_map_fa;
	}


	/* actually query_dev_cap */
	rc = cmd_query_dev_cap(&dev_cap);
	if (rc) {
		eprintf("");
		goto undo_map_fa;
	}

	/* initilaize mtt allocator */
	mtt_alloc_init(1 << dev_cap.log2_rsvd_mtts, dev_cap.mtt_entry_sz);

	/* Calculating EQ number */
	uar_eqn = MAX(dev_cap.num_rsvd_uars,(dev_cap.num_rsvd_eqs+3)/4);
	eqn = uar_eqn * 4;

	dev_ib_data.uar_idx = UAR_IDX;

	memset(&init_hca, 0, sizeof init_hca);
	icm_start = 0;

	/* In this section I made for qp,srq,cq,eq the following :
           <-1-> allocate physical memory for qp, srq, cq, eq.
                 icm_<resource>_size += num_resource * cMPT_entry_size
	*/
	cmpt_resource_size = (CMPT_ENTRIES_PER_RESOURCE * dev_cap.c_mpt_entry_sz);
	/* QP MAPPING */
	tmp = get_req_icm_pages(dev_cap.log2_rsvd_qps,
                            MAX_APP_QPS,
                            dev_cap.c_mpt_entry_sz, &log2_entries);
	qp_cmpt_size = (tmp << 12);

	/* SRQ MAPPING */
	tmp = get_req_icm_pages(dev_cap.log2_rsvd_srqs, 0, 
				dev_cap.c_mpt_entry_sz, &log2_entries);
	srq_cmpt_size = (tmp << 12);


	/* CQ MAPPING */
	tmp = get_req_icm_pages(dev_cap.log2_rsvd_cqs, MAX_APP_CQS,
                            dev_cap.c_mpt_entry_sz, &log2_entries);
	cq_cmpt_size = (tmp << 12);

	/* EQ mapping */
	tmp = get_req_icm_pages(0,dev_cap.num_rsvd_eqs + eqn,
				dev_cap.c_mpt_entry_sz, &log2_entries);
	icm_start += ((__u64)cmpt_resource_size * 4); 
	eq_cmpt_size = (tmp << 12);
	/* End of cMPT table allocation */
	
	DBG ( "Hermon ICM QPC base = %llx\n", icm_start );
	tmp = get_req_icm_pages(dev_cap.log2_rsvd_qps,
				MAX_APP_QPS,
				dev_cap.qpc_entry_sz, &log2_entries);
	init_hca.qpc_base_addr_l = icm_start & F_MASK;
	init_hca.qpc_base_addr_h = icm_start >> 32;
	init_hca.log_num_of_qp = log2_entries;
	icm_start += (tmp << 12);

	DBG ( "Hermon ICM ALTC base = %llx\n", icm_start );
	tmp = get_req_icm_pages(dev_cap.log2_rsvd_qps,
                            MAX_APP_QPS,
                            dev_cap.altc_entry_size, &log2_entries);
	init_hca.altc_base_addr_l = icm_start & F_MASK;
	init_hca.altc_base_addr_h = icm_start >> 32;
	icm_start += (tmp << 12);

	DBG ( "Hermon ICM AUXC base = %llx\n", icm_start );
	tmp = get_req_icm_pages(dev_cap.log2_rsvd_qps,
                            MAX_APP_QPS,
                            dev_cap.auxc_entry_size, &log2_entries);
	init_hca.auxc_base_addr_l = icm_start & F_MASK;
	init_hca.auxc_base_addr_h = icm_start >> 32;
	icm_start += (tmp << 12);

	DBG ( "Hermon ICM SRQC base = %llx\n", icm_start );
	tmp = get_req_icm_pages(dev_cap.log2_rsvd_srqs,
                            0, dev_cap.srq_entry_sz, &log2_entries);
	init_hca.srqc_base_addr_l = icm_start & F_MASK;
	init_hca.srqc_base_addr_h = icm_start >> 32;
	init_hca.log_num_of_srq = log2_entries;
	icm_start += (tmp << 12);

	DBG ( "Hermon ICM CQC base = %llx\n", icm_start );
	tmp = get_req_icm_pages(dev_cap.log2_rsvd_cqs,
				MAX_APP_CQS,
				dev_cap.cqc_entry_sz, &log2_entries);
	init_hca.cqc_base_addr_l = icm_start & F_MASK;
	init_hca.cqc_base_addr_h = icm_start >> 32;
        init_hca.log_num_of_cq = log2_entries;
	icm_start += (tmp << 12);

	DBG ( "Hermon ICM EQC base = %llx\n", icm_start );
	tmp = get_req_icm_pages(0, dev_cap.num_rsvd_eqs + eqn,
				dev_cap.eqc_entry_sz, &log2_entries);
	init_hca.eqc_base_addr_l = icm_start & F_MASK;
	init_hca.eqc_base_addr_h = icm_start >> 32;
	init_hca.log_num_of_eq = log2_entries;
	icm_start += (tmp << 12);

	DBG ( "Hermon ICM MTT base = %llx\n", icm_start );
        tmp = get_req_icm_pages(dev_cap.log2_rsvd_mtts,
				NUM_RESOURCES_MTTS + 
				get_num_pages(sizeof(ib_buffers),&ib_buffers),
				dev_cap.mtt_entry_sz, &log2_entries);
	init_hca.mtt_base_addr_l = icm_start & F_MASK;
	init_hca.mtt_base_addr_h = icm_start >> 32;
	icm_start += (tmp << 12);

	DBG ( "Hermon ICM DMPT base = %llx\n", icm_start );
	tmp = get_req_icm_pages(dev_cap.log2_rsvd_mrws,
                            1, dev_cap.d_mpt_entry_sz, &log2_entries);
	init_hca.dmpt_base_addr_l = icm_start & F_MASK;
	init_hca.dmpt_base_addr_h = icm_start >> 32;
	init_hca.log_dmpt_sz = log2_entries;
	icm_start += (tmp << 12);

	DBG ( "Hermon ICM MC base = %llx\n", icm_start );
	init_hca.mc_base_addr_l = icm_start & F_MASK;
	init_hca.mc_base_addr_h = icm_start >> 32;
	init_hca.log_mc_table_entry_sz =
	    my_log2(MT_STRUCT_SIZE(hermonprm_mgm_entry_st));
	init_hca.log_mc_table_hash_sz = 3; 
	init_hca.log_mc_table_sz = 3;

	icm_start += 4096 * (1 +
               (((dev_cap.num_rsvd_eqs + eqn) * dev_cap.c_mpt_entry_sz) / 4096));


	/* Mapping the pages 
        1. Mapping ICM_AUX_pages.
        2. Mapping qp,srq,cq,eq cMPT talbe 
        3. Mapping the rest of reserved, etc ... 
	*/
	rc = cmd_set_icm_size(icm_start, &aux_pages);
	if (rc) {
		eprintf("");
		goto undo_map_fa;
	}

	memset(&map_obj, 0, sizeof map_obj);
	map_obj.num_vpm = 1;
	map_obj.vpm_arr[0].pa_l = phys_mem.base + phys_mem.offset;
	map_obj.vpm_arr[0].log2_size = my_log2(aux_pages);
	rc = cmd_map_icm_aux(&map_obj);
	if (rc) {
		eprintf("");
		goto undo_map_fa;
	}
	phys_mem.offset += (1 << (map_obj.vpm_arr[0].log2_size + 12));


	/* using icm_start to calc va_l and log2_pages for physical addres */
	icm_map_start = 0;
        
	/* mapping cMPT table for qps */
	rc = map_icm_wrapper(icm_map_start,qp_cmpt_size);
	if (rc) {
                eprintf("");
		goto undo_map_fa;
	}
	add_icm_obj(icm_map_start, qp_cmpt_size);
	phys_mem.offset += (1 << (my_log2((qp_cmpt_size + 4095) >> 12) + 12));
	icm_map_start += cmpt_resource_size;
	
	/* mapping cMPT table for srq */
	rc = map_icm_wrapper(icm_map_start,srq_cmpt_size);
	if (rc) {
		eprintf("");
		goto undo_map_fa;
	}
	add_icm_obj(icm_map_start, srq_cmpt_size);
	phys_mem.offset += (1 << (my_log2((srq_cmpt_size + 4095) >> 12) + 12));
        icm_map_start += cmpt_resource_size;

	/* mapping cMPT table for cq */
	rc = map_icm_wrapper(icm_map_start,cq_cmpt_size);
	if (rc) {
		eprintf("");
		goto undo_map_fa;
	}
	add_icm_obj(icm_map_start, cq_cmpt_size);
	phys_mem.offset += (1 << (my_log2((cq_cmpt_size + 4095) >> 12) + 12));
	icm_map_start += cmpt_resource_size;

	/* mapping cMPT table for eq */
        rc = map_icm_wrapper(icm_map_start,eq_cmpt_size);
	if (rc) {
                eprintf("");
		goto undo_map_fa;
	}
	add_icm_obj(icm_map_start, eq_cmpt_size);
        phys_mem.offset += (1 << (my_log2((eq_cmpt_size + 4095) >> 12) + 12));
	icm_map_start += cmpt_resource_size;

	/* mapping the rest of the ICM table */
	rc = map_icm_wrapper(icm_map_start,icm_start - icm_map_start);
	if (rc) {
		eprintf("");
		goto undo_map_fa;
	}
	add_icm_obj(icm_map_start, icm_start - icm_map_start);
        phys_mem.offset += (1 << (my_log2(((icm_start - icm_map_start) + 4095)
			    >> 12) + 12));
        init_hca.log_max_uars = log_max_uars;
	/* End of icm mapping */



	prep_init_hca_buf(&init_hca, inprm);
	rc = cmd_init_hca(inprm, MT_STRUCT_SIZE(hermonprm_init_hca_st));
	if (rc) {
		eprintf("");
		goto undo_map_fa;
	}

	rc = cmd_query_adapter(&qa);
	if (rc) {
		eprintf("");
		return rc;
	}
	dev_ib_data.clr_int_data = 1 << qa.intapin;

	mkey_idx = 1 << dev_cap.log2_rsvd_mrws;
	mem_key = (mkey_idx << 8) | (MKEY_PREFIX >> 24);

	prep_sw2hw_mpt_buf(inprm, mkey_idx | MKEY_PREFIX);
	/* key will be 1 << log2_rsvd_mrws in case of success. */
	rc = cmd_sw2hw_mpt(&key, mkey_idx, inprm, SW2HW_MPT_IBUF_SZ);
	if (rc) {
		eprintf("");
		goto undo_map_fa;
	} else 
	  
        dev_ib_data.mkey = mem_key;

	/* TODO: check it */
	uar_eq = ioremap((unsigned long)hermon_pci_dev.uar + (uar_eqn << 12), 4096);

	/* allocate a single EQ which will receive 
	   all the events */
        eq_buf = dev_buffers_p->eq_buf;
	num_of_pages = get_num_pages(sizeof(*eq_buf) << LOG2_EQ_SZ, eq_buf);
	mtt_offset = mtt_alloc(num_of_pages);
        prep_write_mtt_buf(inprm, mtt_offset, eq_buf, num_of_pages);
	rc = cmd_write_mtt(inprm, num_of_pages);
	if (rc) {
		eprintf("");
		goto undo_sw2hw_mpt;
	}

	prep_sw2hw_eq_buf(inprm, get_page_offset(eq_buf), mtt_offset);
	rc = cmd_sw2hw_eq(eqn, SW2HW_EQ_IBUF_SZ);
	if (rc) {
		eprintf("");
		goto undo_sw2hw_mpt;
	} 
	  

	event_mask = (1 << XDEV_EV_TYPE_CQ_COMP) |
	    (1 << XDEV_EV_TYPE_CQ_ERR) |
	    (1 << XDEV_EV_TYPE_LOCAL_WQ_CATAS_ERR) |
	    (1 << XDEV_EV_TYPE_PORT_ERR) |
	    (1 << XDEV_EV_TYPE_LOCAL_WQ_INVALID_REQ_ERR) |
	    (1 << XDEV_EV_TYPE_LOCAL_WQ_ACCESS_VIOL_ERR); 

	rc = cmd_map_eq(eqn, event_mask, 1);
	if (rc) {
		eprintf("");
		goto undo_sw2hw_eq;
	} 
	  


	*eq_p = &dev_ib_data.eq;
	dev_ib_data.eq.eqn = eqn;
	dev_ib_data.eq.eq_buf = eq_buf;
	dev_ib_data.eq.cons_counter = 0;
	dev_ib_data.eq.eq_size = 1 << LOG2_EQ_SZ;
	dev_ib_data.eq.eq_doorbell = uar_eq;

	prep_set_port(inprm);
	rc = cmd_init_ib(port,inprm,SET_PORT_IBUF_SZ);
	if (rc) {
		eprintf("");
		goto undo_sw2hw_eq;
	} 
	  
	init_av_array();

	/* set the qp and cq numbers according
	   to the results of query_dev_cap */
	dev_ib_data.mads_qp.qpn = (1 << dev_cap.log2_rsvd_qps) +
	    + QPN_BASE + MADS_QPN_SN;
	dev_ib_data.ipoib_qp.qpn = (1 << dev_cap.log2_rsvd_qps) +
	    + QPN_BASE + IPOIB_QPN_SN;

	dev_ib_data.mads_qp.snd_cq.cqn = (1 << dev_cap.log2_rsvd_cqs) +
	    MADS_SND_CQN_SN;
	dev_ib_data.mads_qp.rcv_cq.cqn = (1 << dev_cap.log2_rsvd_cqs) +
	    MADS_RCV_CQN_SN;

	dev_ib_data.ipoib_qp.snd_cq.cqn = (1 << dev_cap.log2_rsvd_cqs) +
	    IPOIB_SND_CQN_SN;
	dev_ib_data.ipoib_qp.rcv_cq.cqn = (1 << dev_cap.log2_rsvd_cqs) +
	    IPOIB_RCV_CQN_SN;

	return 0;

undo_sw2hw_eq:
	ret = -1;
	rc = cmd_hw2sw_eq(eqn);
	if (rc)
		eprintf("");

undo_sw2hw_mpt:
	ret = -1;
	rc = cmd_hw2sw_mpt(mkey_idx);
	if (rc)
		eprintf("");

undo_map_fa:
        ret = -1;
	unamp_icm_array();
	rc = cmd_unmap_fa();
	if (rc)
		eprintf("");

	return ret;
}


static int unset_hca(void)
{
	int rc, ret = 0;

	unamp_icm_array();

	rc = cmd_unmap_icm_aux();
	if (rc)
		eprintf("");

	ret |= rc;

	rc = cmd_unmap_fa();
	if (rc)
		eprintf("");
	ret |= rc;

	return ret;
}

static void *get_inprm_buf(void)
{
	return dev_buffers_p->inprm_buf;
}

static void *get_outprm_buf(void)
{
	return dev_buffers_p->outprm_buf;
}

static void *get_send_wqe_buf(void *wqe, __u8 index)
{
	struct ud_send_wqe_st *snd_wqe = wqe;

	return bus_to_virt(be32_to_cpu(snd_wqe->mpointer[index].local_addr_l));
}

static void *get_rcv_wqe_buf(void *wqe, __u8 index)
{
	struct recv_wqe_st *rcv_wqe = wqe;

	return bus_to_virt(be32_to_cpu(rcv_wqe->mpointer[index].local_addr_l));
}

static __u8 rate2mlx4rate(int rate)
{
	if (rate < 2 || rate > 10)
		return 8;

	return rate + 5;
}

static void modify_av_params(struct ud_av_st *av,
			     __u16 dlid,
			     __u8 g,
			     __u8 sl, __u8 rate, union ib_gid_u *gid, __u32 qpn)
{
	struct address_vector_st *p = &av->av;

	memset(p, 0, sizeof *p);

	p->fl_portn_pd = cpu_to_be32(dev_ib_data.port << 24 | dev_ib_data.pd);
	p->rlid = cpu_to_be16(dlid);
	p->g_mlid = g ? 0x80 : 0;
	p->sl_tclass_flow_label = cpu_to_be32(sl << 28);

	p->stat_rate = rate2mlx4rate(rate);

	if (g && gid) {
		p->rgid[0] = *((__u32 *) (&gid->raw[0]));
		p->rgid[1] = *((__u32 *) (&gid->raw[4]));
		p->rgid[2] = *((__u32 *) (&gid->raw[8]));
		p->rgid[3] = *((__u32 *) (&gid->raw[12]));
	} else
		memset(p->rgid, 0, sizeof(p->rgid));

	av->dest_qp = qpn;
	av->qkey = dev_ib_data.qkey;
}

static int post_rcv_buf(struct udqp_st *qp)
{
	qp->post_rcv_counter++;
	WRITE_WORD_VOL(&qp->rcv_uar_context, 2, htons(qp->post_rcv_counter));

	return 0;
}


/* the folowwing two functions are not used in this
   driver and appear only to supress warnings */
static void dev_post_dbell(void *dbell, __u32 offset)
{
}

static int cmd_post_doorbell(void *inprm, __u32 offset)
{
	dev_post_dbell(inprm, offset);
	return 0;
}

static int post_send_req(void *qph, void *wqeh, __u8 num_gather)
{
	struct udqp_st *qp = qph;
	struct ud_send_wqe_st *wqe = wqeh;
	__u32 hw_owner, ind;

	wqe->next.ds = cpu_to_be32((sizeof(wqe->next) + sizeof(wqe->udseg) +
		sizeof(wqe->mpointer[0]) * num_gather) >> 4);

	wqe->next.flags = cpu_to_be32(3 << 2); /* signaled completion */

	ind = qp->send_counter++; // ?? initialize
	hw_owner = ind & qp->max_snd_wqes ? (1 << 31) : 0;
	wqe->next.owner_opcode = cpu_to_be32(0xa | hw_owner);

	wmb();

	writel(htonl(qp->qpn << 8), qp->post_send_dbell);

	/* just to supress warnings */
	if (0)
		cmd_post_doorbell(NULL, 0);

	//	static void dev_post_dbell(void *dbell, __u32 offset);

	return 0;
}


static void init_send_wqe_owner_sw(void *wqe)
{
	__u32 *p = wqe;

	/* endianess taken care of by caller */
	*p = 0x80000000;
}

static int create_mads_qp(void **qp_pp, void **snd_cq_pp, void **rcv_cq_pp)
{
	__u8 i, k;
	int rc;
	struct udqp_st *qp;
	__u32 bus_addr;

	qp = &dev_ib_data.mads_qp;

	/* set the pointer to the receive WQEs buffer */
	qp->rcv_wq = dev_buffers_p->mads_qp_rcv_queue;

	qp->send_buf_sz = MAD_BUF_SZ;
	qp->rcv_buf_sz = MAD_BUF_SZ;

	qp->max_recv_wqes = NUM_MADS_RCV_WQES;	/* max wqes in this work queue */
	qp->recv_wqe_cur_free = NUM_MADS_RCV_WQES;	/* current free wqes */
	qp->recv_wqe_alloc_idx = 0;	/* index from wqes can be allocated if there are free wqes */

        memset(&qp->rcv_wq[0], 0, NUM_MADS_RCV_WQES * sizeof(qp->rcv_wq[0]));

	/* iterrate through the list */
	for (i = 0; i < NUM_MADS_RCV_WQES; ++i) {
		qp->rcv_bufs[i] = ib_buffers.rcv_mad_buf[i];
		bus_addr = virt_to_bus(qp->rcv_bufs[i]);
		qp->rcv_wq[i].wqe.mpointer[0].local_addr_l = bus_addr;
		qp->rcv_wq[i].wqe.mpointer[0].byte_count = GRH_SIZE;
		bus_addr = virt_to_bus(qp->rcv_bufs[i] + GRH_SIZE);
		qp->rcv_wq[i].wqe.mpointer[1].local_addr_l = bus_addr;
		qp->rcv_wq[i].wqe.mpointer[1].byte_count = MAD_BUF_SZ;

		/* set memory key */
		for (k = 0; k < ((sizeof(qp->rcv_wq[i])) >> 4); ++k)
			qp->rcv_wq[i].wqe.mpointer[k].lkey = dev_ib_data.mkey;
	}
	cpu_to_be_buf(&qp->rcv_wq[0],
		      NUM_MADS_RCV_WQES * sizeof(qp->rcv_wq[0]));

	/* set the pointer to the send WQEs buffer */
	qp->snd_wq = dev_buffers_p->mads_qp_snd_queue;

	qp->snd_wqe_alloc_idx = 0;
	qp->max_snd_wqes = NUM_MADS_SND_WQES;
	qp->snd_wqe_cur_free = NUM_MADS_SND_WQES;

	memset(&qp->snd_wq[0], 0, NUM_MADS_SND_WQES * sizeof(qp->snd_wq[i]));
	/* iterrate through the list */
	for (i = 0; i < NUM_MADS_SND_WQES; ++i) {
		/* set the allocated buffers */
		qp->snd_bufs[i] = ib_buffers.send_mad_buf[i];
		bus_addr = virt_to_bus(qp->snd_bufs[i]);
		qp->snd_wq[i].wqe_cont.wqe.mpointer[0].local_addr_l = bus_addr;
		qp->snd_wq[i].wqe_cont.wqe.mpointer[0].lkey = dev_ib_data.mkey;
		qp->snd_wq[i].wqe_cont.wqe.mpointer[0].byte_count = qp->send_buf_sz;
		init_send_wqe_owner_sw(&qp->snd_wq[i].wqe_cont.wqe);
	}

	cpu_to_be_buf(&qp->snd_wq[0],
		      NUM_MADS_SND_WQES * sizeof(qp->snd_wq[i]));

	/* qp number and cq numbers are already set up */
	qp->snd_cq.cq_buf = dev_buffers_p->mads_snd_cq_buf;
	qp->rcv_cq.cq_buf = dev_buffers_p->mads_rcv_cq_buf;
	qp->snd_cq.num_cqes = NUM_MADS_SND_CQES;
	qp->rcv_cq.num_cqes = NUM_MADS_RCV_CQES;
	qp->qkey = GLOBAL_QKEY;
	rc = create_udqp(qp);
	if (!rc) {
		*qp_pp = qp;
		*snd_cq_pp = &qp->snd_cq;
		*rcv_cq_pp = &qp->rcv_cq;
	}

	return rc;
}

static int create_ipoib_qp(void **qp_pp,
			   void **snd_cq_pp, void **rcv_cq_pp, __u32 qkey)
{
	__u8 i, k;
	int rc;
	struct udqp_st *qp;
	__u32 bus_addr;

	qp = &dev_ib_data.ipoib_qp;

	/* set the pointer to the receive WQEs buffer */
	qp->rcv_wq = dev_buffers_p->ipoib_qp_rcv_queue;

	qp->send_buf_sz = IPOIB_SND_BUF_SZ;
	qp->rcv_buf_sz = IPOIB_RCV_BUF_SZ;

	qp->max_recv_wqes = NUM_IPOIB_RCV_WQES;
	qp->recv_wqe_cur_free = NUM_IPOIB_RCV_WQES;
	qp->recv_wqe_alloc_idx = 0;	/* index from wqes can be allocated if there are free wqes */

	memset(&qp->rcv_wq[0], 0, NUM_IPOIB_RCV_WQES * sizeof(qp->rcv_wq[0]));

	for (i = 0; i < NUM_IPOIB_RCV_WQES; ++i) {
		qp->rcv_bufs[i] = ib_buffers.ipoib_rcv_buf[i];
		bus_addr = virt_to_bus(qp->rcv_bufs[i]);
		qp->rcv_wq[i].wqe.mpointer[0].local_addr_l = bus_addr;
		qp->rcv_wq[i].wqe.mpointer[0].byte_count = GRH_SIZE;
		bus_addr = virt_to_bus(qp->rcv_bufs[i] + GRH_SIZE);
		qp->rcv_wq[i].wqe.mpointer[1].local_addr_l = bus_addr;
		qp->rcv_wq[i].wqe.mpointer[1].byte_count = IPOIB_RCV_BUF_SZ;

		for (k = 0; k < ((sizeof(qp->rcv_wq[i])) >> 4); ++k)
			qp->rcv_wq[i].wqe.mpointer[k].lkey = dev_ib_data.mkey;
	}
	cpu_to_be_buf(&qp->rcv_wq[0],
		      NUM_IPOIB_RCV_WQES * sizeof(qp->rcv_wq[0]));


	// set the pointer to the send WQEs buffer 
	qp->snd_wq = dev_buffers_p->ipoib_qp_snd_queue;

	qp->snd_wqe_alloc_idx = 0;
	qp->max_snd_wqes = NUM_IPOIB_SND_WQES;
	qp->snd_wqe_cur_free = NUM_IPOIB_SND_WQES;

	memset(&qp->snd_wq[0], 0, NUM_IPOIB_SND_WQES * sizeof(qp->snd_wq[i]));

	for (i = 0; i < NUM_IPOIB_SND_WQES; ++i) {
		qp->snd_bufs[i] = ib_buffers.send_ipoib_buf[i];
		bus_addr = virt_to_bus(qp->snd_bufs[i]);
		qp->snd_wq[i].wqe_cont.wqe.mpointer[0].local_addr_l = bus_addr;
		qp->snd_wq[i].wqe_cont.wqe.mpointer[0].lkey = dev_ib_data.mkey;
		init_send_wqe_owner_sw(&qp->snd_wq[i].wqe_cont.wqe);
	}

	cpu_to_be_buf(&qp->snd_wq[0],
		      NUM_IPOIB_SND_WQES * sizeof(qp->snd_wq[i]));

	// qp number and cq numbers are already set up 
	qp->snd_cq.cq_buf = dev_buffers_p->ipoib_snd_cq_buf;
	qp->rcv_cq.cq_buf = dev_buffers_p->ipoib_rcv_cq_buf;
	qp->snd_cq.num_cqes = NUM_IPOIB_SND_CQES;
	qp->rcv_cq.num_cqes = NUM_IPOIB_RCV_CQES;
	qp->qkey = qkey;
	rc = create_udqp(qp);
	if (!rc) {
		*qp_pp = qp;
		*snd_cq_pp = &qp->snd_cq;
		*rcv_cq_pp = &qp->rcv_cq;
	}

	return rc;
}

static int create_udqp(struct udqp_st *qp)
{
	int rc, ret = 0;
	void *inprm;
	struct recv_wqe_st *rcv_wqe;
	unsigned mtt_offset, num_pages;
	struct cq_st *cq;
	int qsz;

        inprm = dev_buffers_p->inprm_buf;

        /* create send CQ */
	cq = &qp->snd_cq;
	num_pages = get_num_pages(sizeof(cq->cq_buf[0]) * cq->num_cqes, cq->cq_buf);
        mtt_offset = mtt_alloc(num_pages);
	prep_write_mtt_buf(inprm, mtt_offset, cq->cq_buf, num_pages);
	rc = cmd_write_mtt(inprm, num_pages);
	if (rc) {
		eprintf("");
		return -1;
	}



	cq->set_ci_db_record = bus_to_virt(lalign(virt_to_bus(cq->db_rec_buf), 8));
	*cq->set_ci_db_record = 0;
	cq->cons_counter = 0;
        prep_sw2hw_cq_buf(inprm,
			  dev_ib_data.eq.eqn,
			  mtt_offset,
			  get_page_offset(cq->cq_buf),
			  cq->set_ci_db_record);

	rc = cmd_sw2hw_cq(qp->snd_cq.cqn, inprm, SW2HW_CQ_IBUF_SZ);
	if (rc) {
		ret = -1;
		eprintf("");
		goto exit;
	}
	
	/* create receive CQ */
	cq = &qp->rcv_cq;
	num_pages = get_num_pages(sizeof(cq->cq_buf[0]) * cq->num_cqes, cq->cq_buf);
        mtt_offset = mtt_alloc(num_pages);
	prep_write_mtt_buf(inprm, mtt_offset, cq->cq_buf, num_pages);
	rc = cmd_write_mtt(inprm, num_pages);
	if (rc) {
		eprintf("");
		ret = -1;
		goto undo_snd_cq;
	}

	cq->set_ci_db_record = bus_to_virt(lalign(virt_to_bus(cq->db_rec_buf), 8));
	*cq->set_ci_db_record = 0;
	cq->cons_counter = 0;
        prep_sw2hw_cq_buf(inprm,
			  dev_ib_data.eq.eqn,
			  mtt_offset,
			  get_page_offset(cq->cq_buf),
			  cq->set_ci_db_record);

	rc = cmd_sw2hw_cq(qp->rcv_cq.cqn, inprm, SW2HW_CQ_IBUF_SZ);
	if (rc) {
		eprintf("");
		ret = -1;
		goto undo_snd_cq;
	}

	/* create the qp */
	qp->send_counter = 0;
	qp->post_rcv_counter = 0;
	qp->post_send_dbell = hermon_pci_dev.uar + 0x14;
	qsz = sizeof(qp->rcv_wq[0]) * qp->max_recv_wqes +
		sizeof(qp->snd_wq[0]) * qp->max_snd_wqes;
	num_pages = get_num_pages(qsz, qp->snd_wq);
        mtt_offset = mtt_alloc(num_pages);
	prep_write_mtt_buf(inprm, mtt_offset, qp->snd_wq, num_pages);
	rc = cmd_write_mtt(inprm, num_pages);
	if (rc) {
		ret = -1;
		eprintf("");
		goto exit;
	}
	
	prep_rst2init_qpee_buf(inprm,
			       qp->snd_cq.cqn,
			       qp->rcv_cq.cqn,
			       qp->qkey,
			       my_log2(qp->max_recv_wqes),
			       my_log2(sizeof(qp->rcv_wq[0])),
			       my_log2(qp->max_snd_wqes),
			       my_log2(sizeof(qp->snd_wq[0])),
			       virt_to_bus(&qp->rcv_uar_context),
			       mtt_offset,
			       get_page_offset(qp->snd_wq));


	rc = cmd_rst2init_qpee(qp->qpn, inprm, QPCTX_IBUF_SZ);
	if (rc) {
		ret = -1;
		eprintf("");
		goto undo_rcv_cq;
	}

	/* post all the buffers to the receive queue */
	while (1) {
		/* allocate wqe */
		rcv_wqe = alloc_rcv_wqe(qp);
		if (!rcv_wqe)
			break;

		/* post the buffer */
		rc = post_rcv_buf(qp);
		if (rc) {
			ret = -1;
			eprintf("");
			goto undo_rcv_cq;
		}
	}

	prep_init2rtr_qpee_buf(inprm, 0); // ?? sl
	rc = cmd_init2rtr_qpee(qp->qpn, inprm, QPCTX_IBUF_SZ);
	if (rc) {
		ret = -1;
		eprintf("");
		goto undo_rcv_cq;
	}

	memset(inprm, 0, QPCTX_IBUF_SZ);
	rc = cmd_rtr2rts_qpee(qp->qpn, inprm, QPCTX_IBUF_SZ);
	if (rc) {
		ret = -1;
		eprintf("");
		goto undo_rcv_cq;
	}

	goto exit;

undo_rcv_cq:
	rc = cmd_hw2sw_cq(qp->rcv_cq.cqn);
	if (rc)
		eprintf("");

undo_snd_cq:
	rc = cmd_hw2sw_cq(qp->snd_cq.cqn);
	if (rc)
		eprintf("");

exit:
	return ret;
}

static int destroy_udqp(struct udqp_st *qp)
{
	int rc;

	rc = cmd_2err_qpee(qp->qpn);
	if (rc) {
		eprintf("");
		return rc;
	}

	rc = cmd_2rst_qpee(qp->qpn);
	if (rc) {
		eprintf("");
		return rc;
	}

	rc = cmd_hw2sw_cq(qp->rcv_cq.cqn);
	if (rc) {
		eprintf("");
		return rc;
	}

	rc = cmd_hw2sw_cq(qp->snd_cq.cqn);
	if (rc) {
		eprintf("");
		return rc;
	}

	return rc;
}

static void prep_send_wqe_buf(void *qph,
			      void *avh,
			      void *wqeh,
			      const void *buf,
			      unsigned int offset, __u16 len, __u8 e)
{
	struct ud_send_wqe_st *snd_wqe = wqeh;
	struct ud_av_st *av = avh;

	/* suppress warnings */
	if (qph){
	}

	if (e) {
	}

	
	memcpy(&snd_wqe->udseg.av, &av->av, sizeof av->av);
	snd_wqe->udseg.dest_qp = cpu_to_be32(av->dest_qp);
	snd_wqe->udseg.qkey = cpu_to_be32(av->qkey);

	if (buf) {
		memcpy(bus_to_virt(be32_to_cpu(snd_wqe->mpointer[0].local_addr_l)) +
		       offset, buf, len);
		len += offset;
	}
	snd_wqe->mpointer[0].byte_count = cpu_to_be32(len);
}

static void *alloc_ud_av(void)
{
	u8 next_free;

	if (dev_ib_data.udav.udav_next_free == FL_EOL)
		return NULL;

	next_free = dev_ib_data.udav.udav_next_free;
	dev_ib_data.udav.udav_next_free =
	    dev_buffers_p->av_array[next_free].ud_av.next_free;
	return &dev_buffers_p->av_array[next_free].ud_av;
}

static void free_ud_av(void *avh)
{
	union ud_av_u *avu;
	__u8 idx, old_idx;
	struct ud_av_st *av = avh;

	avu = (union ud_av_u *)av;

	idx = avu - dev_buffers_p->av_array;
	old_idx = dev_ib_data.udav.udav_next_free;
	dev_ib_data.udav.udav_next_free = idx;
	avu->ud_av.next_free = old_idx;
}

static int update_cq_cons_idx(struct cq_st *cq)
{
	/* write doorbell record */

	*cq->set_ci_db_record = htonl(cq->cons_counter & 0xffffff);

	return 0;
}

static int poll_cq(void *cqh, union cqe_st *cqe_p, u8 *num_cqes)
{
	struct cq_st *cq = cqh;
	int hw_ownership, idx, rc;

	idx = cq->cons_counter & (cq->num_cqes - 1);

	hw_ownership = !!(cq->cq_buf[idx].good_cqe.owner_opcode & 0x80) ^
		!!(cq->cons_counter & cq->num_cqes);
	if (hw_ownership)
		*num_cqes = 0;
	else {
		barrier();
		*cqe_p = cq->cq_buf[idx];
		++cq->cons_counter;
		rc = update_cq_cons_idx(cq);
		if (rc) {
			eprintf("");
			return rc;
		}
		*num_cqes = 1;
	}

	return 0;
}

static struct udqp_st *qpn2qp(__u32 qpn)
{
	if (qpn == dev_ib_data.mads_qp.qpn)
		return &dev_ib_data.mads_qp;
	else if (qpn == dev_ib_data.ipoib_qp.qpn)
		return &dev_ib_data.ipoib_qp;
	else
		return NULL;
}



static void dev2ib_cqe(struct ib_cqe_st *ib_cqe_p, union cqe_st *cqe_p)
{
	__u16 counter;
	__u32 qpn;
	struct udqp_st *qp;
	int i;

	ib_cqe_p->is_error = (cqe_p->good_cqe.owner_opcode & 0x1f) == 0x1e;

	if (!ib_cqe_p->is_error) {
		ib_cqe_p->is_send = !!(cqe_p->good_cqe.owner_opcode & 0x40);
		counter = be16_to_cpu(cqe_p->good_cqe.wqe_counter);
		qpn = be32_to_cpu(cqe_p->good_cqe.flags_qpn) & 0xffffff;
		qp = qpn2qp(qpn);
		if (!qp) {
			eprintf("qpn = 0x%x\n", qpn);
			ib_cqe_p->is_error = 1;
			return;
		}

		if (ib_cqe_p->is_send) {
			i = counter & (qp->max_snd_wqes - 1);
			ib_cqe_p->wqe = &qp->snd_wq[i];
		}
		else {
			i = counter & (qp->max_recv_wqes - 1);
			ib_cqe_p->wqe = &qp->rcv_wq[i];
		}
		ib_cqe_p->count = be32_to_cpu(cqe_p->good_cqe.byte_count);
	}
}

static int ib_poll_cq(void *cqh, struct ib_cqe_st *ib_cqe_p, u8 *num_cqes)
{
	int rc;
	union cqe_st cqe;
	struct cq_st *cq = cqh;

	rc = poll_cq(cq, &cqe, num_cqes);
	if (rc || (*num_cqes == 0))
		return rc;

	dev2ib_cqe(ib_cqe_p, &cqe);

	return rc;
}

struct mg_st {
	__u32 next_MGI_index;
	__u32 members_count;
	__u32 rsvd1[2];
	__u32 mgi_128_96;
	__u32 mgi_95_64;
	__u32 mgi_63_32;
	__u32 mgi_31_0;
	__u32 mgi_qp[8];
} __attribute__ ((packed));

/* always work on ipoib qp */
static int add_qp_to_mcast_group(union ib_gid_u mcast_gid, __u8 add)
{
	struct mg_st *mg;
	__u8 *tmp;
	int rc;
	__u16 mgid_hash;

	tmp = dev_buffers_p->inprm_buf;
	memcpy(tmp, mcast_gid.raw, 16);
	be_to_cpu_buf(tmp, 16);
	rc = cmd_mgid_hash(tmp, &mgid_hash);
	if (!rc) {
		mg = (struct mg_st *)dev_buffers_p->inprm_buf;
		memset(mg, 0, sizeof *mg);
		mg->mgi_128_96 = be32_to_cpu(mcast_gid.as_u32.dw[0]);
		mg->mgi_95_64 = be32_to_cpu(mcast_gid.as_u32.dw[1]);
		mg->mgi_63_32 = be32_to_cpu(mcast_gid.as_u32.dw[2]);
		mg->mgi_31_0 = be32_to_cpu(mcast_gid.as_u32.dw[3]);
		if (add) {
			mg->mgi_qp[0] = (1 << 30) | dev_ib_data.ipoib_qp.qpn;
			mg->members_count = 1;
		}
		rc = cmd_write_mcg(mg, mgid_hash);
	}
	return rc;
}

static int clear_interrupt(void)
{
	writel(dev_ib_data.clr_int_data, dev_ib_data.clr_int_addr);
	return 0;
}

static struct ud_send_wqe_st *alloc_send_wqe(udqp_t qph)
{
	struct udqp_st *qp = qph;
	__u32 idx;

	if (qp->snd_wqe_cur_free) {
		qp->snd_wqe_cur_free--;
		idx = qp->snd_wqe_alloc_idx;
		qp->snd_wqe_alloc_idx =
		    (qp->snd_wqe_alloc_idx + 1) & (qp->max_snd_wqes - 1);
		return &qp->snd_wq[idx].wqe_cont.wqe;
	}

	return NULL;
}

static struct recv_wqe_st *alloc_rcv_wqe(struct udqp_st *qp)
{
	__u32 idx;

	if (qp->recv_wqe_cur_free) {
		qp->recv_wqe_cur_free--;
		idx = qp->recv_wqe_alloc_idx;
		qp->recv_wqe_alloc_idx =
		    (qp->recv_wqe_alloc_idx + 1) & (qp->max_recv_wqes - 1);
		return &qp->rcv_wq[idx].wqe;
	}

	return NULL;
}

static int free_send_wqe(struct udqp_st *qp)
{
	qp->snd_wqe_cur_free++;

	return 0;
}

static int free_rcv_wqe(struct udqp_st *qp)
{
	qp->recv_wqe_cur_free++;

	return 0;
}

static int free_wqe(void *wqe)
{
	int rc = 0;
	struct recv_wqe_st *rcv_wqe;

	if ((wqe >= (void *)(dev_ib_data.ipoib_qp.rcv_wq)) &&
	    (wqe <
	     (void *)(&dev_ib_data.ipoib_qp.rcv_wq[NUM_IPOIB_RCV_WQES]))) {
		/* ipoib receive wqe */
		free_rcv_wqe(&dev_ib_data.ipoib_qp);
		rcv_wqe = alloc_rcv_wqe(&dev_ib_data.ipoib_qp);
		if (rcv_wqe) {
			rc = post_rcv_buf(&dev_ib_data.ipoib_qp);
			if (rc) {
				eprintf("");
			}
		}
		else
			eprintf("allocation failed");

	} else if (wqe >= (void *)(dev_ib_data.ipoib_qp.snd_wq) &&
		   wqe <
		   (void *)(&dev_ib_data.ipoib_qp.snd_wq[NUM_IPOIB_SND_WQES])) {
		/* ipoib send wqe */
		free_send_wqe(&dev_ib_data.ipoib_qp);
	} else if (wqe >= (void *)(dev_ib_data.mads_qp.rcv_wq) &&
		   wqe <
		   (void *)(&dev_ib_data.mads_qp.rcv_wq[NUM_MADS_RCV_WQES])) {
		/* mads receive wqe */
		free_rcv_wqe(&dev_ib_data.mads_qp);
		rcv_wqe = alloc_rcv_wqe(&dev_ib_data.mads_qp);
		if (rcv_wqe) {
			rc = post_rcv_buf(&dev_ib_data.mads_qp);
			if (rc) {
				eprintf("");
			}
		}
	} else if (wqe >= (void *)(dev_ib_data.mads_qp.snd_wq) &&
		   wqe <
		   (void *)(&dev_ib_data.mads_qp.snd_wq[NUM_MADS_SND_WQES])) {
		/* mads send wqe */
		free_send_wqe(&dev_ib_data.mads_qp);
	} else {
		rc = -1;
		eprintf("");
	}

	return rc;
}

static int update_eq_cons_idx(struct eq_st *eq)
{
        writel(htonl(eq->cons_counter & 0xffffff), eq->eq_doorbell);
	return 0;
}

static int poll_eq(struct ib_eqe_st *ib_eqe_p, __u8 *num_eqes)
{
	struct eq_st *eq = &dev_ib_data.eq;
	int hw_ownership, idx, rc;

	idx = eq->cons_counter & (eq->eq_size - 1);

	hw_ownership = !!(eq->eq_buf[idx].owner & 0x80) ^
		!!(eq->cons_counter & eq->eq_size);

	if (hw_ownership)
		*num_eqes = 0;
	else {
		barrier();
		ib_eqe_p->event_type = eq->eq_buf[idx].event_type;
		if (ib_eqe_p->event_type == 0)
			/* cq event */
			ib_eqe_p->cqn = be32_to_cpu(eq->eq_buf[idx].edata.cq.cqn);
		else
			ib_eqe_p->cqn = 0;

		eq->cons_counter++;
		rc = update_eq_cons_idx(eq);
		if (rc) {
			eprintf("");
			*num_eqes = 0;
			return rc;
		}
		*num_eqes = 1;
	}

	return 0;
}

static int ib_device_close(void)
{
	iounmap(hermon_pci_dev.uar);
	iounmap(hermon_pci_dev.cr_space);
	return 0;
}

static __u32 dev_get_qpn(void *qph)
{
	struct udqp_st *qp = qph;

	return qp->qpn;
}

static int poll_error_buf(__u32 error_buf_size)
{
	__u32 ptr= dev_ib_data.error_buf_addr;
	__u32 i, rc;

	for (i=0; i<error_buf_size; ++i, ptr+=4) {
                gw_read_cr(ptr, &rc);
		if (rc) 
			return -1;
	}

	return 0;
}

