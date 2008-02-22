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

/* getting the toggle bit from the HCR */
static int get_toggle(int *curr_toggle)
{
	int rc;
	__u32 result;

	rc = gw_read_cr(HCR_OFFSET_GO, &result);
	if (rc) {
		eprintf("");
		return rc;
	}
	*curr_toggle = (result & 0x200000) != 0;
	return 0;
}


static int cmdif_is_free(int *is_free)
{
	int rc;
	__u32 result;

	rc = gw_read_cr(HCR_OFFSET_GO, &result);
	if (rc) {
		eprintf("");
		return rc;
	}
	*is_free = (result & 0x800000) == 0;
	return 0;
}

static void edit_hcr(command_fields_t *cmd_prms, __u32 *buf)
{
	unsigned int i;

	switch (cmd_prms->in_trans) {
	case TRANS_NA:
		/* note! since these are zeroes I do not bother to deal with endianess */
		buf[0] = 0;
		buf[1] = 0;
		break;

	case TRANS_IMMEDIATE:
		buf[0] = cmd_prms->in_param[0];
		buf[1] = cmd_prms->in_param[1];
		break;

	case TRANS_MAILBOX:
		buf[0] = 0;
		buf[1] = virt_to_bus(cmd_prms->in_param);

		for (i = 0; i < cmd_prms->in_param_size; i += 4)
			cmd_prms->in_param[i >> 2] =
			    cpu_to_be32(cmd_prms->in_param[i >> 2]);
		break;
	}

	buf[2] = cmd_prms->input_modifier;

	switch (cmd_prms->out_trans) {
	case TRANS_NA:
		/* note! since these are zeroes I do not bother to deal with endianess */
		buf[3] = 0;
		buf[4] = 0;
		break;

	case TRANS_IMMEDIATE:
		break;
	case TRANS_MAILBOX:
		buf[3] = 0;
		buf[4] = virt_to_bus(cmd_prms->out_param);
		break;
	}

	buf[5] = 0;		/* token is always 0 */
	buf[6] = cmd_prms->opcode |	/* opcode */
	    0x800000 |		/* go bit */
	    ((cmd_prms->opcode_modifier & 0xf) << 12);	/* opcode modifier */
}


static int wait_cmdif_free(int *last_toggle)
{
	int ret, is_free;
	unsigned int i, relax_time = 1, max_time = 5000;
	int curr_toggle;

	/* wait until go bit is free */
	for (i = 0; i < max_time; i += relax_time) {
                /* First checking toggle bit was changed */
		ret = get_toggle(&curr_toggle);
		if (ret) {
			return ret;
		}
		if (*last_toggle == curr_toggle) {
			ret = cmdif_is_free(&is_free);
			if (ret) {
				return ret;
			}
			if (is_free) {
				return 0;
			}
		}
		mdelay(relax_time);
	}
	if (i >= max_time)
		return -1;
	return 0;
}


/* 
* cmd_invoke:
* In Hermon we use the toggle bit to determine is the command started 
* the operation.
*/
static struct dev_pci_struct hermon_pci_dev;
static XHH_cmd_status_t cmd_invoke(command_fields_t *cmd_prms)
{
	int ret, is_free, i, last_toggle;
	__u32 hcr[7], data;
	__u8 status;

	/* check if go bit is free */
	ret = cmdif_is_free(&is_free);
	if (ret) {
                eprintf("");
		return -1;
	}

	__asm__ __volatile__("":::"memory");
	/* it must be free */
	if (!is_free) {
		eprintf("");
		return -1;
	}
	__asm__ __volatile__("":::"memory");
	edit_hcr(cmd_prms, hcr);
	__asm__ __volatile__("":::"memory");

	/* using the toggle bit to determine if the command arrived */
	ret = get_toggle(&last_toggle);
	if (ret) {
		eprintf("");
		return -1;
	}

	/* writing the oposite toggle bit */
	if (last_toggle) {
		last_toggle = 0;
		MT_INSERT32(hcr[6],last_toggle,21,1);
        } else {
		last_toggle = 1;
		MT_INSERT32(hcr[6],last_toggle,21,1); 
	}

	DBG ( "Issuing command:\n" );
	uint32_t hack[7];
	memcpy ( hack, hcr, sizeof ( hack ) );
	be_to_cpu_buf ( hack, sizeof ( hack ) );
	DBG_HDA ( virt_to_phys ( hermon_pci_dev.cr_space + HCR_BASE ),
		  hack, sizeof ( hack ) );
	if ( cmd_prms->in_trans == TRANS_MAILBOX ) {
		DBG2 ( "Input mailbox:\n" );
		DBG2_HDA ( virt_to_phys ( cmd_prms->in_param ),
			   cmd_prms->in_param, cmd_prms->in_param_size );
	}
    
	for (i = 0; i < 7; ++i) {
		ret = gw_write_cr(HCR_BASE + i * 4, hcr[i]);
        if (ret) {
			eprintf("");
			return -1;
		}
	}

	__asm__ __volatile__("":::"memory");
	/* wait for completion */
	ret = wait_cmdif_free(&last_toggle);
	if (ret) {
		eprintf("");
		return -1;
	}

#if 0
	int z;
	for ( z = 0 ; z < 7 ; z++ ) {
		gw_read_cr ( HCR_BASE + z * 4, &hack[z] );
	}
	be_to_cpu_buf ( hack, sizeof ( hack ) );
	DBG ( "Response:\n" );
	DBG_HDA ( virt_to_phys ( hermon_pci_dev.cr_space + HCR_BASE ),
		  hack, sizeof ( hack ) );
#endif

	__asm__ __volatile__("":::"memory");
	ret = gw_read_cr(HCR_OFFSET_STATUS, &data);
	if (ret) {
		eprintf("");
		return -1;
	}

	status = data >> 24;

	if (status) 
		return status;

	if (cmd_prms->out_trans == TRANS_MAILBOX) {
		DBG2 ( "Output mailbox:\n" );
		DBG2_HDA ( virt_to_phys ( cmd_prms->out_param ),
			   cmd_prms->out_param, cmd_prms->out_param_size );
		be_to_cpu_buf(cmd_prms->out_param, cmd_prms->out_param_size);
	} else if (cmd_prms->out_trans == TRANS_IMMEDIATE) {
		if (gw_read_cr(HCR_OFFSET_OUTPRM_H, &cmd_prms->out_param[0]))
			return -1;
		if (gw_read_cr(HCR_OFFSET_OUTPRM_L, &cmd_prms->out_param[1]))
			return -1;
		DBG2 ( "Output:\n" );
		uint32_t x[2];
		x[0] = ntohl ( cmd_prms->out_param[0] );
		x[1] = ntohl ( cmd_prms->out_param[1] );
		DBG2_HDA ( virt_to_phys ( hermon_pci_dev.cr_space +
					  HCR_BASE + 3*4 ), x, 8 );
	}

	return 0;
}

/* 
 * START OF COMMANDS 
 *
 *
*/

/* new command */
static int cmd_write_mtt(void *inprm, int num_mtt)
{
	int rc;
	command_fields_t cmd_desc;
	
	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.input_modifier = num_mtt; 
	cmd_desc.opcode = XDEV_CMD_WRITE_MTT;
	cmd_desc.in_param = inprm;
	cmd_desc.in_param_size = 16 + 8 * num_mtt;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}



/* commands taken from common */
/*
 *  cmd_close_hca
 */
static int cmd_close_hca(int panic)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = XDEV_CMD_CLOSE_HCA;
	cmd_desc.opcode_modifier= panic;
	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_init_hca
 */
static int cmd_init_hca(__u32 *inprm, __u32 in_prm_size)
{
	int rc;

	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.opcode = XDEV_CMD_INIT_HCA;
	cmd_desc.in_param = inprm;
	cmd_desc.in_param_size = in_prm_size;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_sw2hw_eq
 */
static int cmd_sw2hw_eq(__u8 eqn, __u32 inprm_sz)
{
	int rc;
	command_fields_t cmd_desc;
	void *inprm;

	memset(&cmd_desc, 0, sizeof cmd_desc);

	inprm = get_inprm_buf();
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.input_modifier = eqn;
	cmd_desc.opcode = XDEV_CMD_SW2HW_EQ;
	cmd_desc.in_param = inprm;
	cmd_desc.in_param_size = inprm_sz;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_hw2sw_eq
 */
static int cmd_hw2sw_eq(__u8 eqn)
{
	int rc;
	command_fields_t cmd_desc;
	void *outprm;

	memset(&cmd_desc, 0, sizeof cmd_desc);

	outprm = get_outprm_buf();
	cmd_desc.opcode = XDEV_CMD_HW2SW_EQ;
	cmd_desc.input_modifier = eqn;
	cmd_desc.out_trans = TRANS_MAILBOX;
	cmd_desc.out_param = outprm;
	cmd_desc.out_param_size = 0x40;
	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_map_eq
 */
static int cmd_map_eq(__u8 eqn, __u32 mask, int map)
{
	int rc;
	command_fields_t cmd_desc;
	__u32 *inprm;

	memset(&cmd_desc, 0, sizeof cmd_desc);

	inprm = get_inprm_buf();

	inprm[1] = mask;
	inprm[0] = 0;

	cmd_desc.opcode = XDEV_CMD_MAP_EQ;
	cmd_desc.in_trans = TRANS_IMMEDIATE;
	cmd_desc.in_param = inprm;
	cmd_desc.input_modifier = ((map ? 0 : 1) << 31) | eqn;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_sw2hw_mpt
 */
static int cmd_sw2hw_mpt(__u32 *lkey, __u32 in_key, __u32 *inprm,
			 __u32 inprm_sz)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.opcode = XDEV_CMD_SW2HW_MPT;
	cmd_desc.input_modifier = in_key & MKEY_IDX_MASK;	/* only one MR for the whole driver */
	cmd_desc.in_param = inprm;
	cmd_desc.in_param_size = inprm_sz;

	rc = cmd_invoke(&cmd_desc);
	if (!rc)
		*lkey = in_key;

	return rc;
}

/*
 *  cmd_hw2sw_mpt
 */
static int cmd_hw2sw_mpt(__u32 key)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = XDEV_CMD_HW2SW_MPT;
	cmd_desc.input_modifier = key & MKEY_IDX_MASK;
	cmd_desc.opcode_modifier = 1;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_init_ib
 */
static int cmd_init_ib(__u32 port, void *inprm, __u32 inprm_sz)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = XDEV_CMD_INIT_IB;
	cmd_desc.input_modifier = port;

	/* TODO: this should be delete when moving to new prm */
	cmd_desc.in_trans = TRANS_MAILBOX; 
	cmd_desc.in_param = inprm;         
	cmd_desc.in_param_size = inprm_sz; 

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_close_ib
 */
static int cmd_close_ib(__u32 port)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = XDEV_CMD_CLOSE_IB;
	cmd_desc.input_modifier = port;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_sw2hw_cq
 */
static int cmd_sw2hw_cq(__u32 cqn, __u32 *inprm, __u32 inprm_sz)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = XDEV_CMD_SW2HW_CQ;
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.in_param = inprm;
	cmd_desc.in_param_size = inprm_sz;
	cmd_desc.input_modifier = cqn;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}



/*
 *  cmd_hw2sw_cq
 */
static int cmd_hw2sw_cq(__u32 cqn)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = XDEV_CMD_HW2SW_CQ;
	cmd_desc.input_modifier = cqn;
	cmd_desc.out_trans = TRANS_MAILBOX;
	cmd_desc.out_param = get_outprm_buf();

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_rst2init_qpee
 */
static int cmd_rst2init_qpee(__u32 qpn, __u32 *inprm, __u32 inprm_sz)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = XDEV_CMD_RST2INIT_QPEE;
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.in_param = inprm;
	cmd_desc.in_param_size = inprm_sz;
	cmd_desc.input_modifier = qpn;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_init2rtr_qpee
 */
static int cmd_init2rtr_qpee(__u32 qpn, __u32 *inprm, __u32 inprm_sz)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = XDEV_CMD_INIT2RTR_QPEE;
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.in_param = inprm;
	cmd_desc.in_param_size = inprm_sz;
	cmd_desc.input_modifier = qpn;;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_rtr2rts_qpee
 */
static int cmd_rtr2rts_qpee(__u32 qpn, __u32 *inprm, __u32 inprm_sz)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = XDEV_CMD_RTR2RTS_QPEE;
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.in_param = inprm;
	cmd_desc.in_param_size = inprm_sz;
	cmd_desc.input_modifier = qpn;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_2rst_qpee
 */
static int cmd_2rst_qpee(__u32 qpn)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = XDEV_CMD_ERR2RST_QPEE;
	cmd_desc.opcode_modifier = 0;
	cmd_desc.input_modifier = qpn;
	cmd_desc.out_trans = TRANS_MAILBOX;
	cmd_desc.out_param = get_outprm_buf();

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_2err_qpee
 */
static int cmd_2err_qpee(__u32 qpn)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = XDEV_CMD_2ERR_QPEE;
	cmd_desc.input_modifier = qpn;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

static int cmd_mad_ifc(void *inprm, struct ib_mad_st *mad, __u8 port)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = XDEV_CMD_MAD_IFC;
	cmd_desc.opcode_modifier = 1;	/* no mkey/bkey validation */
	cmd_desc.input_modifier = port;
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.in_param_size = 256;
	cmd_desc.in_param = (__u32 *) inprm;
	cmd_desc.out_trans = TRANS_MAILBOX;
	cmd_desc.out_param = (__u32 *) mad;
	cmd_desc.out_param_size = 256;
	rc = cmd_invoke(&cmd_desc);

	return rc;
}

static int cmd_mgid_hash(__u8 *gid, __u16 *mgid_hash_p)
{
	int rc;
	command_fields_t cmd_desc;
	__u16 result[2];

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = XDEV_CMD_MGID_HASH;
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.in_param = (__u32 *)gid;
	cmd_desc.in_param_size = 16;
	cmd_desc.out_trans = TRANS_IMMEDIATE;

	rc = cmd_invoke(&cmd_desc);
	if (!rc) {
		rc = gw_read_cr(HCR_BASE + 16, (__u32 *)result);
		if (!rc)
			*mgid_hash_p = result[0];
	}

	return rc;
}


/* OTHER COMMANDS */

/*
 *  cmd_write_mcg
 *  
 */
static int cmd_write_mcg(void *mg, __u16 index)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = HERMON_CMD_WRITE_MCG;
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.in_param_size = MT_STRUCT_SIZE(hermonprm_mgm_entry_st);
	cmd_desc.in_param = (__u32 *) mg;
	cmd_desc.input_modifier = index;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}


/*
 *  cmd_mod_stat_cfg
 */
static int cmd_mod_stat_cfg(void)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);
	cmd_desc.opcode = HERMON_CMD_MOD_STAT_CFG;
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.in_param_size = MT_STRUCT_SIZE(hermonprm_mod_stat_cfg_st);
	cmd_desc.in_param = get_inprm_buf();
	memset(cmd_desc.in_param, 0, cmd_desc.in_param_size);

	rc = cmd_invoke(&cmd_desc);

	return rc;
}


/*
 *  cmd_query_fw
 */
static int cmd_query_fw(struct query_fw_st *qfw)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = HERMON_CMD_QUERY_FW;
	cmd_desc.out_trans = TRANS_MAILBOX;
	cmd_desc.out_param = get_outprm_buf();
	cmd_desc.out_param_size = MT_STRUCT_SIZE(hermonprm_query_fw_st);

	rc = cmd_invoke(&cmd_desc);
	if (!rc) {
		qfw->fw_rev_major =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_fw_st, fw_rev_major);
		qfw->fw_rev_minor =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_fw_st, fw_rev_minor);
		qfw->fw_rev_subminor =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_fw_st, fw_rev_subminor);

		qfw->error_buf_offset_h = 
		    EX_FLD(cmd_desc.out_param, hermonprm_query_fw_st, error_buf_offset_h);
		qfw->error_buf_offset_l = 
		    EX_FLD(cmd_desc.out_param, hermonprm_query_fw_st, error_buf_offset_l);
		qfw->error_buf_bar = 
		    EX_FLD(cmd_desc.out_param, hermonprm_query_fw_st, error_buf_bar);
		qfw->error_buf_size =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_fw_st, error_buf_size);
		qfw->fw_pages = 
		    EX_FLD(cmd_desc.out_param, hermonprm_query_fw_st, fw_pages);
		qfw->clear_int_base_offset_h = 
		    EX_FLD(cmd_desc.out_param, hermonprm_query_fw_st,
			   clr_int_base_offset_h);
		qfw->clear_int_base_offset_l = 
		    EX_FLD(cmd_desc.out_param, hermonprm_query_fw_st,
			   clr_int_base_offset_l);
		qfw->clear_int_bar = 
		    EX_FLD(cmd_desc.out_param, hermonprm_query_fw_st, clr_int_bar);
	}

	return rc;
}


/*
 *  cmd_query_adapter
 */
static int cmd_query_adapter(struct query_adapter_st *qa)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = HERMON_CMD_QUERY_ADAPTER;
	cmd_desc.out_trans = TRANS_MAILBOX;
	cmd_desc.out_param = get_outprm_buf();
	cmd_desc.out_param_size = MT_STRUCT_SIZE(hermonprm_query_adapter_st);

	rc = cmd_invoke(&cmd_desc);
	if (!rc) {
		qa->intapin =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_adapter_st,
			   intapin);
	}

	return rc;
}




/*
 *  cmd_map_fa
 */
static int cmd_map_fa(struct map_icm_st *map_fa_p)
{
	int rc;
	command_fields_t cmd_desc;
	unsigned int in_param_size, i;
	unsigned long off;

	if (map_fa_p->num_vpm > MAX_VPM_PER_CALL) {
		return -1;
	}

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = HERMON_CMD_MAP_FA;
	cmd_desc.input_modifier = map_fa_p->num_vpm;
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.in_param = get_inprm_buf();
	in_param_size =
	    MT_STRUCT_SIZE(hermonprm_virtual_physical_mapping_st) *
	    map_fa_p->num_vpm;
	cmd_desc.in_param_size = in_param_size;
	memset(cmd_desc.in_param, 0, in_param_size);

	for (i = 0; i < map_fa_p->num_vpm; ++i) {
		off = (unsigned long)(cmd_desc.in_param) +
		    MT_STRUCT_SIZE(hermonprm_virtual_physical_mapping_st) * i;
		INS_FLD(map_fa_p->vpm_arr[i].va_h, off,
			hermonprm_virtual_physical_mapping_st, va_h);
		INS_FLD(map_fa_p->vpm_arr[i].va_l >> 12, off,
			hermonprm_virtual_physical_mapping_st, va_l);
		INS_FLD(map_fa_p->vpm_arr[i].pa_h, off,
			hermonprm_virtual_physical_mapping_st, pa_h);
		INS_FLD(map_fa_p->vpm_arr[i].pa_l >> 12, off,
			hermonprm_virtual_physical_mapping_st, pa_l);
		INS_FLD(map_fa_p->vpm_arr[i].log2_size, off,
			hermonprm_virtual_physical_mapping_st, log2size);
	}

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_unmap_fa
 */
static int cmd_unmap_fa(void)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = HERMON_CMD_UNMAP_FA;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_run_fw
 */
static int cmd_run_fw(void)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = HERMON_CMD_RUN_FW;
	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_set_icm_size
 */
static int cmd_set_icm_size(unsigned long long icm_size, __u32 *aux_pages_p)
{
	int rc;
	command_fields_t cmd_desc;
	__u32 iprm[2], oprm[2];

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = HERMON_CMD_SET_ICM_SIZE;

	iprm[1] = icm_size & 0xFFFFFFFF;
	iprm[0] = icm_size >> 32;
	cmd_desc.in_trans = TRANS_IMMEDIATE;
	cmd_desc.in_param = iprm;
	cmd_desc.out_trans = TRANS_IMMEDIATE;
	cmd_desc.out_param = oprm;
	rc = cmd_invoke(&cmd_desc);
	if (!rc) {
		if (oprm[0]) {
			/* too many pages required */
			return -1;
		}
		*aux_pages_p = oprm[1];
	}

	return rc;
}


/*
 *  cmd_map_icm_aux
 */
static int cmd_map_icm_aux(struct map_icm_st *map_icm_aux_p)
{
	int rc;
	command_fields_t cmd_desc;
	unsigned int in_param_size, i;
	unsigned long off;

	if (map_icm_aux_p->num_vpm > MAX_VPM_PER_CALL) {
		return -1;
	}

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = HERMON_CMD_MAP_ICM_AUX;
	cmd_desc.input_modifier = map_icm_aux_p->num_vpm;
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.in_param = get_inprm_buf();
	in_param_size =
	    MT_STRUCT_SIZE(hermonprm_virtual_physical_mapping_st) *
	    map_icm_aux_p->num_vpm;
	cmd_desc.in_param_size = in_param_size;
	memset(cmd_desc.in_param, 0, in_param_size);

	for (i = 0; i < map_icm_aux_p->num_vpm; ++i) {
		off = (unsigned long)(cmd_desc.in_param) +
		    MT_STRUCT_SIZE(hermonprm_virtual_physical_mapping_st) * i;
		INS_FLD(map_icm_aux_p->vpm_arr[i].va_h, off,
			hermonprm_virtual_physical_mapping_st, va_h);
		INS_FLD(map_icm_aux_p->vpm_arr[i].va_l >> 12, off,
			hermonprm_virtual_physical_mapping_st, va_l);
		INS_FLD(map_icm_aux_p->vpm_arr[i].pa_h, off,
			hermonprm_virtual_physical_mapping_st, pa_h);
		INS_FLD(map_icm_aux_p->vpm_arr[i].pa_l >> 12, off,
			hermonprm_virtual_physical_mapping_st, pa_l);
		INS_FLD(map_icm_aux_p->vpm_arr[i].log2_size, off,
			hermonprm_virtual_physical_mapping_st, log2size);
	}

	rc = cmd_invoke(&cmd_desc);

	return rc;
}


/*
 *  cmd_unmap_icm_aux
 */
static int cmd_unmap_icm_aux(void)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = HERMON_CMD_UNMAP_ICM_AUX;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}

/*
 *  cmd_map_icm
 */
static int cmd_map_icm(struct map_icm_st *map_icm_p)
{
	int rc;
	command_fields_t cmd_desc;
	unsigned int in_param_size, i;
	unsigned long off;

	if (map_icm_p->num_vpm > MAX_VPM_PER_CALL) {
		return -1;
	}

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = HERMON_CMD_MAP_ICM;
	cmd_desc.input_modifier = map_icm_p->num_vpm;
	cmd_desc.in_trans = TRANS_MAILBOX;
	cmd_desc.in_param = get_inprm_buf();
	in_param_size =
	    MT_STRUCT_SIZE(hermonprm_virtual_physical_mapping_st) *
	    map_icm_p->num_vpm;
	cmd_desc.in_param_size = in_param_size;
	memset(cmd_desc.in_param, 0, in_param_size);

	for (i = 0; i < map_icm_p->num_vpm; ++i) {
		off = (unsigned long)(cmd_desc.in_param) +
		    MT_STRUCT_SIZE(hermonprm_virtual_physical_mapping_st) * i;
		INS_FLD(map_icm_p->vpm_arr[i].va_h, off,
			hermonprm_virtual_physical_mapping_st, va_h);
		INS_FLD(map_icm_p->vpm_arr[i].va_l >> 12, off,
			hermonprm_virtual_physical_mapping_st, va_l);
		INS_FLD(map_icm_p->vpm_arr[i].pa_h, off,
			hermonprm_virtual_physical_mapping_st, pa_h);
		INS_FLD(map_icm_p->vpm_arr[i].pa_l >> 12, off,
			hermonprm_virtual_physical_mapping_st, pa_l);
		INS_FLD(map_icm_p->vpm_arr[i].log2_size, off,
			hermonprm_virtual_physical_mapping_st, log2size);
	}

	rc = cmd_invoke(&cmd_desc);

	return rc;
}



/*
 *  cmd_unmap_icm
 */
static int cmd_unmap_icm(__u64 va, int npages)
{
	int rc;
	command_fields_t cmd_desc;
	__u32 iprm[2];

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = HERMON_CMD_UNMAP_ICM;
	iprm[0] = va >> 32;
	iprm[1] = va & 0xffffffffULL;
	cmd_desc.in_param = iprm;
	cmd_desc.in_trans = TRANS_IMMEDIATE;
	cmd_desc.input_modifier = npages;

	rc = cmd_invoke(&cmd_desc);

	return rc;
}


/*
 *  cmd_query_dev_cap
 */
static int cmd_query_dev_cap(struct dev_cap_st *dev_cap_p)
{
	int rc;
	command_fields_t cmd_desc;

	memset(&cmd_desc, 0, sizeof cmd_desc);

	cmd_desc.opcode = HERMON_CMD_QUERY_DEV_CAP;
	cmd_desc.out_trans = TRANS_MAILBOX;
	cmd_desc.out_param = get_outprm_buf();
	cmd_desc.out_param_size = MT_STRUCT_SIZE(hermonprm_query_dev_cap_st);

	rc = cmd_invoke(&cmd_desc);
	if (!rc) {
		dev_cap_p->log2_rsvd_qps =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   log2_rsvd_qps);
		dev_cap_p->qpc_entry_sz =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   qpc_entry_sz);

		dev_cap_p->log2_rsvd_srqs =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   log2_rsvd_srqs);
		dev_cap_p->srq_entry_sz =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   srq_entry_sz);
		dev_cap_p->log2_rsvd_cqs =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   log2_rsvd_cqs);
		dev_cap_p->cqc_entry_sz =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   cqc_entry_sz);

		dev_cap_p->log2_rsvd_mtts =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   log2_rsvd_mtts);
		dev_cap_p->mtt_entry_sz =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   mtt_entry_sz);

		dev_cap_p->log2_rsvd_mrws =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   log2_rsvd_mrws);
		dev_cap_p->d_mpt_entry_sz = 
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   d_mpt_entry_sz);

		dev_cap_p->c_mpt_entry_sz = 
			    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
				   c_mpt_entry_sz);
                
		dev_cap_p->auxc_entry_size =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
				   aux_entry_sz); 
		dev_cap_p->altc_entry_size =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
				   altc_entry_sz); 
					  
		dev_cap_p->num_rsvd_eqs = 
			EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			       num_rsvd_eqs);
		dev_cap_p->eqc_entry_sz =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   eqc_entry_sz);
	
		dev_cap_p->max_icm_size_l =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   max_icm_size_l);
		dev_cap_p->max_icm_size_h =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   max_icm_size_h);
	
		dev_cap_p->num_rsvd_uars =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   num_rsvd_uars);
		dev_cap_p->uar_sz =
		    EX_FLD(cmd_desc.out_param, hermonprm_query_dev_cap_st,
			   uar_sz);
	}

	return rc;
}
