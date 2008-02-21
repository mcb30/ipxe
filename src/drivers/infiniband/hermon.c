/*
 * Copyright (C) 2007 Michael Brown <mbrown@fensystems.co.uk>.
 *
 * Based in part upon the original driver by Mellanox Technologies
 * Ltd.  Portions may be Copyright (c) Mellanox Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <errno.h>
#include <timer.h>
#include <byteswap.h>
#include <gpxe/pci.h>
#include <gpxe/malloc.h>
#include <gpxe/umalloc.h>
#include <gpxe/iobuf.h>
#include <gpxe/netdevice.h>
#include <gpxe/infiniband.h>
#include <gpxe/ipoib.h>
#include "hermon.h"

/**
 * @file
 *
 * Mellanox Hermon Infiniband HCA
 *
 */

/* Port to use */
#define PXE_IB_PORT 1

/***************************************************************************
 *
 * Queue number allocation
 *
 ***************************************************************************
 */

/**
 * Allocate queue number
 *
 * @v q_inuse		Queue usage bitmask
 * @v max_inuse		Maximum number of in-use queues
 * @ret qn_offset	Free queue number offset, or negative error
 */
static int hermon_alloc_qn_offset ( hermon_bitmask_t *q_inuse,
				    unsigned int max_inuse ) {
	unsigned int qn_offset = 0;
	hermon_bitmask_t mask = 1;

	while ( qn_offset < max_inuse ) {
		if ( ( mask & *q_inuse ) == 0 ) {
			*q_inuse |= mask;
			return qn_offset;
		}
		qn_offset++;
		mask <<= 1;
		if ( ! mask ) {
			mask = 1;
			q_inuse++;
		}
	}
	return -ENFILE;
}

/**
 * Free queue number
 *
 * @v q_inuse		Queue usage bitmask
 * @v qn_offset		Queue number offset
 */
static void hermon_free_qn_offset ( hermon_bitmask_t *q_inuse,
				    int qn_offset ) {
	hermon_bitmask_t mask;

	mask = ( 1 << ( qn_offset % ( 8 * sizeof ( mask ) ) ) );
	q_inuse += ( qn_offset / ( 8 * sizeof ( mask ) ) );
	*q_inuse &= ~mask;
}

/***************************************************************************
 *
 * HCA commands
 *
 ***************************************************************************
 */

/**
 * Wait for Hermon command completion
 *
 * @v hermon		Hermon device
 * @v hcr		HCA command registers
 * @ret rc		Return status code
 */
static int hermon_cmd_wait ( struct hermon *hermon,
			     struct hermonprm_hca_command_register *hcr ) {
	unsigned int wait;

	for ( wait = HERMON_HCR_MAX_WAIT_MS ; wait ; wait-- ) {
		hcr->u.dwords[6] =
			readl ( hermon->config + HERMON_HCR_REG ( 6 ) );
		if ( ( MLX_GET ( hcr, go ) == 0 ) &&
		     ( MLX_GET ( hcr, t ) == hermon->toggle ) )
			return 0;
		mdelay ( 1 );
	}
	return -EBUSY;
}

/**
 * Issue HCA command
 *
 * @v hermon		Hermon device
 * @v command		Command opcode, flags and input/output lengths
 * @v op_mod		Opcode modifier (0 if no modifier applicable)
 * @v in		Input parameters
 * @v in_mod		Input modifier (0 if no modifier applicable)
 * @v out		Output parameters
 * @ret rc		Return status code
 */
static int hermon_cmd ( struct hermon *hermon, unsigned long command,
			unsigned int op_mod, const void *in,
			unsigned int in_mod, void *out ) {
	struct hermonprm_hca_command_register hcr;
	unsigned int opcode = HERMON_HCR_OPCODE ( command );
	size_t in_len = HERMON_HCR_IN_LEN ( command );
	size_t out_len = HERMON_HCR_OUT_LEN ( command );
	void *in_buffer;
	void *out_buffer;
	unsigned int status;
	unsigned int i;
	int rc;

	assert ( in_len <= HERMON_MBOX_SIZE );
	assert ( out_len <= HERMON_MBOX_SIZE );

	DBGC2 ( hermon, "Hermon %p command %02x in %zx%s out %zx%s\n",
		hermon, opcode, in_len,
		( ( command & HERMON_HCR_IN_MBOX ) ? "(mbox)" : "" ), out_len,
		( ( command & HERMON_HCR_OUT_MBOX ) ? "(mbox)" : "" ) );

	/* Check that HCR is free */
	if ( ( rc = hermon_cmd_wait ( hermon, &hcr ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p command interface locked\n",
		       hermon );
		return rc;
	}

	/* Flip HCR toggle */
	hermon->toggle = ( 1 - hermon->toggle );

	/* Prepare HCR */
	memset ( &hcr, 0, sizeof ( hcr ) );
	in_buffer = &hcr.u.dwords[0];
	if ( in_len && ( command & HERMON_HCR_IN_MBOX ) ) {
		in_buffer = hermon->mailbox_in;
		MLX_FILL_1 ( &hcr, 1, in_param_l, virt_to_bus ( in_buffer ) );
	}
	memcpy ( in_buffer, in, in_len );
	MLX_FILL_1 ( &hcr, 2, input_modifier, in_mod );
	out_buffer = &hcr.u.dwords[3];
	if ( out_len && ( command & HERMON_HCR_OUT_MBOX ) ) {
		out_buffer = hermon->mailbox_out;
		MLX_FILL_1 ( &hcr, 4, out_param_l,
			     virt_to_bus ( out_buffer ) );
	}
	MLX_FILL_4 ( &hcr, 6,
		     opcode, opcode,
		     opcode_modifier, op_mod,
		     go, 1,
		     t, hermon->toggle );
	DBGC2_HDA ( hermon, virt_to_phys ( hermon->config + HERMON_HCR_BASE ),
		    &hcr, sizeof ( hcr ) );
	if ( in_len ) {
		DBGC2 ( hermon, "Input:\n" );
		DBGC2_HD ( hermon, in, ( ( in_len < 512 ) ? in_len : 512 ) );
	}


	DBGC ( hermon, "Issuing command:\n" );
	DBGC_HDA ( hermon, virt_to_phys ( hermon->config + HERMON_HCR_BASE ),
		   &hcr, sizeof ( hcr ) );

	/* Issue command */
	for ( i = 0 ; i < ( sizeof ( hcr ) / sizeof ( hcr.u.dwords[0] ) ) ;
	      i++ ) {
		writel ( hcr.u.dwords[i],
			 hermon->config + HERMON_HCR_REG ( i ) );
		barrier();
	}

	/* Wait for command completion */
	if ( ( rc = hermon_cmd_wait ( hermon, &hcr ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p timed out waiting for command:\n",
		       hermon );
		DBGC_HDA ( hermon,
			   virt_to_phys ( hermon->config + HERMON_HCR_BASE ),
			   &hcr, sizeof ( hcr ) );
		return rc;
	}

	/* Check command status */
	status = MLX_GET ( &hcr, status );
	if ( status != 0 ) {
		DBGC ( hermon, "Hermon %p command failed with status %02x:\n",
		       hermon, status );
		DBGC_HDA ( hermon,
			   virt_to_phys ( hermon->config + HERMON_HCR_BASE ),
			   &hcr, sizeof ( hcr ) );
		return -EIO;
	}

	/* Read output parameters, if any */
	hcr.u.dwords[3] = readl ( hermon->config + HERMON_HCR_REG ( 3 ) );
	hcr.u.dwords[4] = readl ( hermon->config + HERMON_HCR_REG ( 4 ) );
	memcpy ( out, out_buffer, out_len );
	if ( out_len ) {
		DBGC2 ( hermon, "Output:\n" );
		DBGC2_HD ( hermon, out,
			   ( ( out_len < 512 ) ? out_len : 512 ) );
	}

	return 0;
}

static inline int
hermon_cmd_query_dev_cap ( struct hermon *hermon,
			   struct hermonprm_query_dev_cap *dev_cap ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_QUERY_DEV_CAP,
						 1, sizeof ( *dev_cap ) ),
			    0, NULL, 0, dev_cap );
}

static inline int
hermon_cmd_query_fw ( struct hermon *hermon, struct hermonprm_query_fw *fw ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_QUERY_FW,
						 1, sizeof ( *fw ) ),
			    0, NULL, 0, fw );
}

static inline int
hermon_cmd_init_hca ( struct hermon *hermon,
		      const struct hermonprm_init_hca *init_hca ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_INIT_HCA,
						1, sizeof ( *init_hca ) ),
			    0, init_hca, 0, NULL );
}

static inline int
hermon_cmd_close_hca ( struct hermon *hermon ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_CLOSE_HCA ),
			    0, NULL, 0, NULL );
}

static inline int
hermon_cmd_init_port ( struct hermon *hermon, unsigned int port,
		       const struct hermonprm_init_port *init_port ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_INIT_PORT,
						1, sizeof ( *init_port ) ),
			    0, init_port, port, NULL );
}

static inline int
hermon_cmd_close_port ( struct hermon *hermon, unsigned int port ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_CLOSE_PORT ),
			    0, NULL, port, NULL );
}

static inline int
hermon_cmd_sw2hw_mpt ( struct hermon *hermon, unsigned int index,
		       const struct hermonprm_mpt *mpt ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_SW2HW_MPT,
						1, sizeof ( *mpt ) ),
			    0, mpt, index, NULL );
}

static inline int
hermon_cmd_sw2hw_eq ( struct hermon *hermon, unsigned int index,
		      const struct hermonprm_eqc *eqc ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_SW2HW_EQ,
						1, sizeof ( *eqc ) ),
			    0, eqc, index, NULL );
}

static inline int
hermon_cmd_hw2sw_eq ( struct hermon *hermon, unsigned int index ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_HW2SW_EQ ),
			    1, NULL, index, NULL );
}

static inline int
hermon_cmd_sw2hw_cq ( struct hermon *hermon, unsigned long cqn,
		      const struct hermonprm_completion_queue_context *cqctx ){
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_SW2HW_CQ,
						1, sizeof ( *cqctx ) ),
			    0, cqctx, cqn, NULL );
}

static inline int
hermon_cmd_hw2sw_cq ( struct hermon *hermon, unsigned long cqn,
		      struct hermonprm_completion_queue_context *cqctx) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_HW2SW_CQ,
						 1, sizeof ( *cqctx ) ),
			    0, NULL, cqn, cqctx );
}

static inline int
hermon_cmd_rst2init_qpee ( struct hermon *hermon, unsigned long qpn,
			const struct hermonprm_qp_ee_state_transitions *ctx ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_RST2INIT_QPEE,
						1, sizeof ( *ctx ) ),
			    0, ctx, qpn, NULL );
}

static inline int
hermon_cmd_init2rtr_qpee ( struct hermon *hermon, unsigned long qpn,
			const struct hermonprm_qp_ee_state_transitions *ctx ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_INIT2RTR_QPEE,
						1, sizeof ( *ctx ) ),
			    0, ctx, qpn, NULL );
}

static inline int
hermon_cmd_rtr2rts_qpee ( struct hermon *hermon, unsigned long qpn,
			const struct hermonprm_qp_ee_state_transitions *ctx ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_RTR2RTS_QPEE,
						1, sizeof ( *ctx ) ),
			    0, ctx, qpn, NULL );
}

static inline int
hermon_cmd_2rst_qpee ( struct hermon *hermon, unsigned long qpn ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_2RST_QPEE ),
			    0x03, NULL, qpn, NULL );
}

static inline int
hermon_cmd_mad_ifc ( struct hermon *hermon, union hermonprm_mad *mad ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_INOUT_CMD ( HERMON_HCR_MAD_IFC,
						   1, sizeof ( *mad ),
						   1, sizeof ( *mad ) ),
			    0x03, mad, PXE_IB_PORT, mad );
}

static inline int
hermon_cmd_read_mgm ( struct hermon *hermon, unsigned int index,
		      struct hermonprm_mgm_entry *mgm ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_READ_MGM,
						 1, sizeof ( *mgm ) ),
			    0, NULL, index, mgm );
}

static inline int
hermon_cmd_write_mgm ( struct hermon *hermon, unsigned int index,
		       const struct hermonprm_mgm_entry *mgm ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_WRITE_MGM,
						1, sizeof ( *mgm ) ),
			    0, mgm, index, NULL );
}

static inline int
hermon_cmd_mgid_hash ( struct hermon *hermon, const struct ib_gid *gid,
		       struct hermonprm_mgm_hash *hash ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_INOUT_CMD ( HERMON_HCR_MGID_HASH,
						   1, sizeof ( *gid ),
						   0, sizeof ( *hash ) ),
			    0, gid, 0, hash );
}

static inline int
hermon_cmd_run_fw ( struct hermon *hermon ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_RUN_FW ),
			    0, NULL, 0, NULL );
}

static inline int
hermon_cmd_unmap_icm ( struct hermon *hermon, unsigned int page_count ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_UNMAP_ICM ),
			    0, NULL, page_count, NULL );
}

static inline int
hermon_cmd_map_icm ( struct hermon *hermon,
		     const struct hermonprm_virtual_physical_mapping *map ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_MAP_ICM,
						1, sizeof ( *map ) ),
			    0, map, 1, NULL );
}

static inline int
hermon_cmd_unmap_icm_aux ( struct hermon *hermon ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_UNMAP_ICM_AUX ),
			    0, NULL, 0, NULL );
}

static inline int
hermon_cmd_map_icm_aux ( struct hermon *hermon,
		       const struct hermonprm_virtual_physical_mapping *map ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_MAP_ICM_AUX,
						1, sizeof ( *map ) ),
			    0, map, 1, NULL );
}

static inline int
hermon_cmd_set_icm_size ( struct hermon *hermon,
			  const struct hermonprm_scalar_parameter *icm_size,
			  struct hermonprm_scalar_parameter *icm_aux_size ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_INOUT_CMD ( HERMON_HCR_SET_ICM_SIZE,
						   0, sizeof ( *icm_size ),
						   0, sizeof (*icm_aux_size) ),
			    0, icm_size, 0, icm_aux_size );
}

static inline int
hermon_cmd_unmap_fa ( struct hermon *hermon ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_UNMAP_FA ),
			    0, NULL, 0, NULL );
}

static inline int
hermon_cmd_map_fa ( struct hermon *hermon,
		    const struct hermonprm_virtual_physical_mapping *map ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_MAP_FA,
						1, sizeof ( *map ) ),
			    0, map, 1, NULL );
}

/***************************************************************************
 *
 * Completion queue operations
 *
 ***************************************************************************
 */

/**
 * Create completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 * @ret rc		Return status code
 */
static int hermon_create_cq ( struct ib_device *ibdev,
			      struct ib_completion_queue *cq ) {
#if 0
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_completion_queue *hermon_cq;
	struct hermonprm_completion_queue_context cqctx;
	struct hermonprm_cq_ci_db_record *ci_db_rec;
	struct hermonprm_cq_arm_db_record *arm_db_rec;
	int cqn_offset;
	unsigned int i;
	int rc;

	/* Find a free completion queue number */
	cqn_offset = hermon_alloc_qn_offset ( hermon->cq_inuse,
					      HERMON_MAX_CQS );
	if ( cqn_offset < 0 ) {
		DBGC ( hermon, "Hermon %p out of completion queues\n",
		       hermon );
		rc = cqn_offset;
		goto err_cqn_offset;
	}
	cq->cqn = ( hermon->limits.reserved_cqs + cqn_offset );

	/* Allocate control structures */
	hermon_cq = zalloc ( sizeof ( *hermon_cq ) );
	if ( ! hermon_cq ) {
		rc = -ENOMEM;
		goto err_hermon_cq;
	}
	hermon_cq->ci_doorbell_idx = hermon_cq_ci_doorbell_idx ( cqn_offset );
	hermon_cq->arm_doorbell_idx = hermon_cq_arm_doorbell_idx( cqn_offset );

	/* Allocate completion queue itself */
	hermon_cq->cqe_size = ( cq->num_cqes * sizeof ( hermon_cq->cqe[0] ) );
	hermon_cq->cqe = malloc_dma ( hermon_cq->cqe_size,
				      sizeof ( hermon_cq->cqe[0] ) );
	if ( ! hermon_cq->cqe ) {
		rc = -ENOMEM;
		goto err_cqe;
	}
	memset ( hermon_cq->cqe, 0, hermon_cq->cqe_size );
	for ( i = 0 ; i < cq->num_cqes ; i++ ) {
		MLX_FILL_1 ( &hermon_cq->cqe[i].normal, 7, owner, 1 );
	}
	barrier();

	/* Initialise doorbell records */
	ci_db_rec = &hermon->db_rec[hermon_cq->ci_doorbell_idx].cq_ci;
	MLX_FILL_1 ( ci_db_rec, 0, counter, 0 );
	MLX_FILL_2 ( ci_db_rec, 1,
		     res, HERMON_UAR_RES_CQ_CI,
		     cq_number, cq->cqn );
	arm_db_rec = &hermon->db_rec[hermon_cq->arm_doorbell_idx].cq_arm;
	MLX_FILL_1 ( arm_db_rec, 0, counter, 0 );
	MLX_FILL_2 ( arm_db_rec, 1,
		     res, HERMON_UAR_RES_CQ_ARM,
		     cq_number, cq->cqn );

	/* Hand queue over to hardware */
	memset ( &cqctx, 0, sizeof ( cqctx ) );
	MLX_FILL_1 ( &cqctx, 0, st, 0xa /* "Event fired" */ );
	MLX_FILL_1 ( &cqctx, 2, start_address_l,
		     virt_to_bus ( hermon_cq->cqe ) );
	MLX_FILL_2 ( &cqctx, 3,
		     usr_page, hermon->limits.reserved_uars,
		     log_cq_size, fls ( cq->num_cqes - 1 ) );
	MLX_FILL_1 ( &cqctx, 5, c_eqn, HERMON_NO_EQ );
	MLX_FILL_1 ( &cqctx, 6, pd, HERMON_GLOBAL_PD );
	MLX_FILL_1 ( &cqctx, 7, l_key, hermon->reserved_lkey );
	MLX_FILL_1 ( &cqctx, 12, cqn, cq->cqn );
	MLX_FILL_1 ( &cqctx, 13,
		     cq_ci_db_record, hermon_cq->ci_doorbell_idx );
	MLX_FILL_1 ( &cqctx, 14,
		     cq_state_db_record, hermon_cq->arm_doorbell_idx );
	if ( ( rc = hermon_cmd_sw2hw_cq ( hermon, cq->cqn, &cqctx ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p SW2HW_CQ failed: %s\n",
		       hermon, strerror ( rc ) );
		goto err_sw2hw_cq;
	}

	DBGC ( hermon, "Hermon %p CQN %#lx ring at [%p,%p)\n",
	       hermon, cq->cqn, hermon_cq->cqe,
	       ( ( ( void * ) hermon_cq->cqe ) + hermon_cq->cqe_size ) );
	cq->dev_priv = hermon_cq;
	return 0;

 err_sw2hw_cq:
	MLX_FILL_1 ( ci_db_rec, 1, res, HERMON_UAR_RES_NONE );
	MLX_FILL_1 ( arm_db_rec, 1, res, HERMON_UAR_RES_NONE );
	free_dma ( hermon_cq->cqe, hermon_cq->cqe_size );
 err_cqe:
	free ( hermon_cq );
 err_hermon_cq:
	hermon_free_qn_offset ( hermon->cq_inuse, cqn_offset );
 err_cqn_offset:
	return rc;
#endif
	return -ENOSYS;
}

/**
 * Destroy completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 */
static void hermon_destroy_cq ( struct ib_device *ibdev,
				struct ib_completion_queue *cq ) {
#if 0
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_completion_queue *hermon_cq = cq->dev_priv;
	struct hermonprm_completion_queue_context cqctx;
	struct hermonprm_cq_ci_db_record *ci_db_rec;
	struct hermonprm_cq_arm_db_record *arm_db_rec;
	int cqn_offset;
	int rc;

	/* Take ownership back from hardware */
	if ( ( rc = hermon_cmd_hw2sw_cq ( hermon, cq->cqn, &cqctx ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p FATAL HW2SW_CQ failed on CQN %#lx: "
		       "%s\n", hermon, cq->cqn, strerror ( rc ) );
		/* Leak memory and return; at least we avoid corruption */
		return;
	}

	/* Clear doorbell records */
	ci_db_rec = &hermon->db_rec[hermon_cq->ci_doorbell_idx].cq_ci;
	arm_db_rec = &hermon->db_rec[hermon_cq->arm_doorbell_idx].cq_arm;
	MLX_FILL_1 ( ci_db_rec, 1, res, HERMON_UAR_RES_NONE );
	MLX_FILL_1 ( arm_db_rec, 1, res, HERMON_UAR_RES_NONE );

	/* Free memory */
	free_dma ( hermon_cq->cqe, hermon_cq->cqe_size );
	free ( hermon_cq );

	/* Mark queue number as free */
	cqn_offset = ( cq->cqn - hermon->limits.reserved_cqs );
	hermon_free_qn_offset ( hermon->cq_inuse, cqn_offset );

	cq->dev_priv = NULL;
#endif
}

/***************************************************************************
 *
 * Queue pair operations
 *
 ***************************************************************************
 */

/**
 * Create send work queue
 *
 * @v hermon_send_wq	Send work queue
 * @v num_wqes		Number of work queue entries
 * @ret rc		Return status code
 */
#if 0
static int
hermon_create_send_wq ( struct hermon_send_work_queue *hermon_send_wq,
			unsigned int num_wqes ) {
	struct hermonprm_ud_send_wqe *wqe;
	struct hermonprm_ud_send_wqe *next_wqe;
	unsigned int wqe_idx_mask;
	unsigned int i;

	/* Allocate work queue */
	hermon_send_wq->wqe_size = ( num_wqes *
				     sizeof ( hermon_send_wq->wqe[0] ) );
	hermon_send_wq->wqe = malloc_dma ( hermon_send_wq->wqe_size,
					   sizeof ( hermon_send_wq->wqe[0] ) );
	if ( ! hermon_send_wq->wqe )
		return -ENOMEM;
	memset ( hermon_send_wq->wqe, 0, hermon_send_wq->wqe_size );

	/* Link work queue entries */
	wqe_idx_mask = ( num_wqes - 1 );
	for ( i = 0 ; i < num_wqes ; i++ ) {
		wqe = &hermon_send_wq->wqe[i].ud;
		next_wqe = &hermon_send_wq->wqe[ ( i + 1 ) & wqe_idx_mask ].ud;
		MLX_FILL_1 ( &wqe->next, 0, nda_31_6,
			     ( virt_to_bus ( next_wqe ) >> 6 ) );
	}

	return 0;
}
#endif

/**
 * Create receive work queue
 *
 * @v hermon_recv_wq	Receive work queue
 * @v num_wqes		Number of work queue entries
 * @ret rc		Return status code
 */
#if 0
static int
hermon_create_recv_wq ( struct hermon_recv_work_queue *hermon_recv_wq,
			unsigned int num_wqes ) {
	struct hermonprm_recv_wqe *wqe;
	struct hermonprm_recv_wqe *next_wqe;
	unsigned int wqe_idx_mask;
	size_t nds;
	unsigned int i;
	unsigned int j;

	/* Allocate work queue */
	hermon_recv_wq->wqe_size = ( num_wqes *
				     sizeof ( hermon_recv_wq->wqe[0] ) );
	hermon_recv_wq->wqe = malloc_dma ( hermon_recv_wq->wqe_size,
					   sizeof ( hermon_recv_wq->wqe[0] ) );
	if ( ! hermon_recv_wq->wqe )
		return -ENOMEM;
	memset ( hermon_recv_wq->wqe, 0, hermon_recv_wq->wqe_size );

	/* Link work queue entries */
	wqe_idx_mask = ( num_wqes - 1 );
	nds = ( ( offsetof ( typeof ( *wqe ), data ) +
		  sizeof ( wqe->data[0] ) ) >> 4 );
	for ( i = 0 ; i < num_wqes ; i++ ) {
		wqe = &hermon_recv_wq->wqe[i].recv;
		next_wqe = &hermon_recv_wq->wqe[( i + 1 ) & wqe_idx_mask].recv;
		MLX_FILL_1 ( &wqe->next, 0, nda_31_6,
			     ( virt_to_bus ( next_wqe ) >> 6 ) );
		MLX_FILL_1 ( &wqe->next, 1, nds, ( sizeof ( *wqe ) / 16 ) );
		for ( j = 0 ; ( ( ( void * ) &wqe->data[j] ) <
				( ( void * ) ( wqe + 1 ) ) ) ; j++ ) {
			MLX_FILL_1 ( &wqe->data[j], 1,
				     l_key, HERMON_INVALID_LKEY );
		}
	}

	return 0;
}
#endif

/**
 * Create queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @ret rc		Return status code
 */
static int hermon_create_qp ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp ) {
#if 0
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_queue_pair *hermon_qp;
	struct hermonprm_qp_ee_state_transitions qpctx;
	struct hermonprm_qp_db_record *send_db_rec;
	struct hermonprm_qp_db_record *recv_db_rec;
	int qpn_offset;
	int rc;

	/* Find a free queue pair number */
	qpn_offset = hermon_alloc_qn_offset ( hermon->qp_inuse,
					      HERMON_MAX_QPS );
	if ( qpn_offset < 0 ) {
		DBGC ( hermon, "Hermon %p out of queue pairs\n", hermon );
		rc = qpn_offset;
		goto err_qpn_offset;
	}
	qp->qpn = ( HERMON_QPN_BASE + hermon->limits.reserved_qps +
		    qpn_offset );

	/* Allocate control structures */
	hermon_qp = zalloc ( sizeof ( *hermon_qp ) );
	if ( ! hermon_qp ) {
		rc = -ENOMEM;
		goto err_hermon_qp;
	}
	hermon_qp->send.doorbell_idx = hermon_send_doorbell_idx ( qpn_offset );
	hermon_qp->recv.doorbell_idx = hermon_recv_doorbell_idx ( qpn_offset );

	/* Create send and receive work queues */
	if ( ( rc = hermon_create_send_wq ( &hermon_qp->send,
					    qp->send.num_wqes ) ) != 0 )
		goto err_create_send_wq;
	if ( ( rc = hermon_create_recv_wq ( &hermon_qp->recv,
					    qp->recv.num_wqes ) ) != 0 )
		goto err_create_recv_wq;

	/* Initialise doorbell records */
	send_db_rec = &hermon->db_rec[hermon_qp->send.doorbell_idx].qp;
	MLX_FILL_1 ( send_db_rec, 0, counter, 0 );
	MLX_FILL_2 ( send_db_rec, 1,
		     res, HERMON_UAR_RES_SQ,
		     qp_number, qp->qpn );
	recv_db_rec = &hermon->db_rec[hermon_qp->recv.doorbell_idx].qp;
	MLX_FILL_1 ( recv_db_rec, 0, counter, 0 );
	MLX_FILL_2 ( recv_db_rec, 1,
		     res, HERMON_UAR_RES_RQ,
		     qp_number, qp->qpn );

	/* Hand queue over to hardware */
	memset ( &qpctx, 0, sizeof ( qpctx ) );
	MLX_FILL_3 ( &qpctx, 2,
		     qpc_eec_data.de, 1,
		     qpc_eec_data.pm_state, 0x03 /* Always 0x03 for UD */,
		     qpc_eec_data.st, HERMON_ST_UD );
	MLX_FILL_6 ( &qpctx, 4,
		     qpc_eec_data.mtu, HERMON_MTU_2048,
		     qpc_eec_data.msg_max, 11 /* 2^11 = 2048 */,
		     qpc_eec_data.log_rq_size, fls ( qp->recv.num_wqes - 1 ),
		     qpc_eec_data.log_rq_stride,
		     ( fls ( sizeof ( hermon_qp->recv.wqe[0] ) - 1 ) - 4 ),
		     qpc_eec_data.log_sq_size, fls ( qp->send.num_wqes - 1 ),
		     qpc_eec_data.log_sq_stride,
		     ( fls ( sizeof ( hermon_qp->send.wqe[0] ) - 1 ) - 4 ) );
	MLX_FILL_1 ( &qpctx, 5,
		     qpc_eec_data.usr_page, hermon->limits.reserved_uars );
	MLX_FILL_1 ( &qpctx, 10, qpc_eec_data.primary_address_path.port_number,
		     PXE_IB_PORT );
	MLX_FILL_1 ( &qpctx, 27, qpc_eec_data.pd, HERMON_GLOBAL_PD );
	MLX_FILL_1 ( &qpctx, 29, qpc_eec_data.wqe_lkey,
		     hermon->reserved_lkey );
	MLX_FILL_1 ( &qpctx, 30, qpc_eec_data.ssc, 1 );
	MLX_FILL_1 ( &qpctx, 33, qpc_eec_data.cqn_snd, qp->send.cq->cqn );
	MLX_FILL_1 ( &qpctx, 34, qpc_eec_data.snd_wqe_base_adr_l,
		     ( virt_to_bus ( hermon_qp->send.wqe ) >> 6 ) );
	MLX_FILL_1 ( &qpctx, 35, qpc_eec_data.snd_db_record_index,
		     hermon_qp->send.doorbell_idx );
	MLX_FILL_1 ( &qpctx, 38, qpc_eec_data.rsc, 1 );
	MLX_FILL_1 ( &qpctx, 41, qpc_eec_data.cqn_rcv, qp->recv.cq->cqn );
	MLX_FILL_1 ( &qpctx, 42, qpc_eec_data.rcv_wqe_base_adr_l,
		     ( virt_to_bus ( hermon_qp->recv.wqe ) >> 6 ) );
	MLX_FILL_1 ( &qpctx, 43, qpc_eec_data.rcv_db_record_index,
		     hermon_qp->recv.doorbell_idx );
	MLX_FILL_1 ( &qpctx, 44, qpc_eec_data.q_key, qp->qkey );
	if ( ( rc = hermon_cmd_rst2init_qpee ( hermon, qp->qpn,
					       &qpctx ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p RST2INIT_QPEE failed: %s\n",
		       hermon, strerror ( rc ) );
		goto err_rst2init_qpee;
	}
	memset ( &qpctx, 0, sizeof ( qpctx ) );
	MLX_FILL_2 ( &qpctx, 4,
		     qpc_eec_data.mtu, HERMON_MTU_2048,
		     qpc_eec_data.msg_max, 11 /* 2^11 = 2048 */ );
	if ( ( rc = hermon_cmd_init2rtr_qpee ( hermon, qp->qpn,
					       &qpctx ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p INIT2RTR_QPEE failed: %s\n",
		       hermon, strerror ( rc ) );
		goto err_init2rtr_qpee;
	}
	memset ( &qpctx, 0, sizeof ( qpctx ) );
	if ( ( rc = hermon_cmd_rtr2rts_qpee ( hermon, qp->qpn,
					      &qpctx ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p RTR2RTS_QPEE failed: %s\n",
		       hermon, strerror ( rc ) );
		goto err_rtr2rts_qpee;
	}

	DBGC ( hermon, "Hermon %p QPN %#lx send ring at [%p,%p)\n",
	       hermon, qp->qpn, hermon_qp->send.wqe,
	       ( ((void *)hermon_qp->send.wqe ) + hermon_qp->send.wqe_size ) );
	DBGC ( hermon, "Hermon %p QPN %#lx receive ring at [%p,%p)\n",
	       hermon, qp->qpn, hermon_qp->recv.wqe,
	       ( ((void *)hermon_qp->recv.wqe ) + hermon_qp->recv.wqe_size ) );
	qp->dev_priv = hermon_qp;
	return 0;

 err_rtr2rts_qpee:
 err_init2rtr_qpee:
	hermon_cmd_2rst_qpee ( hermon, qp->qpn );
 err_rst2init_qpee:
	MLX_FILL_1 ( send_db_rec, 1, res, HERMON_UAR_RES_NONE );
	MLX_FILL_1 ( recv_db_rec, 1, res, HERMON_UAR_RES_NONE );
	free_dma ( hermon_qp->recv.wqe, hermon_qp->recv.wqe_size );
 err_create_recv_wq:
	free_dma ( hermon_qp->send.wqe, hermon_qp->send.wqe_size );
 err_create_send_wq:
	free ( hermon_qp );
 err_hermon_qp:
	hermon_free_qn_offset ( hermon->qp_inuse, qpn_offset );
 err_qpn_offset:
	return rc;
#endif
	return -ENOSYS;
}

/**
 * Destroy queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 */
static void hermon_destroy_qp ( struct ib_device *ibdev,
				struct ib_queue_pair *qp ) {
#if 0
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_queue_pair *hermon_qp = qp->dev_priv;
	struct hermonprm_qp_db_record *send_db_rec;
	struct hermonprm_qp_db_record *recv_db_rec;
	int qpn_offset;
	int rc;

	/* Take ownership back from hardware */
	if ( ( rc = hermon_cmd_2rst_qpee ( hermon, qp->qpn ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p FATAL 2RST_QPEE failed on QPN %#lx: "
		       "%s\n", hermon, qp->qpn, strerror ( rc ) );
		/* Leak memory and return; at least we avoid corruption */
		return;
	}

	/* Clear doorbell records */
	send_db_rec = &hermon->db_rec[hermon_qp->send.doorbell_idx].qp;
	recv_db_rec = &hermon->db_rec[hermon_qp->recv.doorbell_idx].qp;
	MLX_FILL_1 ( send_db_rec, 1, res, HERMON_UAR_RES_NONE );
	MLX_FILL_1 ( recv_db_rec, 1, res, HERMON_UAR_RES_NONE );

	/* Free memory */
	free_dma ( hermon_qp->send.wqe, hermon_qp->send.wqe_size );
	free_dma ( hermon_qp->recv.wqe, hermon_qp->recv.wqe_size );
	free ( hermon_qp );

	/* Mark queue number as free */
	qpn_offset = ( qp->qpn - HERMON_QPN_BASE -
		       hermon->limits.reserved_qps );
	hermon_free_qn_offset ( hermon->qp_inuse, qpn_offset );

	qp->dev_priv = NULL;
#endif
}

/***************************************************************************
 *
 * Work request operations
 *
 ***************************************************************************
 */

/**
 * Ring doorbell register in UAR
 *
 * @v hermon		Hermon device
 * @v db_reg		Doorbell register structure
 * @v offset		Address of doorbell
 */
static void hermon_ring_doorbell ( struct hermon *hermon,
				   union hermonprm_doorbell_register *db_reg,
				   unsigned int offset ) {
#if 0
	DBGC2 ( hermon, "Hermon %p ringing doorbell %08lx:%08lx at %lx\n",
		hermon, db_reg->dword[0], db_reg->dword[1],
		virt_to_phys ( hermon->uar + offset ) );

	barrier();
	writel ( db_reg->dword[0], ( hermon->uar + offset + 0 ) );
	barrier();
	writel ( db_reg->dword[1], ( hermon->uar + offset + 4 ) );
#endif
}

/** GID used for GID-less send work queue entries */
static const struct ib_gid hermon_no_gid = {
	{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0 } }
};

/**
 * Post send work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int hermon_post_send ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp,
			      struct ib_address_vector *av,
			      struct io_buffer *iobuf ) {
#if 0
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_queue_pair *hermon_qp = qp->dev_priv;
	struct ib_work_queue *wq = &qp->send;
	struct hermon_send_work_queue *hermon_send_wq = &hermon_qp->send;
	struct hermonprm_ud_send_wqe *prev_wqe;
	struct hermonprm_ud_send_wqe *wqe;
	struct hermonprm_qp_db_record *qp_db_rec;
	union hermonprm_doorbell_register db_reg;
	const struct ib_gid *gid;
	unsigned int wqe_idx_mask;
	size_t nds;

	/* Allocate work queue entry */
	wqe_idx_mask = ( wq->num_wqes - 1 );
	if ( wq->iobufs[wq->next_idx & wqe_idx_mask] ) {
		DBGC ( hermon, "Hermon %p send queue full", hermon );
		return -ENOBUFS;
	}
	wq->iobufs[wq->next_idx & wqe_idx_mask] = iobuf;
	prev_wqe = &hermon_send_wq->wqe[(wq->next_idx - 1) & wqe_idx_mask].ud;
	wqe = &hermon_send_wq->wqe[wq->next_idx & wqe_idx_mask].ud;

	/* Construct work queue entry */
	MLX_FILL_1 ( &wqe->next, 1, always1, 1 );
	memset ( &wqe->ctrl, 0, sizeof ( wqe->ctrl ) );
	MLX_FILL_1 ( &wqe->ctrl, 0, always1, 1 );
	memset ( &wqe->ud, 0, sizeof ( wqe->ud ) );
	MLX_FILL_2 ( &wqe->ud, 0,
		     ud_address_vector.pd, HERMON_GLOBAL_PD,
		     ud_address_vector.port_number, PXE_IB_PORT );
	MLX_FILL_2 ( &wqe->ud, 1,
		     ud_address_vector.rlid, av->dlid,
		     ud_address_vector.g, av->gid_present );
	MLX_FILL_2 ( &wqe->ud, 2,
		     ud_address_vector.max_stat_rate,
		     ( ( av->rate >= 3 ) ? 0 : 1 ),
		     ud_address_vector.msg, 3 );
	MLX_FILL_1 ( &wqe->ud, 3, ud_address_vector.sl, av->sl );
	gid = ( av->gid_present ? &av->gid : &hermon_no_gid );
	memcpy ( &wqe->ud.u.dwords[4], gid, sizeof ( *gid ) );
	MLX_FILL_1 ( &wqe->ud, 8, destination_qp, av->dest_qp );
	MLX_FILL_1 ( &wqe->ud, 9, q_key, av->qkey );
	MLX_FILL_1 ( &wqe->data[0], 0, byte_count, iob_len ( iobuf ) );
	MLX_FILL_1 ( &wqe->data[0], 1, l_key, hermon->reserved_lkey );
	MLX_FILL_1 ( &wqe->data[0], 3,
		     local_address_l, virt_to_bus ( iobuf->data ) );

	/* Update previous work queue entry's "next" field */
	nds = ( ( offsetof ( typeof ( *wqe ), data ) +
		  sizeof ( wqe->data[0] ) ) >> 4 );
	MLX_SET ( &prev_wqe->next, nopcode, HERMON_OPCODE_SEND );
	MLX_FILL_3 ( &prev_wqe->next, 1,
		     nds, nds,
		     f, 1,
		     always1, 1 );

	/* Update doorbell record */
	barrier();
	qp_db_rec = &hermon->db_rec[hermon_send_wq->doorbell_idx].qp;
	MLX_FILL_1 ( qp_db_rec, 0,
		     counter, ( ( wq->next_idx + 1 ) & 0xffff ) );

	/* Ring doorbell register */
	MLX_FILL_4 ( &db_reg.send, 0,
		     nopcode, HERMON_OPCODE_SEND,
		     f, 1,
		     wqe_counter, ( wq->next_idx & 0xffff ),
		     wqe_cnt, 1 );
	MLX_FILL_2 ( &db_reg.send, 1,
		     nds, nds,
		     qpn, qp->qpn );
	hermon_ring_doorbell ( hermon, &db_reg, HERMON_DB_POST_SND_OFFSET );

	/* Update work queue's index */
	wq->next_idx++;

	return 0;
#endif
	return -ENOSYS;
}

/**
 * Post receive work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int hermon_post_recv ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp,
			      struct io_buffer *iobuf ) {
#if 0
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_queue_pair *hermon_qp = qp->dev_priv;
	struct ib_work_queue *wq = &qp->recv;
	struct hermon_recv_work_queue *hermon_recv_wq = &hermon_qp->recv;
	struct hermonprm_recv_wqe *wqe;
	union hermonprm_doorbell_record *db_rec;
	unsigned int wqe_idx_mask;

	/* Allocate work queue entry */
	wqe_idx_mask = ( wq->num_wqes - 1 );
	if ( wq->iobufs[wq->next_idx & wqe_idx_mask] ) {
		DBGC ( hermon, "Hermon %p receive queue full", hermon );
		return -ENOBUFS;
	}
	wq->iobufs[wq->next_idx & wqe_idx_mask] = iobuf;
	wqe = &hermon_recv_wq->wqe[wq->next_idx & wqe_idx_mask].recv;

	/* Construct work queue entry */
	MLX_FILL_1 ( &wqe->data[0], 0, byte_count, iob_tailroom ( iobuf ) );
	MLX_FILL_1 ( &wqe->data[0], 1, l_key, hermon->reserved_lkey );
	MLX_FILL_1 ( &wqe->data[0], 3,
		     local_address_l, virt_to_bus ( iobuf->data ) );

	/* Update doorbell record */
	barrier();
	db_rec = &hermon->db_rec[hermon_recv_wq->doorbell_idx];
	MLX_FILL_1 ( &db_rec->qp, 0,
		     counter, ( ( wq->next_idx + 1 ) & 0xffff ) );

	/* Update work queue's index */
	wq->next_idx++;

	return 0;
#endif
	return -ENOSYS;
}

/**
 * Handle completion
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 * @v cqe		Hardware completion queue entry
 * @v complete_send	Send completion handler
 * @v complete_recv	Receive completion handler
 * @ret rc		Return status code
 */
#if 0
static int hermon_complete ( struct ib_device *ibdev,
			     struct ib_completion_queue *cq,
			     union hermonprm_completion_entry *cqe,
			     ib_completer_t complete_send,
			     ib_completer_t complete_recv ) {
	struct hermon *hermon = ibdev->dev_priv;
	struct ib_completion completion;
	struct ib_work_queue *wq;
	struct ib_queue_pair *qp;
	struct hermon_queue_pair *hermon_qp;
	struct hermon_send_work_queue *hermon_send_wq;
	struct hermon_recv_work_queue *hermon_recv_wq;
	struct hermonprm_recv_wqe *recv_wqe;
	struct io_buffer *iobuf;
	ib_completer_t complete;
	unsigned int opcode;
	unsigned long qpn;
	int is_send;
	unsigned long wqe_adr;
	unsigned int wqe_idx;
	int rc = 0;

	/* Parse completion */
	memset ( &completion, 0, sizeof ( completion ) );
	qpn = MLX_GET ( &cqe->normal, my_qpn );
	is_send = MLX_GET ( &cqe->normal, s );
	wqe_adr = ( MLX_GET ( &cqe->normal, wqe_adr ) << 6 );
	opcode = MLX_GET ( &cqe->normal, opcode );
	if ( opcode >= HERMON_OPCODE_RECV_ERROR ) {
		/* "s" field is not valid for error opcodes */
		is_send = ( opcode == HERMON_OPCODE_SEND_ERROR );
		completion.syndrome = MLX_GET ( &cqe->error, syndrome );
		DBGC ( hermon, "Hermon %p CPN %lx syndrome %x vendor %lx\n",
		       hermon, cq->cqn, completion.syndrome,
		       MLX_GET ( &cqe->error, vendor_code ) );
		rc = -EIO;
		/* Don't return immediately; propagate error to completer */
	}

	/* Identify work queue */
	wq = ib_find_wq ( cq, qpn, is_send );
	if ( ! wq ) {
		DBGC ( hermon, "Hermon %p CQN %lx unknown %s QPN %lx\n",
		       hermon, cq->cqn, ( is_send ? "send" : "recv" ), qpn );
		return -EIO;
	}
	qp = wq->qp;
	hermon_qp = qp->dev_priv;
	hermon_send_wq = &hermon_qp->send;
	hermon_recv_wq = &hermon_qp->recv;

	/* Identify work queue entry index */
	if ( is_send ) {
		wqe_idx = ( ( wqe_adr - virt_to_bus ( hermon_send_wq->wqe ) ) /
			    sizeof ( hermon_send_wq->wqe[0] ) );
		assert ( wqe_idx < qp->send.num_wqes );
	} else {
		wqe_idx = ( ( wqe_adr - virt_to_bus ( hermon_recv_wq->wqe ) ) /
			    sizeof ( hermon_recv_wq->wqe[0] ) );
		assert ( wqe_idx < qp->recv.num_wqes );
	}

	/* Identify I/O buffer */
	iobuf = wq->iobufs[wqe_idx];
	if ( ! iobuf ) {
		DBGC ( hermon, "Hermon %p CQN %lx QPN %lx empty WQE %x\n",
		       hermon, cq->cqn, qpn, wqe_idx );
		return -EIO;
	}
	wq->iobufs[wqe_idx] = NULL;

	/* Fill in length for received packets */
	if ( ! is_send ) {
		completion.len = MLX_GET ( &cqe->normal, byte_cnt );
		recv_wqe = &hermon_recv_wq->wqe[wqe_idx].recv;
		assert ( MLX_GET ( &recv_wqe->data[0], local_address_l ) ==
			 virt_to_bus ( iobuf->data ) );
		assert ( MLX_GET ( &recv_wqe->data[0], byte_count ) ==
			 iob_tailroom ( iobuf ) );
		MLX_FILL_1 ( &recv_wqe->data[0], 0, byte_count, 0 );
		MLX_FILL_1 ( &recv_wqe->data[0], 1,
			     l_key, HERMON_INVALID_LKEY );
		if ( completion.len > iob_tailroom ( iobuf ) ) {
			DBGC ( hermon, "Hermon %p CQN %lx QPN %lx IDX %x "
			       "overlength received packet length %zd\n",
			       hermon, cq->cqn, qpn, wqe_idx, completion.len );
			return -EIO;
		}
	}

	/* Pass off to caller's completion handler */
	complete = ( is_send ? complete_send : complete_recv );
	complete ( ibdev, qp, &completion, iobuf );

	return rc;
}
#endif

/**
 * Poll completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 * @v complete_send	Send completion handler
 * @v complete_recv	Receive completion handler
 */
static void hermon_poll_cq ( struct ib_device *ibdev,
			     struct ib_completion_queue *cq,
			     ib_completer_t complete_send,
			     ib_completer_t complete_recv ) {
#if 0
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_completion_queue *hermon_cq = cq->dev_priv;
	struct hermonprm_cq_ci_db_record *ci_db_rec;
	union hermonprm_completion_entry *cqe;
	unsigned int cqe_idx_mask;
	int rc;

	while ( 1 ) {
		/* Look for completion entry */
		cqe_idx_mask = ( cq->num_cqes - 1 );
		cqe = &hermon_cq->cqe[cq->next_idx & cqe_idx_mask];
		if ( MLX_GET ( &cqe->normal, owner ) != 0 ) {
			/* Entry still owned by hardware; end of poll */
			break;
		}

		/* Handle completion */
		if ( ( rc = hermon_complete ( ibdev, cq, cqe, complete_send,
					      complete_recv ) ) != 0 ) {
			DBGC ( hermon, "Hermon %p failed to complete: %s\n",
			       hermon, strerror ( rc ) );
			DBGC_HD ( hermon, cqe, sizeof ( *cqe ) );
		}

		/* Return ownership to hardware */
		MLX_FILL_1 ( &cqe->normal, 7, owner, 1 );
		barrier();
		/* Update completion queue's index */
		cq->next_idx++;
		/* Update doorbell record */
		ci_db_rec = &hermon->db_rec[hermon_cq->ci_doorbell_idx].cq_ci;
		MLX_FILL_1 ( ci_db_rec, 0,
			     counter, ( cq->next_idx & 0xffffffffUL ) );
	}
#endif
}

/***************************************************************************
 *
 * Multicast group operations
 *
 ***************************************************************************
 */

/**
 * Attach to multicast group
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v gid		Multicast GID
 * @ret rc		Return status code
 */
static int hermon_mcast_attach ( struct ib_device *ibdev,
				 struct ib_queue_pair *qp,
				 struct ib_gid *gid ) {
	struct hermon *hermon = ibdev->dev_priv;
	struct hermonprm_mgm_hash hash;
	struct hermonprm_mgm_entry mgm;
	unsigned int index;
	int rc;

	/* Generate hash table index */
	if ( ( rc = hermon_cmd_mgid_hash ( hermon, gid, &hash ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not hash GID: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}
	index = MLX_GET ( &hash, hash );

	/* Check for existing hash table entry */
	if ( ( rc = hermon_cmd_read_mgm ( hermon, index, &mgm ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not read MGM %#x: %s\n",
		       hermon, index, strerror ( rc ) );
		return rc;
	}
	if ( MLX_GET ( &mgm, mgmqp_0.qi ) != 0 ) {
		/* FIXME: this implementation allows only a single QP
		 * per multicast group, and doesn't handle hash
		 * collisions.  Sufficient for IPoIB but may need to
		 * be extended in future.
		 */
		DBGC ( hermon, "Hermon %p MGID index %#x already in use\n",
		       hermon, index );
		return -EBUSY;
	}

	/* Update hash table entry */
	MLX_FILL_2 ( &mgm, 8,
		     mgmqp_0.qpn_i, qp->qpn,
		     mgmqp_0.qi, 1 );
	memcpy ( &mgm.u.dwords[4], gid, sizeof ( *gid ) );
	if ( ( rc = hermon_cmd_write_mgm ( hermon, index, &mgm ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not write MGM %#x: %s\n",
		       hermon, index, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Detach from multicast group
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v gid		Multicast GID
 */
static void hermon_mcast_detach ( struct ib_device *ibdev,
				  struct ib_queue_pair *qp __unused,
				  struct ib_gid *gid ) {
	struct hermon *hermon = ibdev->dev_priv;
	struct hermonprm_mgm_hash hash;
	struct hermonprm_mgm_entry mgm;
	unsigned int index;
	int rc;

	/* Generate hash table index */
	if ( ( rc = hermon_cmd_mgid_hash ( hermon, gid, &hash ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not hash GID: %s\n",
		       hermon, strerror ( rc ) );
		return;
	}
	index = MLX_GET ( &hash, hash );

	/* Clear hash table entry */
	memset ( &mgm, 0, sizeof ( mgm ) );
	if ( ( rc = hermon_cmd_write_mgm ( hermon, index, &mgm ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not write MGM %#x: %s\n",
		       hermon, index, strerror ( rc ) );
		return;
	}
}

/** Hermon Infiniband operations */
static struct ib_device_operations hermon_ib_operations = {
	.create_cq	= hermon_create_cq,
	.destroy_cq	= hermon_destroy_cq,
	.create_qp	= hermon_create_qp,
	.destroy_qp	= hermon_destroy_qp,
	.post_send	= hermon_post_send,
	.post_recv	= hermon_post_recv,
	.poll_cq	= hermon_poll_cq,
	.mcast_attach	= hermon_mcast_attach,
	.mcast_detach	= hermon_mcast_detach,
};

/***************************************************************************
 *
 * MAD IFC operations
 *
 ***************************************************************************
 */

static int hermon_mad_ifc ( struct hermon *hermon,
			    union hermonprm_mad *mad ) {
	struct ib_mad_hdr *hdr = &mad->mad.mad_hdr;
	int rc;

	hdr->base_version = IB_MGMT_BASE_VERSION;
	if ( ( rc = hermon_cmd_mad_ifc ( hermon, mad ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not issue MAD IFC: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}
	if ( hdr->status != 0 ) {
		DBGC ( hermon, "Hermon %p MAD IFC status %04x\n",
		       hermon, ntohs ( hdr->status ) );
		return -EIO;
	}
	return 0;
}

static int hermon_get_port_info ( struct hermon *hermon,
				  struct ib_mad_port_info *port_info ) {
	union hermonprm_mad mad;
	struct ib_mad_hdr *hdr = &mad.mad.mad_hdr;
	int rc;

	memset ( &mad, 0, sizeof ( mad ) );
	hdr->mgmt_class = IB_MGMT_CLASS_SUBN_LID_ROUTED;
	hdr->class_version = 1;
	hdr->method = IB_MGMT_METHOD_GET;
	hdr->attr_id = htons ( IB_SMP_ATTR_PORT_INFO );
	hdr->attr_mod = htonl ( PXE_IB_PORT );
	if ( ( rc = hermon_mad_ifc ( hermon, &mad ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not get port info: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}
	memcpy ( port_info, &mad.mad.port_info, sizeof ( *port_info ) );
	return 0;
}

static int hermon_get_guid_info ( struct hermon *hermon,
				  struct ib_mad_guid_info *guid_info ) {
	union hermonprm_mad mad;
	struct ib_mad_hdr *hdr = &mad.mad.mad_hdr;
	int rc;

	memset ( &mad, 0, sizeof ( mad ) );
	hdr->mgmt_class = IB_MGMT_CLASS_SUBN_LID_ROUTED;
	hdr->class_version = 1;
	hdr->method = IB_MGMT_METHOD_GET;
	hdr->attr_id = htons ( IB_SMP_ATTR_GUID_INFO );
	if ( ( rc = hermon_mad_ifc ( hermon, &mad ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not get GUID info: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}
	memcpy ( guid_info, &mad.mad.guid_info, sizeof ( *guid_info ) );
	return 0;
}

static int hermon_get_pkey_table ( struct hermon *hermon,
				   struct ib_mad_pkey_table *pkey_table ) {
	union hermonprm_mad mad;
	struct ib_mad_hdr *hdr = &mad.mad.mad_hdr;
	int rc;

	memset ( &mad, 0, sizeof ( mad ) );
	hdr->mgmt_class = IB_MGMT_CLASS_SUBN_LID_ROUTED;
	hdr->class_version = 1;
	hdr->method = IB_MGMT_METHOD_GET;
	hdr->attr_id = htons ( IB_SMP_ATTR_PKEY_TABLE );
	if ( ( rc = hermon_mad_ifc ( hermon, &mad ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not get pkey table: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}
	memcpy ( pkey_table, &mad.mad.pkey_table, sizeof ( *pkey_table ) );
	return 0;
}

static int hermon_get_port_gid ( struct hermon *hermon,
				 struct ib_gid *port_gid ) {
	union {
		/* This union exists just to save stack space */
		struct ib_mad_port_info port_info;
		struct ib_mad_guid_info guid_info;
	} u;
	int rc;

	/* Port info gives us the first half of the port GID */
	if ( ( rc = hermon_get_port_info ( hermon, &u.port_info ) ) != 0 )
		return rc;
	memcpy ( &port_gid->u.bytes[0], u.port_info.gid_prefix, 8 );

	/* GUID info gives us the second half of the port GID */
	if ( ( rc = hermon_get_guid_info ( hermon, &u.guid_info ) ) != 0 )
		return rc;
	memcpy ( &port_gid->u.bytes[8], u.guid_info.gid_local, 8 );

	return 0;
}

static int hermon_get_sm_lid ( struct hermon *hermon,
			       unsigned long *sm_lid ) {
	struct ib_mad_port_info port_info;
	int rc;

	if ( ( rc = hermon_get_port_info ( hermon, &port_info ) ) != 0 )
		return rc;
	*sm_lid = ntohs ( port_info.mastersm_lid );
	return 0;
}

static int hermon_get_pkey ( struct hermon *hermon, unsigned int *pkey ) {
	struct ib_mad_pkey_table pkey_table;
	int rc;

	if ( ( rc = hermon_get_pkey_table ( hermon, &pkey_table ) ) != 0 )
		return rc;
	*pkey = ntohs ( pkey_table.pkey[0][0] );
	return 0;
}

/**
 * Wait for link up
 *
 * @v hermon		Hermon device
 * @ret rc		Return status code
 *
 * This function shouldn't really exist.  Unfortunately, IB links take
 * a long time to come up, and we can't get various key parameters
 * e.g. our own IPoIB MAC address without information from the subnet
 * manager).  We should eventually make link-up an asynchronous event.
 */
static int hermon_wait_for_link ( struct hermon *hermon ) {
	struct ib_mad_port_info port_info;
	unsigned int retries;
	int rc;

	printf ( "Waiting for Infiniband link-up..." );
	for ( retries = 20 ; retries ; retries-- ) {
		if ( ( rc = hermon_get_port_info ( hermon,
						   &port_info ) ) != 0 )
			continue;
		if ( ( ( port_info.port_state__link_speed_supported ) & 0xf )
		     == 4 ) {
			printf ( "ok\n" );
			return 0;
		}
		printf ( "." );
		sleep ( 1 );
	}
	printf ( "failed\n" );
	return -ENODEV;
};

/**
 * Get MAD parameters
 *
 * @v hermon		Hermon device
 * @ret rc		Return status code
 */
static int hermon_get_mad_params ( struct ib_device *ibdev ) {
	struct hermon *hermon = ibdev->dev_priv;
	int rc;

	/* Get subnet manager LID */
	if ( ( rc = hermon_get_sm_lid ( hermon, &ibdev->sm_lid ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not determine subnet manager "
		       "LID: %s\n", hermon, strerror ( rc ) );
		return rc;
	}

	/* Get port GID */
	if ( ( rc = hermon_get_port_gid ( hermon, &ibdev->port_gid ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not determine port GID: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	/* Get partition key */
	if ( ( rc = hermon_get_pkey ( hermon, &ibdev->pkey ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not determine partition key: "
		       "%s\n", hermon, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/***************************************************************************
 *
 * Firmware control
 *
 ***************************************************************************
 */

/**
 * Start firmware running
 *
 * @v hermon		Hermon device
 * @ret rc		Return status code
 */
static int hermon_start_firmware ( struct hermon *hermon ) {
	struct hermonprm_query_fw fw;
	struct hermonprm_virtual_physical_mapping map_fa;
	unsigned int fw_pages;
	unsigned int log2_fw_pages;
	size_t fw_size;
	physaddr_t fw_base;
	int rc;

	/* Get firmware parameters */
	if ( ( rc = hermon_cmd_query_fw ( hermon, &fw ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not query firmware: %s\n",
		       hermon, strerror ( rc ) );
		goto err_query_fw;
	}
	DBGC ( hermon, "Hermon %p firmware version %ld.%ld.%ld\n", hermon,
	       MLX_GET ( &fw, fw_rev_major ), MLX_GET ( &fw, fw_rev_minor ),
	       MLX_GET ( &fw, fw_rev_subminor ) );
	fw_pages = MLX_GET ( &fw, fw_pages );
	log2_fw_pages = fls ( fw_pages - 1 );
	fw_pages = ( 1 << log2_fw_pages );
	DBGC ( hermon, "Hermon %p requires %d kB for firmware\n",
	       hermon, ( fw_pages * 4 ) );

	/* Allocate firmware pages and map firmware area */
	fw_size = ( fw_pages * 4096 );
	hermon->firmware_area = umalloc ( fw_size );
	if ( ! hermon->firmware_area ) {
		rc = -ENOMEM;
		goto err_alloc_fa;
	}
	fw_base = ( user_to_phys ( hermon->firmware_area, fw_size ) &
		    ~( fw_size - 1 ) );
	DBGC ( hermon, "Hermon %p firmware area at physical [%lx,%lx)\n",
	       hermon, fw_base, ( fw_base + fw_size ) );
	memset ( &map_fa, 0, sizeof ( map_fa ) );
	MLX_FILL_2 ( &map_fa, 3,
		     log2size, log2_fw_pages,
		     pa_l, ( fw_base >> 12 ) );
	if ( ( rc = hermon_cmd_map_fa ( hermon, &map_fa ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not map firmware: %s\n",
		       hermon, strerror ( rc ) );
		goto err_map_fa;
	}

	/* Start firmware */
	if ( ( rc = hermon_cmd_run_fw ( hermon ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not run firmware: %s\n",
		       hermon, strerror ( rc ) );
		goto err_run_fw;
	}

	DBGC ( hermon, "Hermon %p firmware started\n", hermon );
	return 0;

 err_run_fw:
	hermon_cmd_unmap_fa ( hermon );
 err_map_fa:
	ufree ( hermon->firmware_area );
	hermon->firmware_area = UNULL;
 err_alloc_fa:
 err_query_fw:
	return rc;
}

/**
 * Stop firmware running
 *
 * @v hermon		Hermon device
 */
static void hermon_stop_firmware ( struct hermon *hermon ) {
	int rc;

	if ( ( rc = hermon_cmd_unmap_fa ( hermon ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p FATAL could not stop firmware: %s\n",
		       hermon, strerror ( rc ) );
		/* Leak memory and return; at least we avoid corruption */
		return;
	}
	ufree ( hermon->firmware_area );
	hermon->firmware_area = UNULL;
}

/***************************************************************************
 *
 * Infinihost Context Memory management
 *
 ***************************************************************************
 */

/**
 * Get device limits
 *
 * @v hermon		Hermon device
 * @ret rc		Return status code
 */
static int hermon_get_limits ( struct hermon *hermon ) {
	struct hermonprm_query_dev_cap dev_cap;
	int rc;

	if ( ( rc = hermon_cmd_query_dev_cap ( hermon, &dev_cap ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not get device limits: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	hermon->limits.reserved_qps =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_qps ) );
	hermon->limits.qpc_entry_size = MLX_GET ( &dev_cap, qpc_entry_sz );
	//hermon->limits.eqpc_entry_size = MLX_GET ( &dev_cap, eqpc_entry_sz );
	hermon->limits.reserved_srqs =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_srqs ) );
	hermon->limits.srqc_entry_size = MLX_GET ( &dev_cap, srq_entry_sz );
	//hermon->limits.reserved_ees =
	//	( 1 << MLX_GET ( &dev_cap, log2_rsvd_ees ) );
	//hermon->limits.eec_entry_size = MLX_GET ( &dev_cap, eec_entry_sz );
	//hermon->limits.eeec_entry_size = MLX_GET ( &dev_cap, eeec_entry_sz );
	hermon->limits.reserved_cqs =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_cqs ) );
	hermon->limits.cqc_entry_size = MLX_GET ( &dev_cap, cqc_entry_sz );
	hermon->limits.reserved_mtts =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_mtts ) );
	hermon->limits.mtt_entry_size = MLX_GET ( &dev_cap, mtt_entry_sz );
	hermon->limits.reserved_mrws =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_mrws ) );
	//hermon->limits.mpt_entry_size = MLX_GET ( &dev_cap, mpt_entry_sz );
	//hermon->limits.reserved_rdbs =
	//	( 1 << MLX_GET ( &dev_cap, log2_rsvd_rdbs ) );
	hermon->limits.eqc_entry_size = MLX_GET ( &dev_cap, eqc_entry_sz );
	hermon->limits.reserved_uars = MLX_GET ( &dev_cap, num_rsvd_uars );

	return 0;
}

/**
 * Get ICM usage
 *
 * @v log_num_entries	Log2 of the number of entries
 * @v entry_size	Entry size
 * @ret usage		Usage size in ICM
 */
static size_t icm_usage ( unsigned int log_num_entries, size_t entry_size ) {
	size_t usage;

	usage = ( ( 1 << log_num_entries ) * entry_size );
	usage = ( ( usage + 4095 ) & ~4095 );
	return usage;
}

/**
 * Allocate ICM
 *
 * @v hermon		Hermon device
 * @v init_hca		INIT_HCA structure to fill in
 * @ret rc		Return status code
 */
static int hermon_alloc_icm ( struct hermon *hermon,
			      struct hermonprm_init_hca *init_hca ) {
#if 0
	struct hermonprm_scalar_parameter icm_size;
	struct hermonprm_scalar_parameter icm_aux_size;
	struct hermonprm_virtual_physical_mapping map_icm_aux;
	struct hermonprm_virtual_physical_mapping map_icm;
	union hermonprm_doorbell_record *db_rec;
	size_t icm_offset = 0;
	unsigned int log_num_qps, log_num_srqs, log_num_ees, log_num_cqs;
	unsigned int log_num_mtts, log_num_mpts, log_num_rdbs, log_num_eqs;
	int rc;

	icm_offset = ( ( hermon->limits.reserved_uars + 1 ) << 12 );

	/* Queue pair contexts */
	log_num_qps = fls ( hermon->limits.reserved_qps + HERMON_MAX_QPS - 1 );
	MLX_FILL_2 ( init_hca, 13,
		     qpc_eec_cqc_eqc_rdb_parameters.qpc_base_addr_l,
		     ( icm_offset >> 7 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_qp,
		     log_num_qps );
	DBGC ( hermon, "Hermon %p ICM QPC base = %zx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_qps, hermon->limits.qpc_entry_size );

	/* Extended queue pair contexts */
	MLX_FILL_1 ( init_hca, 25,
		     qpc_eec_cqc_eqc_rdb_parameters.eqpc_base_addr_l,
		     icm_offset );
	DBGC ( hermon, "Hermon %p ICM EQPC base = %zx\n", hermon, icm_offset );
	//arbel	icm_offset += icm_usage ( log_num_qps,
	//arbel				  hermon->limits.eqpc_entry_size );
	icm_offset += icm_usage ( log_num_qps,
				  hermon->limits.qpc_entry_size );

	/* Shared receive queue contexts */
	log_num_srqs = fls ( hermon->limits.reserved_srqs - 1 );
	MLX_FILL_2 ( init_hca, 19,
		     qpc_eec_cqc_eqc_rdb_parameters.srqc_base_addr_l,
		     ( icm_offset >> 5 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_srq,
		     log_num_srqs );
	DBGC ( hermon, "Hermon %p ICM SRQC base = %zx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_srqs,
				  hermon->limits.srqc_entry_size );

	/* End-to-end contexts */
	log_num_ees = fls ( hermon->limits.reserved_ees - 1 );
	MLX_FILL_2 ( init_hca, 17,
		     qpc_eec_cqc_eqc_rdb_parameters.eec_base_addr_l,
		     ( icm_offset >> 7 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_ee,
		     log_num_ees );
	DBGC ( hermon, "Hermon %p ICM EEC base = %zx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_ees, hermon->limits.eec_entry_size );

	/* Extended end-to-end contexts */
	MLX_FILL_1 ( init_hca, 29,
		     qpc_eec_cqc_eqc_rdb_parameters.eeec_base_addr_l,
		     icm_offset );
	DBGC ( hermon, "Hermon %p ICM EEEC base = %zx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_ees,
				  hermon->limits.eeec_entry_size );

	/* Completion queue contexts */
	log_num_cqs = fls ( hermon->limits.reserved_cqs + HERMON_MAX_CQS - 1 );
	MLX_FILL_2 ( init_hca, 21,
		     qpc_eec_cqc_eqc_rdb_parameters.cqc_base_addr_l,
		     ( icm_offset >> 6 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_cq,
		     log_num_cqs );
	DBGC ( hermon, "Hermon %p ICM CQC base = %zx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_cqs, hermon->limits.cqc_entry_size );

	/* Memory translation table */
	log_num_mtts = fls ( hermon->limits.reserved_mtts - 1 );
	MLX_FILL_1 ( init_hca, 65,
		     tpt_parameters.mtt_base_addr_l, icm_offset );
	DBGC ( hermon, "Hermon %p ICM MTT base = %zx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_mtts,
				  hermon->limits.mtt_entry_size );

	/* Memory protection table */
	log_num_mpts = fls ( hermon->limits.reserved_mrws + 1 - 1 );
	MLX_FILL_1 ( init_hca, 61,
		     tpt_parameters.mpt_base_adr_l, icm_offset );
	MLX_FILL_1 ( init_hca, 62,
		     tpt_parameters.log_mpt_sz, log_num_mpts );
	DBGC ( hermon, "Hermon %p ICM MTT base = %zx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_mpts,
				  hermon->limits.mpt_entry_size );

	/* RDMA something or other */
	log_num_rdbs = fls ( hermon->limits.reserved_rdbs - 1 );
	MLX_FILL_1 ( init_hca, 37,
		     qpc_eec_cqc_eqc_rdb_parameters.rdb_base_addr_l,
		     icm_offset );
	DBGC ( hermon, "Hermon %p ICM RDB base = %zx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_rdbs, 32 );

	/* Event queue contexts */
	log_num_eqs = 6;
	MLX_FILL_2 ( init_hca, 33,
		     qpc_eec_cqc_eqc_rdb_parameters.eqc_base_addr_l,
		     ( icm_offset >> 6 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_eq,
		     log_num_eqs );
	DBGC ( hermon, "Hermon %p ICM EQ base = %zx\n", hermon, icm_offset );
	icm_offset += ( ( 1 << log_num_eqs ) * hermon->limits.eqc_entry_size );

	/* Multicast table */
	MLX_FILL_1 ( init_hca, 49,
		     multicast_parameters.mc_base_addr_l, icm_offset );
	MLX_FILL_1 ( init_hca, 52,
		     multicast_parameters.log_mc_table_entry_sz,
		     fls ( sizeof ( struct hermonprm_mgm_entry ) - 1 ) );
	MLX_FILL_1 ( init_hca, 53,
		     multicast_parameters.mc_table_hash_sz, 8 );
	MLX_FILL_1 ( init_hca, 54,
		     multicast_parameters.log_mc_table_sz, 3 );
	DBGC ( hermon, "Hermon %p ICM MC base = %zx\n", hermon, icm_offset );
	icm_offset += ( 8 * sizeof ( struct hermonprm_mgm_entry ) );

	hermon->icm_len = icm_offset;
	hermon->icm_len = ( ( hermon->icm_len + 4095 ) & ~4095 );

	/* Get ICM auxiliary area size */
	memset ( &icm_size, 0, sizeof ( icm_size ) );
	MLX_FILL_1 ( &icm_size, 1, value, hermon->icm_len );
	if ( ( rc = hermon_cmd_set_icm_size ( hermon, &icm_size,
					      &icm_aux_size ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not set ICM size: %s\n",
		       hermon, strerror ( rc ) );
		goto err_set_icm_size;
	}
	hermon->icm_aux_len = ( MLX_GET ( &icm_aux_size, value ) * 4096 );

	/* Allocate ICM data and auxiliary area */
	DBGC ( hermon, "Hermon %p requires %zd kB ICM and %zd kB AUX ICM\n",
	       hermon, ( hermon->icm_len / 1024 ),
	       ( hermon->icm_aux_len / 1024 ) );
	hermon->icm = umalloc ( hermon->icm_len + hermon->icm_aux_len );
	if ( ! hermon->icm ) {
		rc = -ENOMEM;
		goto err_alloc;
	}

	/* Map ICM auxiliary area */
	memset ( &map_icm_aux, 0, sizeof ( map_icm_aux ) );
	MLX_FILL_2 ( &map_icm_aux, 3,
		     log2size, fls ( ( hermon->icm_aux_len / 4096 ) - 1 ),
		     pa_l,
		     ( user_to_phys ( hermon->icm, hermon->icm_len ) >> 12 ) );
	if ( ( rc = hermon_cmd_map_icm_aux ( hermon, &map_icm_aux ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not map AUX ICM: %s\n",
		       hermon, strerror ( rc ) );
		goto err_map_icm_aux;
	}

	/* MAP ICM area */
	memset ( &map_icm, 0, sizeof ( map_icm ) );
	MLX_FILL_2 ( &map_icm, 3,
		     log2size, fls ( ( hermon->icm_len / 4096 ) - 1 ),
		     pa_l, ( user_to_phys ( hermon->icm, 0 ) >> 12 ) );
	if ( ( rc = hermon_cmd_map_icm ( hermon, &map_icm ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not map ICM: %s\n",
		       hermon, strerror ( rc ) );
		goto err_map_icm;
	}

	/* Initialise UAR context */
	hermon->db_rec = phys_to_virt ( user_to_phys ( hermon->icm, 0 ) +
					( hermon->limits.reserved_uars *
					  HERMON_PAGE_SIZE ) );
	memset ( hermon->db_rec, 0, HERMON_PAGE_SIZE );
	db_rec = &hermon->db_rec[HERMON_GROUP_SEPARATOR_DOORBELL];
	MLX_FILL_1 ( &db_rec->qp, 1, res, HERMON_UAR_RES_GROUP_SEP );

	return 0;

	hermon_cmd_unmap_icm ( hermon, ( hermon->icm_len / 4096 ) );
 err_map_icm:
	hermon_cmd_unmap_icm_aux ( hermon );
 err_map_icm_aux:
	ufree ( hermon->icm );
	hermon->icm = UNULL;
 err_alloc:
 err_set_icm_size:
	return rc;
#endif
	return -ENOSYS;
}

/**
 * Free ICM
 *
 * @v hermon		Hermon device
 */
static void hermon_free_icm ( struct hermon *hermon ) {
	hermon_cmd_unmap_icm ( hermon, ( hermon->icm_len / 4096 ) );
	hermon_cmd_unmap_icm_aux ( hermon );
	ufree ( hermon->icm );
	hermon->icm = UNULL;
}

/***************************************************************************
 *
 * Infiniband link-layer operations
 *
 ***************************************************************************
 */

/**
 * Initialise Infiniband link
 *
 * @v hermon		Hermon device
 * @ret rc		Return status code
 */
static int hermon_init_port ( struct hermon *hermon ) {
	struct hermonprm_init_port init_port;
	int rc;

	memset ( &init_port, 0, sizeof ( init_port ) );
	MLX_FILL_2 ( &init_port, 0,
		     port_width_cap, 3,
		     vl_cap, 1 );
	MLX_FILL_2 ( &init_port, 1,
		     mtu, 2048,
		     max_gid, 1 );
	MLX_FILL_1 ( &init_port, 2, max_pkey, 64 );
	if ( ( rc = hermon_cmd_init_port ( hermon, PXE_IB_PORT,
					   &init_port ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not intialise port: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Close Infiniband link
 *
 * @v hermon		Hermon device
 */
static void hermon_close_port ( struct hermon *hermon ) {
	int rc;

	if ( ( rc = hermon_cmd_close_port ( hermon, PXE_IB_PORT ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not close port: %s\n",
		       hermon, strerror ( rc ) );
		/* Nothing we can do about this */
	}
}

/***************************************************************************
 *
 * PCI interface
 *
 ***************************************************************************
 */

/**
 * Set up memory protection table
 *
 * @v hermon		Hermon device
 * @ret rc		Return status code
 */
static int hermon_setup_mpt ( struct hermon *hermon ) {
	struct hermonprm_mpt mpt;
	uint32_t key;
	int rc;

	/* Derive key */
	key = ( hermon->limits.reserved_mrws | HERMON_MKEY_PREFIX );
	hermon->reserved_lkey = ( ( key << 8 ) | ( key >> 24 ) );

	/* Initialise memory protection table */
	memset ( &mpt, 0, sizeof ( mpt ) );
	MLX_FILL_4 ( &mpt, 0,
		     r_w, 1,
		     pa, 1,
		     lr, 1,
		     lw, 1 );
	MLX_FILL_1 ( &mpt, 2, mem_key, key );
	MLX_FILL_1 ( &mpt, 3, pd, HERMON_GLOBAL_PD );
	MLX_FILL_1 ( &mpt, 10, len64, 1 );
	if ( ( rc = hermon_cmd_sw2hw_mpt ( hermon,
					   hermon->limits.reserved_mrws,
					   &mpt ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not set up MPT: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Probe PCI device
 *
 * @v pci		PCI device
 * @v id		PCI ID
 * @ret rc		Return status code
 */
static int hermon_probe ( struct pci_device *pci,
			  const struct pci_device_id *id __unused ) {
	struct ib_device *ibdev;
	struct hermon *hermon;
	struct hermonprm_init_hca init_hca;
	int rc;

	/* Allocate Infiniband device */
	ibdev = alloc_ibdev ( sizeof ( *hermon ) );
	if ( ! ibdev ) {
		rc = -ENOMEM;
		goto err_ibdev;
	}
	ibdev->op = &hermon_ib_operations;
	pci_set_drvdata ( pci, ibdev );
	ibdev->dev = &pci->dev;
	hermon = ibdev->dev_priv;
	memset ( hermon, 0, sizeof ( *hermon ) );

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Get PCI BARs */
	hermon->config = ioremap ( pci_bar_start ( pci, HERMON_PCI_CONFIG_BAR),
				   HERMON_PCI_CONFIG_BAR_SIZE );
	DBG ( "Config at %p (phys %08lx)\n", hermon->config,
	      virt_to_phys ( hermon->config ) );
	hermon->uar = ioremap ( ( pci_bar_start ( pci, HERMON_PCI_UAR_BAR ) +
				  HERMON_PCI_UAR_IDX * HERMON_PCI_UAR_SIZE ),
				HERMON_PCI_UAR_SIZE );

	/* Allocate space for mailboxes */
	hermon->mailbox_in = malloc_dma ( HERMON_MBOX_SIZE,
					  HERMON_MBOX_ALIGN );
	if ( ! hermon->mailbox_in ) {
		rc = -ENOMEM;
		goto err_mailbox_in;
	}
	hermon->mailbox_out = malloc_dma ( HERMON_MBOX_SIZE,
					   HERMON_MBOX_ALIGN );
	if ( ! hermon->mailbox_out ) {
		rc = -ENOMEM;
		goto err_mailbox_out;
	}

	/* Start firmware */
	if ( ( rc = hermon_start_firmware ( hermon ) ) != 0 )
		goto err_start_firmware;

	/* Get device limits */
	if ( ( rc = hermon_get_limits ( hermon ) ) != 0 )
		goto err_get_limits;

	/* Allocate ICM */
	memset ( &init_hca, 0, sizeof ( init_hca ) );
	if ( ( rc = hermon_alloc_icm ( hermon, &init_hca ) ) != 0 )
		goto err_alloc_icm;

	/* Initialise HCA */
	MLX_FILL_1 ( &init_hca, 74, uar_parameters.log_max_uars, 1 );
	if ( ( rc = hermon_cmd_init_hca ( hermon, &init_hca ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not initialise HCA: %s\n",
		       hermon, strerror ( rc ) );
		goto err_init_hca;
	}

	/* Set up memory protection */
	if ( ( rc = hermon_setup_mpt ( hermon ) ) != 0 )
		goto err_setup_mpt;

	/* Bring up IB layer */
	if ( ( rc = hermon_init_port ( hermon ) ) != 0 )
		goto err_init_port;

	/* Wait for link */
	if ( ( rc = hermon_wait_for_link ( hermon ) ) != 0 )
		goto err_wait_for_link;

	/* Get MAD parameters */
	if ( ( rc = hermon_get_mad_params ( ibdev ) ) != 0 )
		goto err_get_mad_params;

	DBGC ( hermon, "Hermon %p port GID is %08lx:%08lx:%08lx:%08lx\n",
	       hermon, htonl ( ibdev->port_gid.u.dwords[0] ),
	       htonl ( ibdev->port_gid.u.dwords[1] ),
	       htonl ( ibdev->port_gid.u.dwords[2] ),
	       htonl ( ibdev->port_gid.u.dwords[3] ) );

	/* Add IPoIB device */
	if ( ( rc = ipoib_probe ( ibdev ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not add IPoIB device: %s\n",
		       hermon, strerror ( rc ) );
		goto err_ipoib_probe;
	}

	return 0;

 err_ipoib_probe:
 err_get_mad_params:
 err_wait_for_link:
	hermon_close_port ( hermon );
 err_init_port:
 err_setup_mpt:
	hermon_cmd_close_hca ( hermon );
 err_init_hca:
	hermon_free_icm ( hermon );
 err_alloc_icm:
 err_get_limits:
	hermon_stop_firmware ( hermon );
 err_start_firmware:
	free_dma ( hermon->mailbox_out, HERMON_MBOX_SIZE );
 err_mailbox_out:
	free_dma ( hermon->mailbox_in, HERMON_MBOX_SIZE );
 err_mailbox_in:
	free_ibdev ( ibdev );
 err_ibdev:
	return rc;
}

/**
 * Remove PCI device
 *
 * @v pci		PCI device
 */
static void hermon_remove ( struct pci_device *pci ) {
	struct ib_device *ibdev = pci_get_drvdata ( pci );
	struct hermon *hermon = ibdev->dev_priv;

	ipoib_remove ( ibdev );
	hermon_close_port ( hermon );
	hermon_cmd_close_hca ( hermon );
	hermon_free_icm ( hermon );
	hermon_stop_firmware ( hermon );
	hermon_stop_firmware ( hermon );
	free_dma ( hermon->mailbox_out, HERMON_MBOX_SIZE );
	free_dma ( hermon->mailbox_in, HERMON_MBOX_SIZE );
	free_ibdev ( ibdev );
}

static struct pci_device_id hermon_nics[] = {
	PCI_ROM ( 0x15b3, 0x6340, "mt25408", "MT25408 HCA driver" ),
	PCI_ROM ( 0x15b3, 0x634a, "mt25418", "MT25418 HCA driver" ),
};

struct pci_driver hermon_driver __pci_driver = {
	.ids = hermon_nics,
	.id_count = ( sizeof ( hermon_nics ) / sizeof ( hermon_nics[0] ) ),
	.probe = hermon_probe,
	.remove = hermon_remove,
};
