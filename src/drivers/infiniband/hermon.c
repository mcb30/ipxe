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
 * Allocate offsets within usage bitmask
 *
 * @v bits		Usage bitmask
 * @v bits_len		Length of usage bitmask
 * @v num_bits		Number of contiguous bits to allocate within bitmask
 * @ret bit		First free bit within bitmask, or negative error
 */
static int hermon_bitmask_alloc ( hermon_bitmask_t *bits,
				  unsigned int bits_len,
				  unsigned int num_bits ) {
	unsigned int bit = 0;
	hermon_bitmask_t mask = 1;
	unsigned int found = 0;

	/* Search bits for num_bits contiguous free bits */
	while ( bit < bits_len ) {
		if ( ( mask & *bits ) == 0 ) {
			if ( ++found == num_bits )
				goto found;
		} else {
			found = 0;
		}
		bit++;
		mask = ( mask << 1 ) | ( mask >> ( 8 * sizeof ( mask ) - 1 ) );
		if ( mask == 1 )
			bits++;
	}
	return -ENFILE;

 found:
	/* Mark bits as in-use */
	do {
		*bits |= mask;
		if ( mask == 1 )
			bits--;
		mask = ( mask >> 1 ) | ( mask << ( 8 * sizeof ( mask ) - 1 ) );
	} while ( --found );

	return ( bit - num_bits + 1 );
}

/**
 * Free offsets within usage bitmask
 *
 * @v bits		Usage bitmask
 * @v bit		Starting bit within bitmask
 * @v num_bits		Number of contiguous bits to free within bitmask
 */
static void hermon_bitmask_free ( hermon_bitmask_t *bits,
				  int bit, unsigned int num_bits ) {
	hermon_bitmask_t mask;

	for ( ; num_bits ; bit++, num_bits-- ) {
		mask = ( 1 << ( bit % ( 8 * sizeof ( mask ) ) ) );
		bits[ ( bit / ( 8 * sizeof ( mask ) ) ) ] &= ~mask;
	}
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
	DBGC ( hermon, "Hermon %p issuing command:\n", hermon );
	DBGC_HDA ( hermon, virt_to_phys ( hermon->config + HERMON_HCR_BASE ),
		   &hcr, sizeof ( hcr ) );
	if ( in_len && ( command & HERMON_HCR_IN_MBOX ) ) {
		DBGC2 ( hermon, "Input mailbox:\n" );
		DBGC2_HDA ( hermon, virt_to_phys ( in_buffer ), in_buffer,
			    ( ( in_len < 512 ) ? in_len : 512 ) );
	}

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
		DBGC2 ( hermon, "Output%s:\n",
			( command & HERMON_HCR_OUT_MBOX ) ? " mailbox" : "" );
		DBGC2_HDA ( hermon, virt_to_phys ( out_buffer ), out_buffer,
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
hermon_cmd_write_mtt ( struct hermon *hermon,
		       const struct hermonprm_write_mtt *write_mtt ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_WRITE_MTT,
						1, sizeof ( *write_mtt ) ),
			    0, write_mtt, 1, NULL );
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
hermon_cmd_rst2init_qp ( struct hermon *hermon, unsigned long qpn,
			 const struct hermonprm_qp_ee_state_transitions *ctx ){
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_RST2INIT_QP,
						1, sizeof ( *ctx ) ),
			    0, ctx, qpn, NULL );
}

static inline int
hermon_cmd_init2rtr_qp ( struct hermon *hermon, unsigned long qpn,
			 const struct hermonprm_qp_ee_state_transitions *ctx ){
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_INIT2RTR_QP,
						1, sizeof ( *ctx ) ),
			    0, ctx, qpn, NULL );
}

static inline int
hermon_cmd_rtr2rts_qp ( struct hermon *hermon, unsigned long qpn,
			const struct hermonprm_qp_ee_state_transitions *ctx ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_RTR2RTS_QP,
						1, sizeof ( *ctx ) ),
			    0, ctx, qpn, NULL );
}

static inline int
hermon_cmd_2rst_qp ( struct hermon *hermon, unsigned long qpn ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_VOID_CMD ( HERMON_HCR_2RST_QP ),
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
hermon_cmd_read_mcg ( struct hermon *hermon, unsigned int index,
		      struct hermonprm_mcg_entry *mcg ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_OUT_CMD ( HERMON_HCR_READ_MCG,
						 1, sizeof ( *mcg ) ),
			    0, NULL, index, mcg );
}

static inline int
hermon_cmd_write_mcg ( struct hermon *hermon, unsigned int index,
		       const struct hermonprm_mcg_entry *mcg ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_WRITE_MCG,
						1, sizeof ( *mcg ) ),
			    0, mcg, index, NULL );
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
hermon_cmd_unmap_icm ( struct hermon *hermon, unsigned int page_count,
		       const struct hermonprm_scalar_parameter *offset ) {
	return hermon_cmd ( hermon,
			    HERMON_HCR_IN_CMD ( HERMON_HCR_UNMAP_ICM,
						0, sizeof ( *offset ) ),
			    0, offset, page_count, NULL );
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
 * Memory translation table operations
 *
 ***************************************************************************
 */

/**
 * Allocate MTT entries
 *
 * @v hermon		Hermon device
 * @v memory		Memory to map into MTT
 * @v len		Length of memory to map
 * @v mtt		MTT descriptor to fill in
 * @ret rc		Return status code
 */
static int hermon_alloc_mtt ( struct hermon *hermon,
			      const void *memory, size_t len,
			      struct hermon_mtt *mtt ) {
	struct hermonprm_write_mtt write_mtt;
	physaddr_t start;
	unsigned int page_offset;
	unsigned int num_pages;
	int mtt_offset;
	unsigned int mtt_base_addr;
	unsigned int i;
	int rc;

	/* Find available MTT entries */
	start = virt_to_phys ( memory );
	page_offset = ( start & ( HERMON_PAGE_SIZE - 1 ) );
	start -= page_offset;
	len += page_offset;
	num_pages = ( ( len + HERMON_PAGE_SIZE - 1 ) / HERMON_PAGE_SIZE );
	mtt_offset = hermon_bitmask_alloc ( hermon->mtt_inuse, HERMON_MAX_MTTS,
					    num_pages );
	if ( mtt_offset < 0 ) {
		DBGC ( hermon, "Hermon %p could not allocate %d MTT entries\n",
		       hermon, num_pages );
		rc = mtt_offset;
		goto err_mtt_offset;
	}
	mtt_base_addr = ( ( hermon->cap.reserved_mtts + mtt_offset ) *
			  hermon->cap.mtt_entry_size );

	/* Fill in MTT structure */
	mtt->mtt_offset = mtt_offset;
	mtt->num_pages = num_pages;
	mtt->mtt_base_addr = mtt_base_addr;
	mtt->page_offset = page_offset;

	/* Construct and issue WRITE_MTT commands */
	for ( i = 0 ; i < num_pages ; i++ ) {
		memset ( &write_mtt, 0, sizeof ( write_mtt ) );
		MLX_FILL_1 ( &write_mtt.mtt_base_addr, 1,
			     value, mtt_base_addr );
		MLX_FILL_2 ( &write_mtt.mtt, 1,
			     p, 1,
			     ptag_l, ( start >> 3 ) );
		if ( ( rc = hermon_cmd_write_mtt ( hermon,
						   &write_mtt ) ) != 0 ) {
			DBGC ( hermon, "Hermon %p could not write MTT at %x\n",
			       hermon, mtt_base_addr );
			goto err_write_mtt;
		}
		start += HERMON_PAGE_SIZE;
		mtt_base_addr += hermon->cap.mtt_entry_size;
	}

	return 0;

 err_write_mtt:
	hermon_bitmask_free ( hermon->mtt_inuse, mtt_offset, num_pages );
 err_mtt_offset:
	return rc;
}

/**
 * Free MTT entries
 *
 * @v hermon		Hermon device
 * @v mtt		MTT descriptor
 */
static void hermon_free_mtt ( struct hermon *hermon,
			      struct hermon_mtt *mtt ) {
	hermon_bitmask_free ( hermon->mtt_inuse, mtt->mtt_offset,
			      mtt->num_pages );
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
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_completion_queue *hermon_cq;
	struct hermonprm_completion_queue_context cqctx;
	int cqn_offset;
	unsigned int i;
	int rc;

	/* Find a free completion queue number */
	cqn_offset = hermon_bitmask_alloc ( hermon->cq_inuse,
					    HERMON_MAX_CQS, 1 );
	if ( cqn_offset < 0 ) {
		DBGC ( hermon, "Hermon %p out of completion queues\n",
		       hermon );
		rc = cqn_offset;
		goto err_cqn_offset;
	}
	cq->cqn = ( hermon->cap.reserved_cqs + cqn_offset );

	/* Allocate control structures */
	hermon_cq = zalloc ( sizeof ( *hermon_cq ) );
	if ( ! hermon_cq ) {
		rc = -ENOMEM;
		goto err_hermon_cq;
	}

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

	/* Allocate MTT entries */
	if ( ( rc = hermon_alloc_mtt ( hermon, hermon_cq->cqe,
				       hermon_cq->cqe_size,
				       &hermon_cq->mtt ) ) != 0 )
		goto err_alloc_mtt;

	/* Hand queue over to hardware */
	memset ( &cqctx, 0, sizeof ( cqctx ) );
	MLX_FILL_1 ( &cqctx, 0, st, 0xa /* "Event fired" */ );
	MLX_FILL_1 ( &cqctx, 2,
		     page_offset, ( hermon_cq->mtt.page_offset >> 5 ) );
	MLX_FILL_2 ( &cqctx, 3,
		     usr_page, HERMON_UAR_PAGE,
		     log_cq_size, fls ( cq->num_cqes - 1 ) );
	MLX_FILL_1 ( &cqctx, 7, mtt_base_addr_l,
		     ( hermon_cq->mtt.mtt_base_addr >> 3 ) );
	MLX_FILL_1 ( &cqctx, 15, db_record_addr_l,
		     ( virt_to_phys ( &hermon_cq->doorbell ) >> 3 ) );
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
	hermon_free_mtt ( hermon, &hermon_cq->mtt );
 err_alloc_mtt:
	free_dma ( hermon_cq->cqe, hermon_cq->cqe_size );
 err_cqe:
	free ( hermon_cq );
 err_hermon_cq:
	hermon_bitmask_free ( hermon->cq_inuse, cqn_offset, 1 );
 err_cqn_offset:
	return rc;
}

/**
 * Destroy completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 */
static void hermon_destroy_cq ( struct ib_device *ibdev,
				struct ib_completion_queue *cq ) {
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_completion_queue *hermon_cq = cq->dev_priv;
	struct hermonprm_completion_queue_context cqctx;
	int cqn_offset;
	int rc;

	/* Take ownership back from hardware */
	if ( ( rc = hermon_cmd_hw2sw_cq ( hermon, cq->cqn, &cqctx ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p FATAL HW2SW_CQ failed on CQN %#lx: "
		       "%s\n", hermon, cq->cqn, strerror ( rc ) );
		/* Leak memory and return; at least we avoid corruption */
		return;
	}

	/* Free MTT entries */
	hermon_free_mtt ( hermon, &hermon_cq->mtt );

	/* Free memory */
	free_dma ( hermon_cq->cqe, hermon_cq->cqe_size );
	free ( hermon_cq );

	/* Mark queue number as free */
	cqn_offset = ( cq->cqn - hermon->cap.reserved_cqs );
	hermon_bitmask_free ( hermon->cq_inuse, cqn_offset, 1 );

	cq->dev_priv = NULL;
}

/***************************************************************************
 *
 * Queue pair operations
 *
 ***************************************************************************
 */

/**
 * Create queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @ret rc		Return status code
 */
static int hermon_create_qp ( struct ib_device *ibdev,
			      struct ib_queue_pair *qp ) {
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_queue_pair *hermon_qp;
	struct hermonprm_qp_ee_state_transitions qpctx;
	int qpn_offset;
	int rc;

	/* Find a free queue pair number */
	qpn_offset = hermon_bitmask_alloc ( hermon->qp_inuse,
					    HERMON_MAX_QPS, 1 );
	if ( qpn_offset < 0 ) {
		DBGC ( hermon, "Hermon %p out of queue pairs\n", hermon );
		rc = qpn_offset;
		goto err_qpn_offset;
	}
	qp->qpn = ( HERMON_QPN_BASE + hermon->cap.reserved_qps +
		    qpn_offset );

	/* Allocate control structures */
	hermon_qp = zalloc ( sizeof ( *hermon_qp ) );
	if ( ! hermon_qp ) {
		rc = -ENOMEM;
		goto err_hermon_qp;
	}

	/* Allocate work queue buffer */
	hermon_qp->send.num_wqes = ( qp->send.num_wqes /* headroom */ + 1 +
				( 2048 / sizeof ( hermon_qp->send.wqe[0] ) ) );
	hermon_qp->send.num_wqes =
		( 1 << fls ( hermon_qp->send.num_wqes - 1 ) ); /* round up */
	hermon_qp->send.wqe_size = ( hermon_qp->send.num_wqes *
				     sizeof ( hermon_qp->send.wqe[0] ) );
	hermon_qp->recv.wqe_size = ( qp->recv.num_wqes *
				     sizeof ( hermon_qp->recv.wqe[0] ) );
	hermon_qp->wqe_size = ( hermon_qp->send.wqe_size +
				hermon_qp->recv.wqe_size );
	hermon_qp->wqe = malloc_dma ( hermon_qp->wqe_size,
				      sizeof ( hermon_qp->send.wqe[0] ) );
	if ( ! hermon_qp->wqe ) {
		rc = -ENOMEM;
		goto err_alloc_wqe;
	}
	hermon_qp->send.wqe = hermon_qp->wqe;
	memset ( hermon_qp->send.wqe, 0xff, hermon_qp->send.wqe_size );
	hermon_qp->recv.wqe = ( hermon_qp->wqe + hermon_qp->send.wqe_size );
	memset ( hermon_qp->recv.wqe, 0, hermon_qp->recv.wqe_size );

	/* Allocate MTT entries */
	if ( ( rc = hermon_alloc_mtt ( hermon, hermon_qp->wqe,
				       hermon_qp->wqe_size,
				       &hermon_qp->mtt ) ) != 0 ) {
		goto err_alloc_mtt;
	}

	/* Hand queue over to hardware */
	memset ( &qpctx, 0, sizeof ( qpctx ) );
	MLX_FILL_2 ( &qpctx, 2,
		     qpc_eec_data.pm_state, 0x03 /* Always 0x03 for UD */,
		     qpc_eec_data.st, HERMON_ST_UD );
	MLX_FILL_1 ( &qpctx, 3, qpc_eec_data.pd, HERMON_GLOBAL_PD );
	MLX_FILL_4 ( &qpctx, 4,
		     qpc_eec_data.log_rq_size, fls ( qp->recv.num_wqes - 1 ),
		     qpc_eec_data.log_rq_stride,
		     ( fls ( sizeof ( hermon_qp->recv.wqe[0] ) - 1 ) - 4 ),
		     qpc_eec_data.log_sq_size,
		     fls ( hermon_qp->send.num_wqes - 1 ),
		     qpc_eec_data.log_sq_stride,
		     ( fls ( sizeof ( hermon_qp->send.wqe[0] ) - 1 ) - 4 ) );
	MLX_FILL_1 ( &qpctx, 5,
		     qpc_eec_data.usr_page, HERMON_UAR_PAGE );
	MLX_FILL_1 ( &qpctx, 33, qpc_eec_data.cqn_snd, qp->send.cq->cqn );
	MLX_FILL_1 ( &qpctx, 38, qpc_eec_data.page_offset,
		     ( hermon_qp->mtt.page_offset >> 6 ) );
	MLX_FILL_1 ( &qpctx, 41, qpc_eec_data.cqn_rcv, qp->recv.cq->cqn );
	MLX_FILL_1 ( &qpctx, 43, qpc_eec_data.db_record_addr_l,
		     ( virt_to_phys ( &hermon_qp->recv.doorbell ) >> 2 ) );
	MLX_FILL_1 ( &qpctx, 44, qpc_eec_data.q_key, qp->qkey );
	MLX_FILL_1 ( &qpctx, 53, qpc_eec_data.mtt_base_addr_l,
		     ( hermon_qp->mtt.mtt_base_addr >> 3 ) );
	if ( ( rc = hermon_cmd_rst2init_qp ( hermon, qp->qpn,
					     &qpctx ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p RST2INIT_QP failed: %s\n",
		       hermon, strerror ( rc ) );
		goto err_rst2init_qp;
	}

	memset ( &qpctx, 0, sizeof ( qpctx ) );
	MLX_FILL_2 ( &qpctx, 4,
		     qpc_eec_data.mtu, HERMON_MTU_2048,
		     qpc_eec_data.msg_max, 11 /* 2^11 = 2048 */ );
	MLX_FILL_1 ( &qpctx, 16,
		     qpc_eec_data.primary_address_path.sched_queue,
		     ( 0x83 /* default policy */ |
		       ( ( PXE_IB_PORT - 1 ) << 6 ) ) );
	if ( ( rc = hermon_cmd_init2rtr_qp ( hermon, qp->qpn,
					     &qpctx ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p INIT2RTR_QP failed: %s\n",
		       hermon, strerror ( rc ) );
		goto err_init2rtr_qp;
	}
	memset ( &qpctx, 0, sizeof ( qpctx ) );
	if ( ( rc = hermon_cmd_rtr2rts_qp ( hermon, qp->qpn, &qpctx ) ) != 0 ){
		DBGC ( hermon, "Hermon %p RTR2RTS_QP failed: %s\n",
		       hermon, strerror ( rc ) );
		goto err_rtr2rts_qp;
	}

	DBGC ( hermon, "Hermon %p QPN %#lx send ring at [%p,%p)\n",
	       hermon, qp->qpn, hermon_qp->send.wqe,
	       ( ((void *)hermon_qp->send.wqe ) + hermon_qp->send.wqe_size ) );
	DBGC ( hermon, "Hermon %p QPN %#lx receive ring at [%p,%p)\n",
	       hermon, qp->qpn, hermon_qp->recv.wqe,
	       ( ((void *)hermon_qp->recv.wqe ) + hermon_qp->recv.wqe_size ) );
	qp->dev_priv = hermon_qp;
	return 0;

 err_rtr2rts_qp:
 err_init2rtr_qp:
	hermon_cmd_2rst_qp ( hermon, qp->qpn );
 err_rst2init_qp:
	hermon_free_mtt ( hermon, &hermon_qp->mtt );
 err_alloc_mtt:
	free_dma ( hermon_qp->wqe, hermon_qp->wqe_size );
 err_alloc_wqe:
	free ( hermon_qp );
 err_hermon_qp:
	hermon_bitmask_free ( hermon->qp_inuse, qpn_offset, 1 );
 err_qpn_offset:
	return rc;
}

/**
 * Destroy queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 */
static void hermon_destroy_qp ( struct ib_device *ibdev,
				struct ib_queue_pair *qp ) {
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_queue_pair *hermon_qp = qp->dev_priv;
	int qpn_offset;
	int rc;

	/* Take ownership back from hardware */
	if ( ( rc = hermon_cmd_2rst_qp ( hermon, qp->qpn ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p FATAL 2RST_QP failed on QPN %#lx: "
		       "%s\n", hermon, qp->qpn, strerror ( rc ) );
		/* Leak memory and return; at least we avoid corruption */
		return;
	}

	/* Free MTT entries */
	hermon_free_mtt ( hermon, &hermon_qp->mtt );

	/* Free memory */
	free_dma ( hermon_qp->wqe, hermon_qp->wqe_size );
	free ( hermon_qp );

	/* Mark queue number as free */
	qpn_offset = ( qp->qpn - HERMON_QPN_BASE -
		       hermon->cap.reserved_qps );
	hermon_bitmask_free ( hermon->qp_inuse, qpn_offset, 1 );

	qp->dev_priv = NULL;
}

/***************************************************************************
 *
 * Work request operations
 *
 ***************************************************************************
 */

/** GID used for GID-less send work queue entries */
static const struct ib_gid hermon_no_gid = {
	{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }
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
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_queue_pair *hermon_qp = qp->dev_priv;
	struct ib_work_queue *wq = &qp->send;
	struct hermon_send_work_queue *hermon_send_wq = &hermon_qp->send;
	struct hermonprm_ud_send_wqe *wqe;
	const struct ib_gid *gid;
	union hermonprm_doorbell_register db_reg;
	unsigned int wqe_idx_mask;

	/* Allocate work queue entry */
	wqe_idx_mask = ( wq->num_wqes - 1 );
	if ( wq->iobufs[wq->next_idx & wqe_idx_mask] ) {
		DBGC ( hermon, "Hermon %p send queue full", hermon );
		return -ENOBUFS;
	}
	wq->iobufs[wq->next_idx & wqe_idx_mask] = iobuf;
	wqe = &hermon_send_wq->wqe[ wq->next_idx &
				    ( hermon_send_wq->num_wqes - 1 ) ].ud;

	/* Construct work queue entry */
	memset ( ( ( ( void * ) wqe ) + 4 /* avoid ctrl.owner */ ), 0,
		   ( sizeof ( *wqe ) - 4 ) );
	MLX_FILL_1 ( &wqe->ctrl, 1, ds, ( sizeof ( *wqe ) / 16 ) );
	MLX_FILL_1 ( &wqe->ctrl, 2, c, 0x03 /* generate completion */ );
	MLX_FILL_2 ( &wqe->ud, 0,
		     ud_address_vector.pd, HERMON_GLOBAL_PD,
		     ud_address_vector.port_number, PXE_IB_PORT );
	MLX_FILL_2 ( &wqe->ud, 1,
		     ud_address_vector.rlid, av->dlid,
		     ud_address_vector.g, av->gid_present );
	MLX_FILL_1 ( &wqe->ud, 2,
		     ud_address_vector.max_stat_rate,
		     ( ( ( av->rate < 2 ) || ( av->rate > 10 ) ) ?
		       8 : ( av->rate + 5 ) ) );
	MLX_FILL_1 ( &wqe->ud, 3, ud_address_vector.sl, av->sl );
	gid = ( av->gid_present ? &av->gid : &hermon_no_gid );
	memcpy ( &wqe->ud.u.dwords[4], gid, sizeof ( *gid ) );
	MLX_FILL_1 ( &wqe->ud, 8, destination_qp, av->dest_qp );
	MLX_FILL_1 ( &wqe->ud, 9, q_key, av->qkey );
	MLX_FILL_1 ( &wqe->data[0], 0, byte_count, iob_len ( iobuf ) );
	MLX_FILL_1 ( &wqe->data[0], 1, l_key, hermon->reserved_lkey );
	MLX_FILL_1 ( &wqe->data[0], 3,
		     local_address_l, virt_to_bus ( iobuf->data ) );
	barrier();
	MLX_FILL_2 ( &wqe->ctrl, 0,
		     opcode, HERMON_OPCODE_SEND,
		     owner,
		     ( ( wq->next_idx & hermon_send_wq->num_wqes ) ? 1 : 0 ) );
	DBGCP ( hermon, "Hermon %p posting send WQE:\n", hermon );
	DBGCP_HD ( hermon, wqe, sizeof ( *wqe ) );
	barrier();

	/* Ring doorbell register */
	MLX_FILL_1 ( &db_reg.send, 0, qn, qp->qpn );
	DBGCP ( hermon, "Ringing doorbell %08lx with %08lx\n",
		virt_to_phys ( hermon->uar + HERMON_DB_POST_SND_OFFSET ),
		db_reg.dword[0] );
	writel ( db_reg.dword[0], ( hermon->uar + HERMON_DB_POST_SND_OFFSET ));

	/* Update work queue's index */
	wq->next_idx++;

	return 0;
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
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_queue_pair *hermon_qp = qp->dev_priv;
	struct ib_work_queue *wq = &qp->recv;
	struct hermon_recv_work_queue *hermon_recv_wq = &hermon_qp->recv;
	struct hermonprm_recv_wqe *wqe;
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

	/* Update work queue's index */
	wq->next_idx++;

	/* Update doorbell record */
	barrier();
	MLX_FILL_1 ( &hermon_recv_wq->doorbell, 0, receive_wqe_counter,
		     ( wq->next_idx & 0xffff ) );

	return 0;
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
	struct io_buffer *iobuf;
	ib_completer_t complete;
	unsigned int opcode;
	unsigned long qpn;
	int is_send;
	unsigned int wqe_idx;
	int rc = 0;

	/* Parse completion */
	memset ( &completion, 0, sizeof ( completion ) );
	qpn = MLX_GET ( &cqe->normal, qpn );
	is_send = MLX_GET ( &cqe->normal, s_r );
	opcode = MLX_GET ( &cqe->normal, opcode );
	if ( opcode >= HERMON_OPCODE_RECV_ERROR ) {
		/* "s" field is not valid for error opcodes */
		is_send = ( opcode == HERMON_OPCODE_SEND_ERROR );
		completion.syndrome = MLX_GET ( &cqe->error, syndrome );
		DBGC ( hermon, "Hermon %p CQN %lx syndrome %x vendor %lx\n",
		       hermon, cq->cqn, completion.syndrome,
		       MLX_GET ( &cqe->error, vendor_error_syndrome ) );
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

	/* Identify I/O buffer */
	wqe_idx = ( MLX_GET ( &cqe->normal, wqe_counter ) &
		    ( wq->num_wqes - 1 ) );
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
	struct hermon *hermon = ibdev->dev_priv;
	struct hermon_completion_queue *hermon_cq = cq->dev_priv;
	union hermonprm_completion_entry *cqe;
	unsigned int cqe_idx_mask;
	int rc;

	while ( 1 ) {
		/* Look for completion entry */
		cqe_idx_mask = ( cq->num_cqes - 1 );
		cqe = &hermon_cq->cqe[cq->next_idx & cqe_idx_mask];
		if ( MLX_GET ( &cqe->normal, owner ) ^
		     ( ( cq->next_idx & cq->num_cqes ) ? 1 : 0 ) ) {
			/* Entry still owned by hardware; end of poll */
			break;
		}
		DBGCP ( hermon, "Hermon %p completion:\n", hermon );
		DBGCP_HD ( hermon, cqe, sizeof ( *cqe ) );

		/* Handle completion */
		if ( ( rc = hermon_complete ( ibdev, cq, cqe, complete_send,
					      complete_recv ) ) != 0 ) {
			DBGC ( hermon, "Hermon %p failed to complete: %s\n",
			       hermon, strerror ( rc ) );
			DBGC_HD ( hermon, cqe, sizeof ( *cqe ) );
		}

		/* Update completion queue's index */
		cq->next_idx++;

		/* Update doorbell record */
		MLX_FILL_1 ( &hermon_cq->doorbell, 0, update_ci,
			     ( cq->next_idx & 0xffffffUL ) );
	}
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
	struct hermonprm_mcg_entry mcg;
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
	if ( ( rc = hermon_cmd_read_mcg ( hermon, index, &mcg ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not read MCG %#x: %s\n",
		       hermon, index, strerror ( rc ) );
		return rc;
	}
	if ( MLX_GET ( &mcg, hdr.members_count ) != 0 ) {
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
	MLX_FILL_1 ( &mcg, 1, hdr.members_count, 1 );
	MLX_FILL_1 ( &mcg, 8, qp[0].qpn, qp->qpn );
	memcpy ( &mcg.u.dwords[4], gid, sizeof ( *gid ) );
	if ( ( rc = hermon_cmd_write_mcg ( hermon, index, &mcg ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not write MCG %#x: %s\n",
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
	struct hermonprm_mcg_entry mcg;
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
	memset ( &mcg, 0, sizeof ( mcg ) );
	if ( ( rc = hermon_cmd_write_mcg ( hermon, index, &mcg ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not write MCG %#x: %s\n",
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
	fw_size = ( fw_pages * HERMON_PAGE_SIZE );
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
static int hermon_get_cap ( struct hermon *hermon ) {
	struct hermonprm_query_dev_cap dev_cap;
	int rc;

	if ( ( rc = hermon_cmd_query_dev_cap ( hermon, &dev_cap ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not get device limits: %s\n",
		       hermon, strerror ( rc ) );
		return rc;
	}

	hermon->cap.cmpt_entry_size = MLX_GET ( &dev_cap, c_mpt_entry_sz );
	hermon->cap.reserved_qps =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_qps ) );
	hermon->cap.qpc_entry_size = MLX_GET ( &dev_cap, qpc_entry_sz );
	hermon->cap.altc_entry_size = MLX_GET ( &dev_cap, altc_entry_sz );
	hermon->cap.auxc_entry_size = MLX_GET ( &dev_cap, aux_entry_sz );
	hermon->cap.reserved_srqs =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_srqs ) );
	hermon->cap.srqc_entry_size = MLX_GET ( &dev_cap, srq_entry_sz );
	hermon->cap.reserved_cqs =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_cqs ) );
	hermon->cap.cqc_entry_size = MLX_GET ( &dev_cap, cqc_entry_sz );
	hermon->cap.reserved_eqs = MLX_GET ( &dev_cap, num_rsvd_eqs );
	hermon->cap.eqc_entry_size = MLX_GET ( &dev_cap, eqc_entry_sz );
	hermon->cap.reserved_mtts =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_mtts ) );
	hermon->cap.mtt_entry_size = MLX_GET ( &dev_cap, mtt_entry_sz );
	hermon->cap.reserved_mrws =
		( 1 << MLX_GET ( &dev_cap, log2_rsvd_mrws ) );
	hermon->cap.dmpt_entry_size = MLX_GET ( &dev_cap, d_mpt_entry_sz );
	hermon->cap.reserved_uars = MLX_GET ( &dev_cap, num_rsvd_uars );

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
	usage = ( ( usage + HERMON_PAGE_SIZE - 1 ) &
		  ~( HERMON_PAGE_SIZE - 1 ) );
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
	struct hermonprm_scalar_parameter icm_size;
	struct hermonprm_scalar_parameter icm_aux_size;
	struct hermonprm_virtual_physical_mapping map_icm_aux;
	struct hermonprm_virtual_physical_mapping map_icm;
	uint64_t icm_offset = 0;
	unsigned int log_num_qps, log_num_srqs, log_num_cqs, log_num_eqs;
	unsigned int log_num_mtts, log_num_mpts;
	size_t cmpt_max_len;
	size_t qp_cmpt_len, srq_cmpt_len, cq_cmpt_len, eq_cmpt_len;
	size_t icm_len, icm_aux_len;
	physaddr_t icm_phys;
	int i;
	int rc;

	/*
	 * Start by carving up the ICM virtual address space
	 *
	 */

	/* Calculate number of each object type within ICM */
	log_num_qps = fls ( hermon->cap.reserved_qps + HERMON_MAX_QPS - 1 );
	log_num_srqs = fls ( hermon->cap.reserved_srqs - 1 );
	log_num_cqs = fls ( hermon->cap.reserved_cqs + HERMON_MAX_CQS - 1 );
	log_num_eqs = fls ( hermon->cap.reserved_eqs + HERMON_MAX_EQS - 1 );
	log_num_mtts = fls ( hermon->cap.reserved_mtts + HERMON_MAX_MTTS - 1 );

	/* ICM starts with the cMPT tables, which are sparse */
	cmpt_max_len = ( HERMON_CMPT_MAX_ENTRIES *
			 ( ( uint64_t ) hermon->cap.cmpt_entry_size ) );
	qp_cmpt_len = icm_usage ( log_num_qps, hermon->cap.cmpt_entry_size );
	hermon->icm_map[HERMON_ICM_QP_CMPT].offset = icm_offset;
	hermon->icm_map[HERMON_ICM_QP_CMPT].len = qp_cmpt_len;
	icm_offset += cmpt_max_len;
	srq_cmpt_len = icm_usage ( log_num_srqs, hermon->cap.cmpt_entry_size );
	hermon->icm_map[HERMON_ICM_SRQ_CMPT].offset = icm_offset;
	hermon->icm_map[HERMON_ICM_SRQ_CMPT].len = srq_cmpt_len;
	icm_offset += cmpt_max_len;
	cq_cmpt_len = icm_usage ( log_num_cqs, hermon->cap.cmpt_entry_size );
	hermon->icm_map[HERMON_ICM_CQ_CMPT].offset = icm_offset;
	hermon->icm_map[HERMON_ICM_CQ_CMPT].len = cq_cmpt_len;
	icm_offset += cmpt_max_len;
	eq_cmpt_len = icm_usage ( log_num_eqs, hermon->cap.cmpt_entry_size );
	hermon->icm_map[HERMON_ICM_EQ_CMPT].offset = icm_offset;
	hermon->icm_map[HERMON_ICM_EQ_CMPT].len = eq_cmpt_len;
	icm_offset += cmpt_max_len;

	hermon->icm_map[HERMON_ICM_OTHER].offset = icm_offset;

	/* Queue pair contexts */
	MLX_FILL_1 ( init_hca, 12,
		     qpc_eec_cqc_eqc_rdb_parameters.qpc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_2 ( init_hca, 13,
		     qpc_eec_cqc_eqc_rdb_parameters.qpc_base_addr_l,
		     ( icm_offset >> 5 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_qp,
		     log_num_qps );
	DBGC ( hermon, "Hermon %p ICM QPC base = %llx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_qps, hermon->cap.qpc_entry_size );

	/* Extended alternate path contexts */
	MLX_FILL_1 ( init_hca, 24,
		     qpc_eec_cqc_eqc_rdb_parameters.altc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_1 ( init_hca, 25,
		     qpc_eec_cqc_eqc_rdb_parameters.altc_base_addr_l,
		     icm_offset );
	DBGC ( hermon, "Hermon %p ICM ALTC base = %llx\n", hermon, icm_offset);
	icm_offset += icm_usage ( log_num_qps,
				  hermon->cap.altc_entry_size );

	/* Extended auxiliary contexts */
	MLX_FILL_1 ( init_hca, 28,
		     qpc_eec_cqc_eqc_rdb_parameters.auxc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_1 ( init_hca, 29,
		     qpc_eec_cqc_eqc_rdb_parameters.auxc_base_addr_l,
		     icm_offset );
	DBGC ( hermon, "Hermon %p ICM AUXC base = %llx\n", hermon, icm_offset);
	icm_offset += icm_usage ( log_num_qps,
				  hermon->cap.auxc_entry_size );

	/* Shared receive queue contexts */
	MLX_FILL_1 ( init_hca, 18,
		     qpc_eec_cqc_eqc_rdb_parameters.srqc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_2 ( init_hca, 19,
		     qpc_eec_cqc_eqc_rdb_parameters.srqc_base_addr_l,
		     ( icm_offset >> 5 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_srq,
		     log_num_srqs );
	DBGC ( hermon, "Hermon %p ICM SRQC base = %llx\n", hermon, icm_offset);
	icm_offset += icm_usage ( log_num_srqs,
				  hermon->cap.srqc_entry_size );

	/* Completion queue contexts */
	MLX_FILL_1 ( init_hca, 20,
		     qpc_eec_cqc_eqc_rdb_parameters.cqc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_2 ( init_hca, 21,
		     qpc_eec_cqc_eqc_rdb_parameters.cqc_base_addr_l,
		     ( icm_offset >> 5 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_cq,
		     log_num_cqs );
	DBGC ( hermon, "Hermon %p ICM CQC base = %llx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_cqs, hermon->cap.cqc_entry_size );

	/* Event queue contexts */
	MLX_FILL_1 ( init_hca, 32,
		     qpc_eec_cqc_eqc_rdb_parameters.eqc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_2 ( init_hca, 33,
		     qpc_eec_cqc_eqc_rdb_parameters.eqc_base_addr_l,
		     ( icm_offset >> 5 ),
		     qpc_eec_cqc_eqc_rdb_parameters.log_num_of_eq,
		     log_num_eqs );
	DBGC ( hermon, "Hermon %p ICM EQC base = %llx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_eqs, hermon->cap.eqc_entry_size );

	/* Memory translation table */
	MLX_FILL_1 ( init_hca, 64,
		     tpt_parameters.mtt_base_addr_h, ( icm_offset >> 32 ) );
	MLX_FILL_1 ( init_hca, 65,
		     tpt_parameters.mtt_base_addr_l, icm_offset );
	DBGC ( hermon, "Hermon %p ICM MTT base = %llx\n", hermon, icm_offset );
	icm_offset += icm_usage ( log_num_mtts,
				  hermon->cap.mtt_entry_size );

	/* Memory protection table */
	log_num_mpts = fls ( hermon->cap.reserved_mrws + 1 - 1 );
	MLX_FILL_1 ( init_hca, 60,
		     tpt_parameters.dmpt_base_adr_h, ( icm_offset >> 32 ) );
	MLX_FILL_1 ( init_hca, 61,
		     tpt_parameters.dmpt_base_adr_l, icm_offset );
	MLX_FILL_1 ( init_hca, 62,
		     tpt_parameters.log_dmpt_sz, log_num_mpts );
	DBGC ( hermon, "Hermon %p ICM DMPT base = %llx\n", hermon, icm_offset);
	icm_offset += icm_usage ( log_num_mpts,
				  hermon->cap.dmpt_entry_size );

	/* Multicast table */
	MLX_FILL_1 ( init_hca, 48,
		     multicast_parameters.mc_base_addr_h,
		     ( icm_offset >> 32 ) );
	MLX_FILL_1 ( init_hca, 49,
		     multicast_parameters.mc_base_addr_l, icm_offset );
	MLX_FILL_1 ( init_hca, 52,
		     multicast_parameters.log_mc_table_entry_sz,
		     fls ( sizeof ( struct hermonprm_mcg_entry ) - 1 ) );
	MLX_FILL_1 ( init_hca, 53,
		     multicast_parameters.log_mc_table_hash_sz, 3 );
	MLX_FILL_1 ( init_hca, 54,
		     multicast_parameters.log_mc_table_sz, 3 );
	DBGC ( hermon, "Hermon %p ICM MC base = %llx\n", hermon, icm_offset );
	icm_offset += ( ( 8 * sizeof ( struct hermonprm_mcg_entry ) +
			  HERMON_PAGE_SIZE - 1 ) & ~( HERMON_PAGE_SIZE - 1 ) );

	hermon->icm_map[HERMON_ICM_OTHER].len =
		( icm_offset - hermon->icm_map[HERMON_ICM_OTHER].offset );

	/*
	 * Allocate and map physical memory for (portions of) ICM
	 *
	 * Map is:
	 *   ICM AUX area (aligned to its own size)
	 *   cMPT areas
	 *   Other areas
	 */

	/* Calculate physical memory required for ICM */
	icm_len = 0;
	for ( i = 0 ; i < HERMON_ICM_NUM_REGIONS ; i++ ) {
		icm_len += hermon->icm_map[i].len;
	}

	/* Get ICM auxiliary area size */
	memset ( &icm_size, 0, sizeof ( icm_size ) );
	MLX_FILL_1 ( &icm_size, 0, value_hi, ( icm_offset >> 32 ) );
	MLX_FILL_1 ( &icm_size, 1, value, icm_offset );
	if ( ( rc = hermon_cmd_set_icm_size ( hermon, &icm_size,
					      &icm_aux_size ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not set ICM size: %s\n",
		       hermon, strerror ( rc ) );
		goto err_set_icm_size;
	}
	icm_aux_len = ( MLX_GET ( &icm_aux_size, value ) * HERMON_PAGE_SIZE );
	/* Must round up to nearest power of two :( */
	icm_aux_len = ( 1 << fls ( icm_aux_len - 1 ) );

	/* Allocate ICM data and auxiliary area */
	DBGC ( hermon, "Hermon %p requires %zd kB ICM and %zd kB AUX ICM\n",
	       hermon, ( icm_len / 1024 ), ( icm_aux_len / 1024 ) );
	hermon->icm = umalloc ( 2 * icm_aux_len + icm_len );
	if ( ! hermon->icm ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	icm_phys = user_to_phys ( hermon->icm, 0 );

	/* Map ICM auxiliary area */
	icm_phys = ( ( icm_phys + icm_aux_len - 1 ) & ~( icm_aux_len - 1 ) );
	memset ( &map_icm_aux, 0, sizeof ( map_icm_aux ) );
	MLX_FILL_2 ( &map_icm_aux, 3,
		     log2size, fls ( ( icm_aux_len / HERMON_PAGE_SIZE ) - 1 ),
		     pa_l, ( icm_phys >> 12 ) );
	DBGC ( hermon, "Hermon %p mapping ICM AUX (2^%d pages) => %08lx\n",
	       hermon, fls ( ( icm_aux_len / HERMON_PAGE_SIZE ) - 1 ),
	       icm_phys );
	if ( ( rc = hermon_cmd_map_icm_aux ( hermon, &map_icm_aux ) ) != 0 ) {
		DBGC ( hermon, "Hermon %p could not map AUX ICM: %s\n",
		       hermon, strerror ( rc ) );
		goto err_map_icm_aux;
	}
	icm_phys += icm_aux_len;

	/* MAP ICM area */
	for ( i = 0 ; i < HERMON_ICM_NUM_REGIONS ; i++ ) {
		memset ( &map_icm, 0, sizeof ( map_icm ) );
		MLX_FILL_1 ( &map_icm, 0,
			     va_h, ( hermon->icm_map[i].offset >> 32 ) );
		MLX_FILL_1 ( &map_icm, 1,
			     va_l, ( hermon->icm_map[i].offset >> 12 ) );
		MLX_FILL_2 ( &map_icm, 3,
			     log2size,
			     fls ( ( hermon->icm_map[i].len /
				     HERMON_PAGE_SIZE ) - 1 ),
			     pa_l, ( icm_phys >> 12 ) );
		DBGC ( hermon, "Hermon %p mapping ICM %llx+%zx (2^%d pages) "
		       "=> %08lx\n", hermon, hermon->icm_map[i].offset,
		       hermon->icm_map[i].len,
		       fls ( ( hermon->icm_map[i].len /
			       HERMON_PAGE_SIZE ) - 1 ), icm_phys );
		if ( ( rc = hermon_cmd_map_icm ( hermon, &map_icm ) ) != 0 ) {
			DBGC ( hermon, "Hermon %p could not map ICM: %s\n",
			       hermon, strerror ( rc ) );
			goto err_map_icm;
		}
		icm_phys += hermon->icm_map[i].len;
	}

	return 0;

 err_map_icm:
	assert ( i == 0 ); /* We don't handle partial failure at present */
	hermon_cmd_unmap_icm_aux ( hermon );
 err_map_icm_aux:
	ufree ( hermon->icm );
	hermon->icm = UNULL;
 err_alloc:
 err_set_icm_size:
	return rc;
}

/**
 * Free ICM
 *
 * @v hermon		Hermon device
 */
static void hermon_free_icm ( struct hermon *hermon ) {
	struct hermonprm_scalar_parameter unmap_icm;
	int i;

	for ( i = ( HERMON_ICM_NUM_REGIONS - 1 ) ; i >= 0 ; i-- ) {
		memset ( &unmap_icm, 0, sizeof ( unmap_icm ) );
		MLX_FILL_1 ( &unmap_icm, 0, value_hi,
			     ( hermon->icm_map[i].offset >> 32 ) );
		MLX_FILL_1 ( &unmap_icm, 1, value,
			     hermon->icm_map[i].offset );
		hermon_cmd_unmap_icm ( hermon,
				       ( 1 << fls ( ( hermon->icm_map[i].len /
						      HERMON_PAGE_SIZE ) - 1)),
				       &unmap_icm );
	}
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
		     mtu, HERMON_MTU_2048,
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
	key = ( hermon->cap.reserved_mrws | HERMON_MKEY_PREFIX );
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
					   hermon->cap.reserved_mrws,
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
	hermon->uar = ioremap ( ( pci_bar_start ( pci, HERMON_PCI_UAR_BAR ) +
				  HERMON_UAR_PAGE * HERMON_PAGE_SIZE ),
				HERMON_PAGE_SIZE );

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
	if ( ( rc = hermon_get_cap ( hermon ) ) != 0 )
		goto err_get_cap;

	/* Allocate ICM */
	memset ( &init_hca, 0, sizeof ( init_hca ) );
	if ( ( rc = hermon_alloc_icm ( hermon, &init_hca ) ) != 0 )
		goto err_alloc_icm;

	/* Initialise HCA */
	MLX_FILL_1 ( &init_hca, 0, version, 0x02 /* "Must be 0x02" */ );
	MLX_FILL_1 ( &init_hca, 5, udp, 1 );
	MLX_FILL_1 ( &init_hca, 74, uar_parameters.log_max_uars, 8 );
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
 err_get_cap:
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
