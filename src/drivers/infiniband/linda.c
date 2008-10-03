/*
 * Copyright (C) 2008 Michael Brown <mbrown@fensystems.co.uk>.
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
#include <errno.h>
#include <unistd.h>
#include <assert.h>
#include <gpxe/pci.h>
#include <gpxe/infiniband.h>
#include <gpxe/i2c.h>
#include <gpxe/bitbash.h>
#include <gpxe/malloc.h>
#include <gpxe/iobuf.h>
#include "linda.h"

/**
 * @file
 *
 * QLogic Linda Infiniband HCA
 *
 */

/** A Linda context */
struct linda_context {
	/** Context is in use */
	unsigned int active;
	/** Receive header ring */
	void *header;
	/** Receive header producer offset (written by hardware) */
	struct QIB_7220_scalar header_prod;
	/** Receive header consumer offset */
	unsigned int header_cons;
	/** Offset within register space of the eager array */
	unsigned long eager_array;
	/** Number of entries in eager array */
	unsigned int eager_entries;
	/** Eager array producer offset */
	unsigned int eager_prod;
};

/** A Linda HCA */
struct linda {
	/** Registers */
	void *regs;
	/** Base GUID */
	uint8_t guid[LINDA_EEPROM_GUID_SIZE];

	/** Contexts */
	struct linda_context context[LINDA_NUM_CONTEXTS];

	/** Kernel context completion queue */
	struct ib_completion_queue *kernel_cq;
	/** Kernel context queue pair */
	struct ib_queue_pair *kernel_qp;
	/** Kernel context receive work queue fill level */
	unsigned int kernel_recv_fill;

	/** I2C bit-bashing interface */
	struct i2c_bit_basher i2c;
	/** I2C serial EEPROM */
	struct i2c_device eeprom;
};

static void linda_poll_kctx ( struct ib_device *ibdev );

/***************************************************************************
 *
 * Linda register access
 *
 ***************************************************************************
 *
 * This card requires atomic 64-bit accesses.  Strange things happen
 * if you try to use 32-bit accesses; sometimes they work, sometimes
 * they don't, sometimes you get random data.
 *
 * These accessors use the "movq" MMX instruction, and so won't work
 * on really old Pentiums (which won't have PCIe anyway, so this is
 * something of a moot point).
 */

/**
 * Read Linda qword register
 *
 * @v linda		Linda device
 * @v dwords		Register buffer to read into
 * @v offset		Register offset
 */
static void linda_readq ( struct linda *linda, uint32_t *dwords,
			  unsigned long offset ) {
	void *addr = ( linda->regs + offset );

	__asm__ __volatile__ ( "movq (%1), %%mm0\n\t"
			       "movq %%mm0, (%0)\n\t"
			       : : "r" ( dwords ), "r" ( addr ) : "memory" );

	DBGIO ( "[%08lx] => %08lx%08lx\n",
		virt_to_phys ( addr ), dwords[1], dwords[0] );
}
#define linda_readq( _linda, _ptr, _offset ) \
	linda_readq ( (_linda), (_ptr)->u.dwords, (_offset) )

/**
 * Write Linda qword register
 *
 * @v linda		Linda device
 * @v dwords		Register buffer to write
 * @v offset		Register offset
 */
static void linda_writeq ( struct linda *linda, const uint32_t *dwords,
			   unsigned long offset ) {
	void *addr = ( linda->regs + offset );

	DBGIO ( "[%08lx] <= %08lx%08lx\n",
		virt_to_phys ( addr ), dwords[1], dwords[0] );

	__asm__ __volatile__ ( "movq (%0), %%mm0\n\t"
			       "movq %%mm0, (%1)\n\t"
			       : : "r" ( dwords ), "r" ( addr ) : "memory" );
}
#define linda_writeq( _linda, _ptr, _offset ) \
	linda_writeq ( (_linda), (_ptr)->u.dwords, (_offset) )
#define linda_writeq_array8b( _linda, _ptr, _offset, _idx ) \
	linda_writeq ( (_linda), (_ptr), ( (_offset) + ( (_idx) * 8 ) ) )
#define linda_writeq_array64k( _linda, _ptr, _offset, _idx ) \
	linda_writeq ( (_linda), (_ptr), ( (_offset) + ( (_idx) * 65536 ) ) )

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
static int linda_create_cq ( struct ib_device *ibdev,
			     struct ib_completion_queue *cq ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	static int cqn;

	/* The hardware has no concept of completion queues.  We
	 * simply use the association between CQs and WQs (already
	 * handled by the IB core) to decide which WQs to poll.
	 *
	 * We do set a CQN, just to avoid confusing debug messages
	 * from the IB core.
	 */
	cq->cqn = ++cqn;
	DBGC ( linda, "Linda %p CQN %ld created\n", linda, cq->cqn );

	return 0;
}

/**
 * Destroy completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 */
static void linda_destroy_cq ( struct ib_device *ibdev,
			       struct ib_completion_queue *cq ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	/* Nothing to do */
	DBGC ( linda, "Linda %p CQN %ld destroyed\n", linda, cq->cqn );
}

/***************************************************************************
 *
 * Queue pair operations
 *
 ***************************************************************************
 */

/**
 * Map context number to QPN
 *
 * @v ctx		Context index
 * @ret qpn		Queue pair number
 */
static int linda_ctx_to_qpn ( unsigned int ctx ) {
	/* This mapping is fixed by hardware */
	return ( ctx * 2 );
}

/**
 * Map QPN to context number
 *
 * @v qpn		Queue pair number
 * @ret ctx		Context index
 */
static int linda_qpn_to_ctx ( unsigned int qpn ) {
	/* This mapping is fixed by hardware */
	return ( qpn / 2 );
}

/**
 * Allocate a context
 *
 * @v linda		Linda device
 * @ret ctx		Context index, or negative error
 */
static int linda_alloc_ctx ( struct linda *linda ) {
	unsigned int ctx;

	for ( ctx = 0 ; ctx < LINDA_NUM_CONTEXTS ; ctx++ ) {
		if ( ! linda->context[ctx].active ) {
			DBGC2 ( linda, "Linda %p CTX %d allocated\n",
				linda, ctx );
			linda->context[ctx].active = 1;
			return ctx;
		}
	}

	DBGC ( linda, "Linda %p out of available contexts\n", linda );
	return -ENOENT;
}

/**
 * Free a context
 *
 * @v linda		Linda device
 * @v ctx		Context index
 */
static void linda_free_ctx ( struct linda *linda, unsigned int ctx ) {
	DBGC2 ( linda, "Linda %p CTX %d freed\n", linda, ctx );
	linda->context[ctx].active = 0;
}

/**
 * Create queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @ret rc		Return status code
 */
static int linda_create_qp ( struct ib_device *ibdev,
			     struct ib_queue_pair *qp ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	struct linda_context *context;
	struct QIB_7220_RcvHdrAddr0 rcvhdraddr;
	struct QIB_7220_RcvHdrTailAddr0 rcvhdrtailaddr;
	struct QIB_7220_RcvHdrHead0 rcvhdrhead;
	struct QIB_7220_RcvCtrl rcvctrl;
	int ctx;
	int rc;

	/* Locate an available context */
	ctx = linda_alloc_ctx ( linda );
	if ( ctx < 0 ) {
		rc = ctx;
		goto err_alloc_ctx;
	}
	qp->qpn = linda_ctx_to_qpn ( ctx );
	context = &linda->context[ctx];

	/* Reset context information */
	memset ( &context->header_prod, 0, sizeof ( context->header_prod ) );
	context->header_cons = 0;

	/* Allocate receive header buffer */
	context->header = malloc_dma ( LINDA_RX_HEADERS_SIZE,
				       LINDA_RX_HEADERS_ALIGN );
	if ( ! context->header ) {
		rc = -ENOMEM;
		goto err_alloc_header;
	}

	/* Enable context in hardware */
	memset ( &rcvhdraddr, 0, sizeof ( rcvhdraddr ) );
	BIT_FILL_1 ( &rcvhdraddr, RcvHdrAddr0,
		     ( virt_to_bus ( context->header ) >> 2 ) );
	linda_writeq_array8b ( linda, &rcvhdraddr,
			       QIB_7220_RcvHdrAddr0_offset, ctx );
	memset ( &rcvhdrtailaddr, 0, sizeof ( rcvhdrtailaddr ) );
	BIT_FILL_1 ( &rcvhdrtailaddr, RcvHdrTailAddr0,
		     ( virt_to_bus ( &context->header_prod ) >> 2 ) );
	linda_writeq_array8b ( linda, &rcvhdrtailaddr,
			       QIB_7220_RcvHdrTailAddr0_offset, ctx );
	memset ( &rcvhdrhead, 0, sizeof ( rcvhdrhead ) );
	BIT_FILL_1 ( &rcvhdrhead, counter, 1 );
	linda_writeq_array64k ( linda, &rcvhdrhead,
				QIB_7220_RcvHdrHead0_offset, ctx );
	linda_readq ( linda, &rcvctrl, QIB_7220_RcvCtrl_offset );
	BIT_SET ( &rcvctrl, PortEnable[ctx], 1 );
	BIT_SET ( &rcvctrl, IntrAvail[ctx], 1 );
	linda_writeq ( linda, &rcvctrl, QIB_7220_RcvCtrl_offset );

	DBGC ( linda, "Linda %p QPN %ld using CTX %d hdrs [%lx,%lx) prod "
	       "%lx\n", linda, qp->qpn, ctx, virt_to_bus ( context->header ),
	       ( virt_to_bus ( context->header ) + LINDA_RX_HEADERS_SIZE ),
	       virt_to_bus ( &context->header_prod ) );
	return 0;

	free_dma ( context->header, LINDA_RX_HEADERS_SIZE );
 err_alloc_header:
	linda_free_ctx ( linda, ctx );
 err_alloc_ctx:
	return rc;
}

/**
 * Modify queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v mod_list		Modification list
 * @ret rc		Return status code
 */
static int linda_modify_qp ( struct ib_device *ibdev,
			     struct ib_queue_pair *qp,
			     unsigned long mod_list __unused ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	/* Nothing to do; the hardware doesn't have a notion of queue
	 * keys
	 */
	DBGC ( linda, "Linda %p QPN %ld modified\n", linda, qp->qpn );
	return 0;
}

/**
 * Destroy queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 */
static void linda_destroy_qp ( struct ib_device *ibdev,
			       struct ib_queue_pair *qp ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	unsigned int ctx = linda_qpn_to_ctx ( qp->qpn );
	struct linda_context *context = &linda->context[ctx];
	struct QIB_7220_RcvCtrl rcvctrl;

	/* Disable context in hardware */
	linda_readq ( linda, &rcvctrl, QIB_7220_RcvCtrl_offset );
	BIT_SET ( &rcvctrl, PortEnable[ctx], 0 );
	BIT_SET ( &rcvctrl, IntrAvail[ctx], 0 );
	linda_writeq ( linda, &rcvctrl, QIB_7220_RcvCtrl_offset );

	/* Make sure the hardware has seen that the context is disabled */
	linda_readq ( linda, &rcvctrl, QIB_7220_RcvCtrl_offset );
	mb();

	/* Free headers ring */
	free_dma ( context->header, LINDA_RX_HEADERS_SIZE );

	/* Free context */
	linda_free_ctx ( linda, ctx );
}

/***************************************************************************
 *
 * Work request operations
 *
 ***************************************************************************
 */

/**
 * Post send work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int linda_post_send ( struct ib_device *ibdev,
			     struct ib_queue_pair *qp,
			     struct ib_address_vector *av,
			     struct io_buffer *iobuf ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) qp;
	( void ) av;
	( void ) iobuf;
	return -ENOTSUP;
}

/**
 * Post receive work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int linda_post_recv ( struct ib_device *ibdev,
			     struct ib_queue_pair *qp,
			     struct io_buffer *iobuf ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	struct ib_work_queue *wq = &qp->recv;
	struct linda_context *context;
	struct QIB_7220_RcvEgr rcvegr;
	struct QIB_7220_scalar rcvegrindexhead;
	unsigned int ctx;
	unsigned int wqe_idx_mask;
	unsigned int bufsize;
	physaddr_t addr;
	size_t len;

	/* Allocate work queue entry */
	wqe_idx_mask = ( wq->num_wqes - 1 );
	if ( wq->iobufs[wq->next_idx & wqe_idx_mask] ) {
		DBGC ( linda, "Linda %p QPN %ld receive queue full\n",
		       linda, qp->qpn );
		return -ENOBUFS;
	}

	/* Sanity checks */
	addr = virt_to_bus ( iobuf->data );
	len = iob_tailroom ( iobuf );
	if ( addr & ( LINDA_EAGER_BUFFER_ALIGN - 1 ) ) {
		DBGC ( linda, "Linda %p QPN %ld misaligned RX buffer "
		       "(%08lx)\n", linda, qp->qpn, addr );
		return -EINVAL;
	}
	if ( len != LINDA_RX_PAYLOAD_SIZE ) {
		DBGC ( linda, "Linda %p QPN %ld wrong RX buffer size (%d)\n",
		       linda, qp->qpn, len );
		return -EINVAL;
	}

	/* Identify context for this queue */
	ctx = linda_qpn_to_ctx ( qp->qpn );
	context = &linda->context[ctx];

	/* Calculate buffer size */
	switch ( LINDA_RX_PAYLOAD_SIZE ) {
	case 2048:  bufsize = LINDA_EAGER_BUFFER_2K;  break;
	case 4096:  bufsize = LINDA_EAGER_BUFFER_4K;  break;
	case 8192:  bufsize = LINDA_EAGER_BUFFER_8K;  break;
	case 16384: bufsize = LINDA_EAGER_BUFFER_16K; break;
	case 32768: bufsize = LINDA_EAGER_BUFFER_32K; break;
	case 65536: bufsize = LINDA_EAGER_BUFFER_64K; break;
	default:    linker_assert ( 0, invalid_rx_payload_size );
		    bufsize = LINDA_EAGER_BUFFER_NONE;
	}

	/* Post eager buffer */
	memset ( &rcvegr, 0, sizeof ( rcvegr ) );
	BIT_FILL_2 ( &rcvegr, 
		     Addr, ( addr >> 11 ),
		     BufSize, bufsize );
	linda_writeq_array8b ( linda, &rcvegr,
			       context->eager_array, context->eager_prod );
	DBGC ( linda, "Linda %p QPN %ld posted RX idx %d [%lx,%lx)\n",
	       linda, qp->qpn, context->eager_prod, addr, ( addr + len ) );

	/* Increment producer offset */
	context->eager_prod = ( ( context->eager_prod + 1 ) &
				( context->eager_entries - 1 ) );
	memset ( &rcvegrindexhead, 0, sizeof ( rcvegrindexhead ) );
	BIT_FILL_1 ( &rcvegrindexhead, Value, context->eager_prod );
	linda_writeq_array64k ( linda, &rcvegrindexhead,
				QIB_7220_RcvEgrIndexHead0_offset, ctx );

	/* Update work queue's index */
	wq->iobufs[wq->next_idx & wqe_idx_mask] = iobuf;
	wq->next_idx++;

	return 0;
}

/**
 * Complete receive work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v context		Linda context
 * @v header_offs	Header offset
 */
static void linda_complete_recv ( struct ib_device *ibdev,
				  struct ib_queue_pair *qp,
				  struct linda_context *context,
				  unsigned int header_offs ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	struct QIB_7220_RcvHdrFlags *rcvhdrflags;
	struct ib_completion completion;
	unsigned int rcvtype;
	unsigned int pktlen;
	unsigned int egrindex;
	unsigned int useegrbfr;
	unsigned int iberr, mkerr, tiderr, khdrerr, mtuerr;
	unsigned int lenerr, parityerr, vcrcerr, icrcerr;
	unsigned int err;
	unsigned int hdrqoffset;
	unsigned int header_len;
	unsigned int payload_len;

	/* RcvHdrFlags are at the end of the header entry */
	rcvhdrflags = ( context->header + header_offs + LINDA_RX_HEADER_SIZE -
			sizeof ( *rcvhdrflags ) );
	rcvtype = BIT_GET ( rcvhdrflags, RcvType );
	pktlen = ( BIT_GET ( rcvhdrflags, PktLen ) << 2 );
	egrindex = BIT_GET ( rcvhdrflags, EgrIndex );
	useegrbfr = BIT_GET ( rcvhdrflags, UseEgrBfr );
	hdrqoffset = ( BIT_GET ( rcvhdrflags, HdrqOffset ) << 2 );
	iberr = BIT_GET ( rcvhdrflags, IBErr );
	mkerr = BIT_GET ( rcvhdrflags, MKErr );
	tiderr = BIT_GET ( rcvhdrflags, TIDErr );
	khdrerr = BIT_GET ( rcvhdrflags, KHdrErr );
	mtuerr = BIT_GET ( rcvhdrflags, MTUErr );
	lenerr = BIT_GET ( rcvhdrflags, LenErr );
	parityerr = BIT_GET ( rcvhdrflags, ParityErr );
	vcrcerr = BIT_GET ( rcvhdrflags, VCRCErr );
	icrcerr = BIT_GET ( rcvhdrflags, ICRCErr );
	header_len = ( LINDA_RX_HEADER_SIZE - hdrqoffset -
		       sizeof ( *rcvhdrflags ) );
	payload_len = ( pktlen - header_len - 4 /* ICRC */ );
	err = ( iberr | mkerr | tiderr | khdrerr | mtuerr |
		lenerr | parityerr | vcrcerr | icrcerr );
	DBGC ( linda, "Linda %p QPN %ld RX hdr %d type %d len %d(%d+%d+4) "
	       "egr %d%s%s%s%s%s%s%s%s%s%s%s%s\n", linda, qp->qpn,
	       ( header_offs / LINDA_RX_HEADER_SIZE ), rcvtype,
	       pktlen, header_len, payload_len, egrindex,
	       ( useegrbfr ? "" : "(unused)" ), ( err ? " [Err" : "" ),
	       ( iberr ? " IB" : "" ), ( mkerr ? " MK" : "" ),
	       ( tiderr ? " TID" : "" ), ( khdrerr ? " KHdr" : "" ),
	       ( mtuerr ? " MTU" : "" ), ( lenerr ? " Len" : "" ),
	       ( parityerr ? " Parity" : "" ), ( vcrcerr ? " VCRC" : "" ),
	       ( icrcerr ? " ICRC" : "" ), ( err ? "]" : "" ) );
	/* IB header is placed immediately before RcvHdrFlags */
	DBGC_HDA ( linda, 0,
		   ( ( ( void * ) rcvhdrflags ) - header_len ),
		   ( header_len + sizeof ( *rcvhdrflags ) ) );

	/* Construct IB completion indicator */
	memset ( &completion, 0, sizeof ( completion ) );
	completion.len = payload_len;
	if ( err ) {
		/* Report any errors as "local queue pair
		 * operation error" for simplicity.
		 */
		completion.syndrome = IB_SYN_LOCAL_QP;
	}
}

/**
 * Poll completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 */
static void linda_poll_cq ( struct ib_device *ibdev,
			    struct ib_completion_queue *cq ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	struct ib_work_queue *recv_wq;
	struct linda_context *context;
	struct QIB_7220_RcvHdrHead0 rcvhdrhead;
	unsigned int ctx;
	unsigned int header_prod;

	/* Poll associated RX queues */
	list_for_each_entry ( recv_wq, &cq->work_queues, list ) {
		if ( recv_wq->is_send )
			continue;

		/* Identify context for this queue */
		ctx = linda_qpn_to_ctx ( recv_wq->qp->qpn );
		context = &linda->context[ctx];

		/* Check for received packets */
		header_prod =
			( BIT_GET ( &context->header_prod, Value ) << 2 );
		if ( header_prod == context->header_cons )
			continue;

		/* Process all received packets */
		while ( context->header_cons != header_prod ) {

			/* Complete the receive */
			linda_complete_recv ( ibdev, recv_wq->qp,
					      context, context->header_cons );
			
			/* Increment the consumer offset */
			context->header_cons += LINDA_RX_HEADER_SIZE;
			context->header_cons %= LINDA_RX_HEADERS_SIZE;
		}
		
		/* Update consumer offset */
		memset ( &rcvhdrhead, 0, sizeof ( rcvhdrhead ) );
		BIT_FILL_2 ( &rcvhdrhead,
			     RcvHeadPointer, ( context->header_cons >> 2 ),
			     counter, 1 );
		linda_writeq_array64k ( linda, &rcvhdrhead,
					QIB_7220_RcvHdrHead0_offset, ctx );
	}
}

/***************************************************************************
 *
 * Event queues
 *
 ***************************************************************************
 */

/**
 * Textual representation of link state
 *
 * @v link_state	Link state
 * @ret link_text	Link state text
 */
static const char * linda_link_state_text ( unsigned int link_state ) {
	switch ( link_state ) {
	case LINDA_LINK_STATE_DOWN:	return "DOWN";
	case LINDA_LINK_STATE_INIT:	return "INIT";
	case LINDA_LINK_STATE_ARM:	return "ARM";
	case LINDA_LINK_STATE_ACTIVE:	return "ACTIVE";
	case LINDA_LINK_STATE_ACT_DEFER:return "ACT_DEFER";
	default:			return "UNKNOWN";
	}
}

/**
 * Handle link state change
 *
 * @v linda		Linda device
 */
static void linda_link_state_changed ( struct ib_device *ibdev ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	struct QIB_7220_IBCStatus ibcstatus;
	struct QIB_7220_EXTCtrl extctrl;
	unsigned int link_state;

	/* Read link state */
	linda_readq ( linda, &ibcstatus, QIB_7220_IBCStatus_offset );
	link_state = BIT_GET ( &ibcstatus, LinkState );
	DBGC ( linda, "Linda %p link state %s (%s %s)\n", linda,
	       linda_link_state_text ( link_state ),
	       ( BIT_GET ( &ibcstatus, LinkSpeedActive ) ? "DDR" : "SDR" ),
	       ( BIT_GET ( &ibcstatus, LinkWidthActive ) ? "x4" : "x1" ) );

	/* Set LEDs according to link state */
	linda_readq ( linda, &extctrl, QIB_7220_EXTCtrl_offset );
	BIT_SET ( &extctrl, LEDPriPortGreenOn,
		  ( ( link_state >= LINDA_LINK_STATE_INIT ) ? 1 : 0 ) );
	BIT_SET ( &extctrl, LEDPriPortYellowOn,
		  ( ( link_state >= LINDA_LINK_STATE_ACTIVE ) ? 1 : 0 ) );
	linda_writeq ( linda, &extctrl, QIB_7220_EXTCtrl_offset );

	/* Notify Infiniband core of link state change */
	ib_link_state_changed ( ibdev );
}

/**
 * Poll event queue
 *
 * @v ibdev		Infiniband device
 */
static void linda_poll_eq ( struct ib_device *ibdev ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	struct QIB_7220_ErrStatus errstatus;
	struct QIB_7220_ErrClear errclear;

	/* Check for link status changes */
	DBG_DISABLE ( DBGLVL_IO );
	linda_readq ( linda, &errstatus, QIB_7220_ErrStatus_offset );
	DBG_ENABLE ( DBGLVL_IO );
	if ( BIT_GET ( &errstatus, IBStatusChanged ) ) {
		linda_link_state_changed ( ibdev );
		memset ( &errclear, 0, sizeof ( errclear ) );
		BIT_FILL_1 ( &errclear, IBStatusChangedClear, 1 );
		linda_writeq ( linda, &errclear, QIB_7220_ErrClear_offset );
	}

	/* Poll the kernel context.  This seems the most logical place
	 * to hook it in.
	 */
	linda_poll_kctx ( ibdev );
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
 * @v ibdev		Infiniband device
 * @ret rc		Return status code
 */
static int linda_open ( struct ib_device *ibdev ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	struct QIB_7220_Control control;

	/* Disable link */
	linda_readq ( linda, &control, QIB_7220_Control_offset );
	BIT_SET ( &control, LinkEn, 1 );
	linda_writeq ( linda, &control, QIB_7220_Control_offset );
	return 0;
}

/**
 * Close Infiniband link
 *
 * @v ibdev		Infiniband device
 */
static void linda_close ( struct ib_device *ibdev ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	struct QIB_7220_Control control;

	/* Disable link */
	linda_readq ( linda, &control, QIB_7220_Control_offset );
	BIT_SET ( &control, LinkEn, 0 );
	linda_writeq ( linda, &control, QIB_7220_Control_offset );
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
static int linda_mcast_attach ( struct ib_device *ibdev,
				struct ib_queue_pair *qp,
				struct ib_gid *gid ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) qp;
	( void ) gid;
	return -ENOTSUP;
}

/**
 * Detach from multicast group
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v gid		Multicast GID
 */
static void linda_mcast_detach ( struct ib_device *ibdev,
				 struct ib_queue_pair *qp,
				 struct ib_gid *gid ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) qp;
	( void ) gid;
}

/***************************************************************************
 *
 * MAD operations
 *
 ***************************************************************************
 */

/**
 * Issue management datagram
 *
 * @v ibdev		Infiniband device
 * @v mad		Management datagram
 * @v len		Length of management datagram
 * @ret rc		Return status code
 */
static int linda_mad ( struct ib_device *ibdev, struct ib_mad_hdr *mad,
		       size_t len ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) mad;
	( void ) len;
	return 0;
}

/** Linda Infiniband operations */
static struct ib_device_operations linda_ib_operations = {
	.create_cq	= linda_create_cq,
	.destroy_cq	= linda_destroy_cq,
	.create_qp	= linda_create_qp,
	.modify_qp	= linda_modify_qp,
	.destroy_qp	= linda_destroy_qp,
	.post_send	= linda_post_send,
	.post_recv	= linda_post_recv,
	.poll_cq	= linda_poll_cq,
	.poll_eq	= linda_poll_eq,
	.open		= linda_open,
	.close		= linda_close,
	.mcast_attach	= linda_mcast_attach,
	.mcast_detach	= linda_mcast_detach,
	.mad		= linda_mad,
};

/***************************************************************************
 *
 * I2C bus operations
 *
 ***************************************************************************
 */

/** Linda I2C bit to GPIO mappings */
static unsigned int linda_i2c_bits[] = {
	[I2C_BIT_SCL] = ( 1 << LINDA_GPIO_SCL ),
	[I2C_BIT_SDA] = ( 1 << LINDA_GPIO_SDA ),
};

/**
 * Read Linda I2C line status
 *
 * @v basher		Bit-bashing interface
 * @v bit_id		Bit number
 * @ret zero		Input is a logic 0
 * @ret non-zero	Input is a logic 1
 */
static int linda_i2c_read_bit ( struct bit_basher *basher,
				unsigned int bit_id ) {
	struct linda *linda =
		container_of ( basher, struct linda, i2c.basher );
	struct QIB_7220_EXTStatus extstatus;
	unsigned int status;

	DBG_DISABLE ( DBGLVL_IO );

	linda_readq ( linda, &extstatus, QIB_7220_EXTStatus_offset );
	status = ( BIT_GET ( &extstatus, GPIOIn ) & linda_i2c_bits[bit_id] );

	DBG_ENABLE ( DBGLVL_IO );

	return status;
}

/**
 * Write Linda I2C line status
 *
 * @v basher		Bit-bashing interface
 * @v bit_id		Bit number
 * @v data		Value to write
 */
static void linda_i2c_write_bit ( struct bit_basher *basher,
				  unsigned int bit_id, unsigned long data ) {
	struct linda *linda =
		container_of ( basher, struct linda, i2c.basher );
	struct QIB_7220_EXTCtrl extctrl;
	struct QIB_7220_GPIO gpioout;
	unsigned int bit = linda_i2c_bits[bit_id];
	unsigned int outputs = 0;
	unsigned int output_enables = 0;

	DBG_DISABLE ( DBGLVL_IO );

	/* Read current GPIO mask and outputs */
	linda_readq ( linda, &extctrl, QIB_7220_EXTCtrl_offset );
	linda_readq ( linda, &gpioout, QIB_7220_GPIOOut_offset );

	/* Update outputs and output enables.  I2C lines are tied
	 * high, so we always set the output to 0 and use the output
	 * enable to control the line.
	 */
	output_enables = BIT_GET ( &extctrl, GPIOOe );
	output_enables = ( ( output_enables & ~bit ) | ( ~data & bit ) );
	outputs = BIT_GET ( &gpioout, GPIO );
	outputs = ( outputs & ~bit );
	BIT_SET ( &extctrl, GPIOOe, output_enables );
	BIT_SET ( &gpioout, GPIO, outputs );

	/* Write the output enable first; that way we avoid logic
	 * hazards.
	 */
	linda_writeq ( linda, &extctrl, QIB_7220_EXTCtrl_offset );
	linda_writeq ( linda, &gpioout, QIB_7220_GPIOOut_offset );
	mb();

	DBG_ENABLE ( DBGLVL_IO );
}

/** Linda I2C bit-bashing interface operations */
static struct bit_basher_operations linda_i2c_basher_ops = {
	.read	= linda_i2c_read_bit,
	.write	= linda_i2c_write_bit,
};

/**
 * Initialise Linda I2C subsystem
 *
 * @v linda		Linda device
 * @ret rc		Return status code
 */
static int linda_init_i2c ( struct linda *linda ) {
	static int try_eeprom_address[] = { 0x51, 0x50 };
	unsigned int i;
	int rc;

	/* Initialise bus */
	if ( ( rc = init_i2c_bit_basher ( &linda->i2c,
					  &linda_i2c_basher_ops ) ) != 0 ) {
		DBGC ( linda, "Linda %p could not initialise I2C bus: %s\n",
		       linda, strerror ( rc ) );
		return rc;
	}

	/* Probe for devices */
	for ( i = 0 ; i < ( sizeof ( try_eeprom_address ) /
			    sizeof ( try_eeprom_address[0] ) ) ; i++ ) {
		init_i2c_eeprom ( &linda->eeprom, try_eeprom_address[i] );
		if ( ( rc = i2c_check_presence ( &linda->i2c.i2c,
						 &linda->eeprom ) ) == 0 ) {
			DBGC2 ( linda, "Linda %p found EEPROM at %02x\n",
				linda, try_eeprom_address[i] );
			return 0;
		}
	}

	DBGC ( linda, "Linda %p could not find EEPROM\n", linda );
	return -ENODEV;
}

/**
 * Read EEPROM parameters
 *
 * @v linda		Linda device
 * @ret rc		Return status code
 */
static int linda_read_eeprom ( struct linda *linda ) {
	struct i2c_interface *i2c = &linda->i2c.i2c;
	int rc;

	/* Read GUID */
	if ( ( rc = i2c->read ( i2c, &linda->eeprom, LINDA_EEPROM_GUID_OFFSET,
				linda->guid, sizeof ( linda->guid ) ) ) != 0 ){
		DBGC ( linda, "Linda %p could not read GUID: %s\n",
		       linda, strerror ( rc ) );
		return rc;
	}
	DBGC2 ( linda, "Linda %p has GUID %02x:%02x:%02x:%02x:"
		"%02x:%02x:%02x:%02x\n", linda, linda->guid[0],
		linda->guid[1], linda->guid[2], linda->guid[3], linda->guid[4],
		linda->guid[5], linda->guid[6], linda->guid[7] );

	/* Read serial number (debug only) */
	if ( DBG_LOG ) {
		uint8_t serial[LINDA_EEPROM_SERIAL_SIZE + 1];

		serial[ sizeof ( serial ) - 1 ] = '\0';
		if ( ( rc = i2c->read ( i2c, &linda->eeprom,
					LINDA_EEPROM_SERIAL_OFFSET, serial,
					( sizeof ( serial ) - 1 ) ) ) != 0 ) {
			DBGC ( linda, "Linda %p could not read serial: %s\n",
			       linda, strerror ( rc ) );
			return rc;
		}
		DBGC2 ( linda, "Linda %p has serial number \"%s\"\n",
			linda, serial );
	}

	return 0;
}

/***************************************************************************
 *
 * External parallel bus access
 *
 ***************************************************************************
 */

/**
 * Request ownership of the IB external parallel bus
 *
 * @v linda		Linda device
 * @ret rc		Return status code
 */
static int linda_ib_epb_request ( struct linda *linda ) {
	struct QIB_7220_ibsd_epb_access_ctrl access;
	unsigned int i;

	/* Request ownership */
	memset ( &access, 0, sizeof ( access ) );
	BIT_FILL_1 ( &access, sw_ib_epb_req, 1 );
	linda_writeq ( linda, &access, QIB_7220_ibsd_epb_access_ctrl_offset );

	/* Wait for ownership to be granted */
	for ( i = 0 ; i < LINDA_EPB_REQUEST_MAX_WAIT_US ; i++ ) {
		linda_readq ( linda, &access,
			      QIB_7220_ibsd_epb_access_ctrl_offset );
		if ( BIT_GET ( &access, sw_ib_epb_req_granted ) )
			return 0;
		udelay ( 1 );
	}

	DBGC ( linda, "Linda %p timed out waiting for IB EPB request\n",
	       linda );
	return -ETIMEDOUT;
}

/**
 * Wait for IB external parallel bus transaction to complete
 *
 * @v linda		Linda device
 * @v xact		Buffer to hold transaction result
 * @ret rc		Return status code
 */
static int linda_ib_epb_wait ( struct linda *linda,
			    struct QIB_7220_ibsd_epb_transaction_reg *xact ) {
	unsigned int i;

	/* Discard first read to allow for signals crossing clock domains */
	linda_readq ( linda, xact, QIB_7220_ibsd_epb_transaction_reg_offset );

	for ( i = 0 ; i < LINDA_EPB_XACT_MAX_WAIT_US ; i++ ) {
		linda_readq ( linda, xact,
			      QIB_7220_ibsd_epb_transaction_reg_offset );
		if ( BIT_GET ( xact, ib_epb_rdy ) ) {
			if ( BIT_GET ( xact, ib_epb_req_error ) ) {
				DBGC ( linda, "Linda %p EPB transaction "
				       "failed\n", linda );
				return -EIO;
			} else {
				return 0;
			}
		}
		udelay ( 1 );
	}

	DBGC ( linda, "Linda %p timed out waiting for IB EPB transaction\n",
	       linda );
	return -ETIMEDOUT;
}

/**
 * Release ownership of the IB external parallel bus
 *
 * @v linda		Linda device
 */
static void linda_ib_epb_release ( struct linda *linda ) {
	struct QIB_7220_ibsd_epb_access_ctrl access;

	memset ( &access, 0, sizeof ( access ) );
	BIT_FILL_1 ( &access, sw_ib_epb_req, 0 );
	linda_writeq ( linda, &access, QIB_7220_ibsd_epb_access_ctrl_offset );
}

/**
 * Read data via IB external parallel bus
 *
 * @v linda		Linda device
 * @v location		EPB location
 * @ret data		Data read, or negative error
 *
 * You must have already acquired ownership of the IB external
 * parallel bus.
 */
static int linda_ib_epb_read ( struct linda *linda, unsigned int location ) {
	struct QIB_7220_ibsd_epb_transaction_reg xact;
	unsigned int data;
	int rc;

	/* Ensure no transaction is currently in progress */
	if ( ( rc = linda_ib_epb_wait ( linda, &xact ) ) != 0 )
		return rc;

	/* Process data */
	memset ( &xact, 0, sizeof ( xact ) );
	BIT_FILL_3 ( &xact,
		     ib_epb_address, LINDA_EPB_LOC_ADDRESS ( location ),
		     ib_epb_read_write, LINDA_EPB_READ,
		     ib_epb_cs, LINDA_EPB_LOC_CS ( location ) );
	linda_writeq ( linda, &xact,
		       QIB_7220_ibsd_epb_transaction_reg_offset );

	/* Wait for transaction to complete */
	if ( ( rc = linda_ib_epb_wait ( linda, &xact ) ) != 0 )
		return rc;

	data = BIT_GET ( &xact, ib_epb_data );
	return data;
}

/**
 * Write data via IB external parallel bus
 *
 * @v linda		Linda device
 * @v location		EPB location
 * @v data		Data to write
 * @ret rc		Return status code
 *
 * You must have already acquired ownership of the IB external
 * parallel bus.
 */
static int linda_ib_epb_write ( struct linda *linda, unsigned int location,
				unsigned int data ) {
	struct QIB_7220_ibsd_epb_transaction_reg xact;
	int rc;

	/* Ensure no transaction is currently in progress */
	if ( ( rc = linda_ib_epb_wait ( linda, &xact ) ) != 0 )
		return rc;

	/* Process data */
	memset ( &xact, 0, sizeof ( xact ) );
	BIT_FILL_4 ( &xact,
		     ib_epb_data, data,
		     ib_epb_address, LINDA_EPB_LOC_ADDRESS ( location ),
		     ib_epb_read_write, LINDA_EPB_WRITE,
		     ib_epb_cs, LINDA_EPB_LOC_CS ( location ) );
	linda_writeq ( linda, &xact,
		       QIB_7220_ibsd_epb_transaction_reg_offset );

	/* Wait for transaction to complete */
	if ( ( rc = linda_ib_epb_wait ( linda, &xact ) ) != 0 )
		return rc;

	return 0;
}

/**
 * Read/modify/write EPB register
 *
 * @v linda		Linda device
 * @v cs		Chip select
 * @v channel		Channel
 * @v element		Element
 * @v reg		Register
 * @v value		Value to set
 * @v mask		Mask to apply to old value
 * @ret rc		Return status code
 */
static int linda_ib_epb_mod_reg ( struct linda *linda, unsigned int cs,
				  unsigned int channel, unsigned int element,
				  unsigned int reg, unsigned int value,
				  unsigned int mask ) {
	unsigned int location;
	int old_value;
	int rc;

	DBG_DISABLE ( DBGLVL_IO );

	/* Sanity check */
	assert ( ( value & mask ) == value );

	/* Acquire bus ownership */
	if ( ( rc = linda_ib_epb_request ( linda ) ) != 0 )
		goto out;

	/* Read existing value, if necessary */
	location = LINDA_EPB_LOC ( cs, channel, element, reg );
	if ( (~mask) & 0xff ) {
		old_value = linda_ib_epb_read ( linda, location );
		if ( old_value < 0 ) {
			rc = old_value;
			goto out_release;
		}
	} else {
		old_value = 0;
	}

	/* Update value */
	value = ( ( old_value & ~mask ) | value );
	DBGCP ( linda, "Linda %p CS %d EPB(%d,%d,%#02x) %#02x => %#02x\n",
		linda, cs, channel, element, reg, old_value, value );
	if ( ( rc = linda_ib_epb_write ( linda, location, value ) ) != 0 )
		goto out_release;

 out_release:
	/* Release bus */
	linda_ib_epb_release ( linda );
 out:
	DBG_ENABLE ( DBGLVL_IO );
	return rc;	
}

/**
 * Transfer data to/from microcontroller RAM
 *
 * @v linda		Linda device
 * @v address		Starting address
 * @v write		Data to write, or NULL
 * @v read		Data to read, or NULL
 * @v len		Length of data
 * @ret rc		Return status code
 */
static int linda_ib_epb_ram_xfer ( struct linda *linda, unsigned int address,
				   const void *write, void *read,
				   size_t len ) {
	unsigned int control;
	unsigned int address_hi;
	unsigned int address_lo;
	int data;
	int rc;

	DBG_DISABLE ( DBGLVL_IO );

	assert ( ! ( write && read ) );
	assert ( ( address % LINDA_EPB_UC_CHUNK_SIZE ) == 0 );
	assert ( ( len % LINDA_EPB_UC_CHUNK_SIZE ) == 0 );

	/* Acquire bus ownership */
	if ( ( rc = linda_ib_epb_request ( linda ) ) != 0 )
		goto out;

	/* Process data */
	while ( len ) {

		/* Reset the address for each new chunk */
		if ( ( address % LINDA_EPB_UC_CHUNK_SIZE ) == 0 ) {

			/* Write the control register */
			control = ( read ? LINDA_EPB_UC_CTL_READ :
				    LINDA_EPB_UC_CTL_WRITE );
			if ( ( rc = linda_ib_epb_write ( linda,
							 LINDA_EPB_UC_CTL,
							 control ) ) != 0 )
				break;

			/* Write the address registers */
			address_hi = ( address >> 8 );
			if ( ( rc = linda_ib_epb_write ( linda,
							 LINDA_EPB_UC_ADDR_HI,
							 address_hi ) ) != 0 )
				break;
			address_lo = ( address & 0xff );
			if ( ( rc = linda_ib_epb_write ( linda,
							 LINDA_EPB_UC_ADDR_LO,
							 address_lo ) ) != 0 )
				break;
		}

		/* Read or write the data */
		if ( read ) {
			data = linda_ib_epb_read ( linda, LINDA_EPB_UC_DATA );
			if ( data < 0 ) {
				rc = data;
				break;
			}
			*( ( uint8_t * ) read++ ) = data;
		} else {
			data = *( ( uint8_t * ) write++ );
			if ( ( rc = linda_ib_epb_write ( linda,
							 LINDA_EPB_UC_DATA,
							 data ) ) != 0 )
				break;
		}
		address++;
		len--;

		/* Reset the control byte after each chunk */
		if ( ( address % LINDA_EPB_UC_CHUNK_SIZE ) == 0 ) {
			if ( ( rc = linda_ib_epb_write ( linda,
							 LINDA_EPB_UC_CTL,
							 0 ) ) != 0 )
				break;
		}
	}

	/* Release bus */
	linda_ib_epb_release ( linda );

 out:
	DBG_ENABLE ( DBGLVL_IO );
	return rc;
}

/***************************************************************************
 *
 * RX initialisation
 *
 ***************************************************************************
 */

/**
 * Initialise RX contexts
 *
 * @v linda		Linda device
 * @ret rc		Return status code
 */
static int linda_init_rx ( struct linda *linda ) {
	struct QIB_7220_RcvCtrl rcvctrl;
	struct QIB_7220_scalar rcvegrbase;
	struct QIB_7220_scalar rcvhdrentsize;
	struct QIB_7220_scalar rcvhdrcnt;
	unsigned int portcfg;
	unsigned long egrbase;
	unsigned int eager_array_size_0;
	unsigned int eager_array_size_other;
	unsigned int ctx;

	/* Select configuration based on number of contexts */
	switch ( LINDA_NUM_CONTEXTS ) {
	case 5:
		portcfg = LINDA_PORTCFG_5CTX;
		eager_array_size_0 = LINDA_EAGER_ARRAY_SIZE_5CTX_0;
		eager_array_size_other = LINDA_EAGER_ARRAY_SIZE_5CTX_OTHER;
		break;
	case 9:
		portcfg = LINDA_PORTCFG_9CTX;
		eager_array_size_0 = LINDA_EAGER_ARRAY_SIZE_9CTX_0;
		eager_array_size_other = LINDA_EAGER_ARRAY_SIZE_9CTX_OTHER;
		break;
	case 17:
		portcfg = LINDA_PORTCFG_17CTX;
		eager_array_size_0 = LINDA_EAGER_ARRAY_SIZE_17CTX_0;
		eager_array_size_other = LINDA_EAGER_ARRAY_SIZE_17CTX_OTHER;
		break;
	default:
		linker_assert ( 0, invalid_LINDA_NUM_CONTEXTS );
		return -EINVAL;
	}

	/* Configure number of contexts */
	memset ( &rcvctrl, 0, sizeof ( rcvctrl ) );
	BIT_FILL_3 ( &rcvctrl,
		     TailUpd, 1,
		     PortCfg, portcfg,
		     RcvQPMapEnable, 1 );
	linda_writeq ( linda, &rcvctrl, QIB_7220_RcvCtrl_offset );

	/* Configure receive header buffer sizes */
	memset ( &rcvhdrcnt, 0, sizeof ( rcvhdrcnt ) );
	BIT_FILL_1 ( &rcvhdrcnt, Value, LINDA_RX_HEADER_COUNT );
	linda_writeq ( linda, &rcvhdrcnt, QIB_7220_RcvHdrCnt_offset );
	memset ( &rcvhdrentsize, 0, sizeof ( rcvhdrentsize ) );
	BIT_FILL_1 ( &rcvhdrentsize, Value, ( LINDA_RX_HEADER_SIZE >> 2 ) );
	linda_writeq ( linda, &rcvhdrentsize, QIB_7220_RcvHdrEntSize_offset );

	/* Calculate eager array start addresses for each context */
	linda_readq ( linda, &rcvegrbase, QIB_7220_RcvEgrBase_offset );
	egrbase = BIT_GET ( &rcvegrbase, Value );
	linda->context[0].eager_array = egrbase;
	linda->context[0].eager_entries = eager_array_size_0;
	egrbase += ( eager_array_size_0 * sizeof ( struct QIB_7220_RcvEgr ) );
	for ( ctx = 1 ; ctx < LINDA_NUM_CONTEXTS ; ctx++ ) {
		linda->context[ctx].eager_array = egrbase;
		linda->context[ctx].eager_entries = eager_array_size_other;
		egrbase += ( eager_array_size_other *
			     sizeof ( struct QIB_7220_RcvEgr ) );
	}
	for ( ctx = 0 ; ctx < LINDA_NUM_CONTEXTS ; ctx++ ) {
		DBGC ( linda, "Linda %p CTX %d eager array at %lx (%d "
		       "entries)\n", linda, ctx,
		       linda->context[ctx].eager_array,
		       linda->context[ctx].eager_entries );
	}

	return 0;
}

/***************************************************************************
 *
 * Infiniband SerDes initialisation
 *
 ***************************************************************************
 */

/** A Linda SerDes parameter */
struct linda_serdes_param {
	/** EPB address as constructed by LINDA_EPB_ADDRESS() */
	uint16_t address;
	/** Value to set */
	uint8_t value;
	/** Mask to apply to old value */
	uint8_t mask;
} __packed;

/** Magic "all channels" channel number */
#define LINDA_EPB_ALL_CHANNELS 31

/** End of SerDes parameter list marker */
#define LINDA_SERDES_PARAM_END { 0, 0, 0 }

/**
 * Program IB SerDes register(s)
 *
 * @v linda		Linda device
 * @v param		SerDes parameter
 * @ret rc		Return status code
 */
static int linda_set_serdes_param ( struct linda *linda,
				    struct linda_serdes_param *param ) {
	unsigned int channel;
	unsigned int channel_start;
	unsigned int channel_end;
	unsigned int element;
	unsigned int reg;
	int rc;

	/* Break down the EPB address and determine channels */
	channel = LINDA_EPB_ADDRESS_CHANNEL ( param->address );
	element = LINDA_EPB_ADDRESS_ELEMENT ( param->address );
	reg = LINDA_EPB_ADDRESS_REG ( param->address );
	if ( channel == LINDA_EPB_ALL_CHANNELS ) {
		channel_start = 0;
		channel_end = 3;
	} else {
		channel_start = channel_end = channel;
	}

	/* Modify register for each specified channel */
	for ( channel = channel_start ; channel <= channel_end ; channel++ ) {
		if ( ( rc = linda_ib_epb_mod_reg ( linda, LINDA_EPB_CS_SERDES,
						   channel, element, reg,
						   param->value,
						   param->mask ) ) != 0 )
			return rc;
	}

	return 0;
}

/**
 * Program IB SerDes registers
 *
 * @v linda		Linda device
 * @v param		SerDes parameters
 * @v count		Number of parameters
 * @ret rc		Return status code
 */
static int linda_set_serdes_params ( struct linda *linda,
				     struct linda_serdes_param *params ) {
	int rc;

	for ( ; params->mask != 0 ; params++ ){
		if ( ( rc = linda_set_serdes_param ( linda,
							 params ) ) != 0 )
			return rc;
	}

	return 0;
}

#define LINDA_DDS_VAL( amp_d, main_d, ipst_d, ipre_d,			\
		       amp_s, main_s, ipst_s, ipre_s )			\
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 9, 0x00 ),	\
	  ( ( ( amp_d & 0x1f ) << 1 ) | 1 ), 0xff },			\
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 9, 0x01 ),	\
	  ( ( ( amp_s & 0x1f ) << 1 ) | 1 ), 0xff },			\
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 9, 0x09 ),	\
	  ( ( main_d << 3 ) | 4 | ( ipre_d >> 2 ) ), 0xff },		\
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 9, 0x0a ),	\
	  ( ( main_s << 3 ) | 4 | ( ipre_s >> 2 ) ), 0xff },		\
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 9, 0x06 ),	\
	  ( ( ( ipst_d & 0xf ) << 1 ) |					\
	    ( ( ipre_d & 3 ) << 6 ) | 0x21 ), 0xff },			\
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 9, 0x07 ),	\
	  ( ( ( ipst_s & 0xf ) << 1 ) |					\
	    ( ( ipre_s & 3 ) << 6) | 0x21 ), 0xff }

/**
 * Linda SerDes default parameters
 *
 * These magic start-of-day values are taken from the Linux driver.
 */
static struct linda_serdes_param linda_serdes_defaults1[] = {
	/* RXHSCTRL0 */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 6, 0x00 ), 0xd4, 0xff },
	/* VCDL_DAC2 */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 6, 0x05 ), 0x2d, 0xff },
	/* VCDL_CTRL2 */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 6, 0x08 ), 0x03, 0x0f },
	/* START_EQ1 */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x27 ), 0x10, 0xff },
	/* START_EQ2 */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x28 ), 0x30, 0xff },
	/* BACTRL */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 6, 0x0e ), 0x40, 0xff },
	/* LDOUTCTRL1 */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x06 ), 0x04, 0xff },
	/* RXHSSTATUS */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 6, 0x0f ), 0x04, 0xff },
	/* End of this block */
	LINDA_SERDES_PARAM_END
};
static struct linda_serdes_param linda_serdes_defaults2[] = {
	/* LDOUTCTRL1 */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x06 ), 0x00, 0xff },
	/* DDS values */
	LINDA_DDS_VAL ( 31, 19, 12, 0, 29, 22, 9, 0 ),
	/* Set Rcv Eq. to Preset node */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x27 ), 0x10, 0xff },
	/* DFELTHFDR */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x08 ), 0x00, 0xff },
	/* DFELTHHDR */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x21 ), 0x00, 0xff },
	/* TLTHFDR */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x09 ), 0x02, 0xff },
	/* TLTHHDR */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x23 ), 0x02, 0xff },
	/* ZFR */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x1b ), 0x0c, 0xff },
	/* ZCNT) */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x1c ), 0x0c, 0xff },
	/* GFR */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x1e ), 0x10, 0xff },
	/* GHR */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x1f ), 0x10, 0xff },
	/* VCDL_CTRL0 toggle */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 6, 0x06 ), 0x20, 0xff },
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 6, 0x06 ), 0x00, 0xff },
	/* CMUCTRL5 */
	{ LINDA_EPB_ADDRESS (			   7, 0, 0x15 ), 0x80, 0xff },
	/* End of this block */
	LINDA_SERDES_PARAM_END
};
static struct linda_serdes_param linda_serdes_defaults3[] = {
	/* START_EQ1 */
	{ LINDA_EPB_ADDRESS ( LINDA_EPB_ALL_CHANNELS, 7, 0x27 ), 0x00, 0x38 },
	/* End of this block */
	LINDA_SERDES_PARAM_END
};

/**
 * Program the 8051 microcontroller RAM
 *
 * @v linda		Linda device
 * @ret rc		Return status code
 */
static int linda_program_8051 ( struct linda *linda ) {
	int rc;

	if ( ( rc = linda_ib_epb_ram_xfer ( linda, 0, linda_ib_fw, NULL,
					    sizeof ( linda_ib_fw ) ) ) != 0 ){
		DBGC ( linda, "Linda %p could not load IB firmware: %s\n",
		       linda, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Verify the 8051 microcontroller RAM
 *
 * @v linda		Linda device
 * @ret rc		Return status code
 */
static int linda_verify_8051 ( struct linda *linda ) {
	uint8_t verify[LINDA_EPB_UC_CHUNK_SIZE];
	unsigned int offset;
	int rc;

	for ( offset = 0 ; offset < sizeof ( linda_ib_fw );
	      offset += sizeof ( verify ) ) {
		if ( ( rc = linda_ib_epb_ram_xfer ( linda, offset,
						    NULL, verify,
						    sizeof (verify) )) != 0 ){
			DBGC ( linda, "Linda %p could not read back IB "
			       "firmware: %s\n", linda, strerror ( rc ) );
			return rc;
		}
		if ( memcmp ( ( linda_ib_fw + offset ), verify,
			      sizeof ( verify ) ) != 0 ) {
			DBGC ( linda, "Linda %p firmware verification failed "
			       "at offset %#x\n", linda, offset );
			DBGC_HDA ( linda, offset, ( linda_ib_fw + offset ),
				   sizeof ( verify ) );
			DBGC_HDA ( linda, offset, verify, sizeof ( verify ) );
			return -EIO;
		}
	}

	DBGC2 ( linda, "Linda %p firmware verified ok\n", linda );
	return 0;
}

/**
 * Use the 8051 microcontroller to trim the IB link
 *
 * @v linda		Linda device
 * @ret rc		Return status code
 */
static int linda_trim_ib ( struct linda *linda ) {
	struct QIB_7220_IBSerDesCtrl ctrl;
	struct QIB_7220_IntStatus intstatus;
	unsigned int i;
	int rc;

	/* Bring the 8051 out of reset */
	linda_readq ( linda, &ctrl, QIB_7220_IBSerDesCtrl_offset );
	BIT_SET ( &ctrl, ResetIB_uC_Core, 0 );
	linda_writeq ( linda, &ctrl, QIB_7220_IBSerDesCtrl_offset );

	/* Wait for the "trim done" signal */
	for ( i = 0 ; i < LINDA_TRIM_DONE_MAX_WAIT_MS ; i++ ) {
		linda_readq ( linda, &intstatus, QIB_7220_IntStatus_offset );
		if ( BIT_GET ( &intstatus, IBSerdesTrimDone ) ) {
			rc = 0;
			goto out_reset;
		}
		mdelay ( 1 );
	}

	DBGC ( linda, "Linda %p timed out waiting for trim done\n", linda );
	rc = -ETIMEDOUT;
 out_reset:
	/* Put the 8051 back into reset */
	BIT_SET ( &ctrl, ResetIB_uC_Core, 1 );
	linda_writeq ( linda, &ctrl, QIB_7220_IBSerDesCtrl_offset );

	return rc;
}

/**
 * Initialise the IB SerDes
 *
 * @v linda		Linda device
 * @ret rc		Return status code
 */
static int linda_init_ib_serdes ( struct linda *linda ) {
	struct QIB_7220_Control control;
	struct QIB_7220_IBCCtrl ibcctrl;
	struct QIB_7220_IBCDDRCtrl ibcddrctrl;
	struct QIB_7220_XGXSCfg xgxscfg;
	int rc;

	/* Disable link */
	linda_readq ( linda, &control, QIB_7220_Control_offset );
	BIT_SET ( &control, LinkEn, 0 );
	linda_writeq ( linda, &control, QIB_7220_Control_offset );

	/* Configure sensible defaults for IBC */
	memset ( &ibcctrl, 0, sizeof ( ibcctrl ) );
	BIT_FILL_6 ( &ibcctrl, /* Tuning values taken from Linux driver */
		     FlowCtrlPeriod, 0x03,
		     FlowCtrlWaterMark, 0x05,
		     MaxPktLen, ( ( LINDA_RX_HEADER_SIZE +
				    LINDA_RX_PAYLOAD_SIZE +
				    4 /* ICRC */ ) >> 2 ),
		     PhyerrThreshold, 0xf,
		     OverrunThreshold, 0xf,
		     CreditScale, 0x4 );
	linda_writeq ( linda, &ibcctrl, QIB_7220_IBCCtrl_offset );

	/* Force SDR only to avoid needing all the DDR tuning,
	 * Mellanox compatibiltiy hacks etc.  SDR is plenty for
	 * boot-time operation.
	 */
	linda_readq ( linda, &ibcddrctrl, QIB_7220_IBCDDRCtrl_offset );
	BIT_SET ( &ibcddrctrl, IB_ENHANCED_MODE, 0 );
	BIT_SET ( &ibcddrctrl, SD_SPEED_SDR, 1 );
	BIT_SET ( &ibcddrctrl, SD_SPEED_DDR, 0 );
	BIT_SET ( &ibcddrctrl, SD_SPEED_QDR, 0 );
	BIT_SET ( &ibcddrctrl, HRTBT_ENB, 0 );
	BIT_SET ( &ibcddrctrl, HRTBT_AUTO, 0 );
	linda_writeq ( linda, &ibcddrctrl, QIB_7220_IBCDDRCtrl_offset );

	/* Set default SerDes parameters */
	if ( ( rc = linda_set_serdes_params ( linda,
					      linda_serdes_defaults1 ) ) != 0 )
		return rc;
	udelay ( 415 ); /* Magic delay while SerDes sorts itself out */
	if ( ( rc = linda_set_serdes_params ( linda,
					      linda_serdes_defaults2 ) ) != 0 )
		return rc;

	/* Program the microcontroller RAM */
	if ( ( rc = linda_program_8051 ( linda ) ) != 0 )
		return rc;

	/* Verify the microcontroller RAM contents */
	if ( DBGLVL_LOG ) {
		if ( ( rc = linda_verify_8051 ( linda ) ) != 0 )
			return rc;
	}

	/* More SerDes tuning */
	if ( ( rc = linda_set_serdes_params ( linda,
					      linda_serdes_defaults3 ) ) != 0 )
		return rc;

	/* Use the microcontroller to trim the IB link */
	if ( ( rc = linda_trim_ib ( linda ) ) != 0 )
		return rc;

	/* Bring XGXS out of reset */
	linda_readq ( linda, &xgxscfg, QIB_7220_XGXSCfg_offset );
	BIT_SET ( &xgxscfg, tx_rx_reset, 0 );
	BIT_SET ( &xgxscfg, xcv_reset, 0 );
	linda_writeq ( linda, &xgxscfg, QIB_7220_XGXSCfg_offset );

	return rc;
}

/***************************************************************************
 *
 * Context 0 ("kernel" context)
 *
 ***************************************************************************
 */

/**
 * Refill context 0 receive ring
 *
 * @v ibdev		Infiniband device
 */
static void linda_kctx_refill_recv ( struct ib_device *ibdev ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	struct io_buffer *iobuf;
	int rc;

	while ( linda->kernel_recv_fill < LINDA_KCTX_NUM_SEND_WQES ) {

		/* Allocate I/O buffer */
		iobuf = alloc_iob ( LINDA_RX_PAYLOAD_SIZE );
		if ( ! iobuf ) {
			/* Non-fatal; we will refill on next attempt */
			return;
		}

		/* Post I/O buffer */
		if ( ( rc = ib_post_recv ( ibdev, linda->kernel_qp,
					   iobuf ) ) != 0 ) {
			DBGC ( linda, "Linda %p KCTX could not refill: %s\n",
			       linda, strerror ( rc ) );
			free_iob ( iobuf );
			/* Give up */
			return;
		}

		linda->kernel_recv_fill++;
	}
}

/**
 * Complete send in context 0
 *
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v completion	Completion
 * @v iobuf		I/O buffer
 */
static void linda_kctx_complete_send ( struct ib_device *ibdev,
				       struct ib_queue_pair *qp,
				       struct ib_completion *completion,
				       struct io_buffer *iobuf ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) qp;
	( void ) completion;
	( void ) iobuf;
}

/**
 * Complete receive in context 0
 *
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v completion	Completion
 * @v iobuf		I/O buffer
 */
static void linda_kctx_complete_recv ( struct ib_device *ibdev,
				       struct ib_queue_pair *qp __unused,
				       struct ib_completion *completion,
				       struct io_buffer *iobuf ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	DBGC ( linda, "Linda %p KCTX RX %zd bytes%s\n",
	       linda, completion->len,
	       ( completion->syndrome ? " [err]" : "" ) );
	iob_put ( iobuf, completion->len );
	DBGC_HDA ( linda, 0, iobuf->data, iob_len ( iobuf ) );

	free_iob ( iobuf );

	/* Reduce receive ring fill level */
	linda->kernel_recv_fill--;
}

/**
 * Create context 0
 *
 * @v ibdev		Infiniband device
 * @ret rc		Return status code
 */
static int linda_create_kctx ( struct ib_device *ibdev ) {
	struct linda *linda = ib_get_drvdata ( ibdev );
	int rc;

	/* Create completion queue */
	linda->kernel_cq = ib_create_cq ( ibdev, 0,
					  linda_kctx_complete_send,
					  linda_kctx_complete_recv );
	if ( ! linda->kernel_cq ) {
		rc = -ENOMEM;
		goto err_create_cq;
	}

	/* Create queue pair */
	linda->kernel_qp = ib_create_qp ( ibdev, LINDA_KCTX_NUM_SEND_WQES,
					  linda->kernel_cq,
					  LINDA_KCTX_NUM_RECV_WQES,
					  linda->kernel_cq, 0 );
	if ( ! linda->kernel_qp ) {
		rc = -ENOMEM;
		goto err_create_qp;
	}
	/* If this isn't context 0, we're screwed */
	assert ( linda_qpn_to_ctx ( linda->kernel_qp->qpn ) == 0 );

	/* Fill receive ring */
	linda_kctx_refill_recv ( ibdev );
	return 0;

	ib_destroy_qp ( ibdev, linda->kernel_qp );
 err_create_qp:
	ib_destroy_cq ( ibdev, linda->kernel_cq );
 err_create_cq:
	return rc;
}

/**
 * Destroy context 0
 *
 * @v ibdev		Infiniband device
 */
static void linda_destroy_kctx ( struct ib_device *ibdev ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	ib_destroy_qp ( ibdev, linda->kernel_qp );
	ib_destroy_cq ( ibdev, linda->kernel_cq );
}

/**
 * Poll context 0
 *
 * @v ibdev		Infiniband device
 */
static void linda_poll_kctx ( struct ib_device *ibdev ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	/* Poll the kernel completion queue */
	ib_poll_cq ( ibdev, linda->kernel_cq );

	/* Refill the receive ring */
	linda_kctx_refill_recv ( ibdev );
}

/***************************************************************************
 *
 * PCI layer interface
 *
 ***************************************************************************
 */

/**
 * Probe PCI device
 *
 * @v pci		PCI device
 * @v id		PCI ID
 * @ret rc		Return status code
 */
static int linda_probe ( struct pci_device *pci,
			 const struct pci_device_id *id __unused ) {
	struct ib_device *ibdev;
	struct linda *linda;
	struct QIB_7220_Revision revision;
	int rc;

	/* Allocate Infiniband device */
	ibdev = alloc_ibdev ( sizeof ( *linda ) );
	if ( ! ibdev ) {
		rc = -ENOMEM;
		goto err_alloc_ibdev;
	}
	pci_set_drvdata ( pci, ibdev );
	linda = ib_get_drvdata ( ibdev );
	ibdev->op = &linda_ib_operations;
	ibdev->dev = &pci->dev;

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Get PCI BARs */
	linda->regs = ioremap ( pci->membase, LINDA_BAR0_SIZE );
	DBGC2 ( linda, "Linda %p has BAR at %08lx\n", linda, pci->membase );

	/* Print some general data */
	linda_readq ( linda, &revision, QIB_7220_Revision_offset );
	DBGC2 ( linda, "Linda %p board %02lx v%ld.%ld.%ld.%ld\n", linda,
		BIT_GET ( &revision, BoardID ),
		BIT_GET ( &revision, R_SW ),
		BIT_GET ( &revision, R_Arch ),
		BIT_GET ( &revision, R_ChipRevMajor ),
		BIT_GET ( &revision, R_ChipRevMinor ) );

	/* Initialise I2C subsystem */
	if ( ( rc = linda_init_i2c ( linda ) ) != 0 )
		goto err_init_i2c;

	/* Read EEPROM parameters */
	if ( ( rc = linda_read_eeprom ( linda ) ) != 0 )
		goto err_read_eeprom;

	/* Initialise receive datapath */
	if ( ( rc = linda_init_rx ( linda ) ) != 0 )
		goto err_init_rx;

	/* Initialise the IB SerDes */
	if ( ( rc = linda_init_ib_serdes ( linda ) ) != 0 )
		goto err_init_ib_serdes;

	/* Create the kernel context */
	if ( ( rc = linda_create_kctx ( ibdev ) ) != 0 )
		goto err_create_kctx;

	/* Register Infiniband device */
	if ( ( rc = register_ibdev ( ibdev ) ) != 0 ) {
		DBGC ( linda, "Linda %p could not register IB "
		       "device: %s\n", linda, strerror ( rc ) );
		goto err_register_ibdev;
	}
	
	return 0;

	unregister_ibdev ( ibdev );
 err_register_ibdev:
	linda_destroy_kctx ( ibdev );
 err_create_kctx:
 err_init_rx:
 err_init_ib_serdes:
 err_read_eeprom:
 err_init_i2c:
	ibdev_put ( ibdev );
 err_alloc_ibdev:
	return rc;
}

/**
 * Remove PCI device
 *
 * @v pci		PCI device
 */
static void linda_remove ( struct pci_device *pci ) {
	struct ib_device *ibdev = pci_get_drvdata ( pci );

	linda_destroy_kctx ( ibdev );
	unregister_ibdev ( ibdev );
	ibdev_put ( ibdev );
}

static struct pci_device_id linda_nics[] = {
	PCI_ROM ( 0x1077, 0x7220, "iba7220", "QLE7240/7280 HCA driver" ),
};

struct pci_driver linda_driver __pci_driver = {
	.ids = linda_nics,
	.id_count = ( sizeof ( linda_nics ) / sizeof ( linda_nics[0] ) ),
	.probe = linda_probe,
	.remove = linda_remove,
};
