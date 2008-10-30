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
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <byteswap.h>
#include <gpxe/infiniband.h>
#include <gpxe/iobuf.h>
#include <gpxe/process.h>
#include <gpxe/ib_sma.h>

/**
 * @file
 *
 * Infiniband Subnet Management Agent
 *
 */

/**
 * Get node information
 *
 * @v sma		Subnet management agent
 * @v get		Attribute to get
 */
static void ib_sma_get_node_info ( struct ib_sma *sma,
				   union ib_smp_data *get ) {
	struct ib_device *ibdev = sma->ibdev;
	struct ib_node_info *node_info = &get->node_info;
	struct ib_device *tmp;

	memset ( node_info, 0, sizeof ( *node_info ) );
	node_info->base_version = IB_MGMT_BASE_VERSION;
	node_info->class_version = IB_SMP_CLASS_VERSION;
	node_info->node_type = IB_NODE_TYPE_HCA;
	/* Search for IB devices with the same physical device to
	 * identify port count and a suitable Node GUID.
	 */
	for_each_ibdev ( tmp ) {
		if ( tmp->dev != ibdev->dev )
			continue;
		if ( node_info->num_ports == 0 ) {
			memcpy ( node_info->node_guid,
				 &tmp->port_gid.u.half[1],
				 sizeof ( node_info->node_guid ) );
		}
		node_info->num_ports++;
	}
	memcpy ( node_info->port_guid, &ibdev->port_gid.u.half[1],
		 sizeof ( node_info->port_guid ) );
	node_info->partition_cap = htons ( 1 );
	node_info->local_port_num = ibdev->port;
}

/**
 * Get GUID information
 *
 * @v sma		Subnet management agent
 * @v get		Attribute to get
 */
static void ib_sma_get_guid_info ( struct ib_sma *sma,
				   union ib_smp_data *get ) {
	struct ib_device *ibdev = sma->ibdev;
	struct ib_guid_info *guid_info = &get->guid_info;

	memset ( guid_info, 0, sizeof ( *guid_info ) );
	memcpy ( guid_info->guid[0], &ibdev->port_gid.u.half[1],
		 sizeof ( guid_info->guid[0] ) );
}

/**
 * Get port information
 *
 * @v sma		Subnet management agent
 * @v get		Attribute to get
 */
static void ib_sma_get_port_info ( struct ib_sma *sma,
				   union ib_smp_data *get ) {
	struct ib_device *ibdev = sma->ibdev;
	struct ib_port_info *port_info = &get->port_info;

	memset ( port_info, 0, sizeof ( *port_info ) );
	// hack
	port_info->port_state__link_speed_supported = ibdev->port_state;
	memcpy ( port_info->gid_prefix, &ibdev->port_gid.u.half[0],
		 sizeof ( port_info->gid_prefix ) );
}

/**
 * Set port information
 *
 * @v sma		Subnet management agent
 * @v set		Attribute to set
 * @ret rc		Return status code
 */
static int ib_sma_set_port_info ( struct ib_sma *sma,
				  const union ib_smp_data *set ) {
	struct ib_device *ibdev = sma->ibdev;
	const struct ib_port_info *port_info = &set->port_info;

	if ( ! sma->op->set_port_info )
		return -ENOTSUP;

	return sma->op->set_port_info ( ibdev, port_info );
}

/**
 * Get partition key table
 *
 * @v sma		Subnet management agent
 * @v get		Attribute to get
 */
static void ib_sma_get_pkey_table ( struct ib_sma *sma,
				    union ib_smp_data *get ) {
	struct ib_device *ibdev = sma->ibdev;
	struct ib_pkey_table *pkey_table = &get->pkey_table;

	memset ( pkey_table, 0, sizeof ( *pkey_table ) );
	pkey_table->pkey[0] = htons ( ibdev->pkey );
}

/**
 * Set partition key table
 *
 * @v sma		Subnet management agent
 * @v set		Attribute to set
 */
static int ib_sma_set_pkey_table ( struct ib_sma *sma,
				   const union ib_smp_data *get ) {
	struct ib_device *ibdev = sma->ibdev;
	const struct ib_pkey_table *pkey_table = &get->pkey_table;

	ibdev->pkey = ntohs ( pkey_table->pkey[0] );
	return 0;
}

/** An attribute handler */
struct ib_sma_handler {
	/** Attribute (in network byte order) */
	uint16_t attr_id;
	/** Get attribute
	 *
	 * @v sma	Subnet management agent
	 * @v get	Attribute to get
	 * @ret rc	Return status code
	 */
	void ( * get ) ( struct ib_sma *sma, union ib_smp_data *get );
	/** Set attribute
	 *
	 * @v sma	Subnet management agent
	 * @v set	Attribute to set
	 * @ret rc	Return status code
	 */
	int ( * set ) ( struct ib_sma *sma, const union ib_smp_data *set );
};

/** List of attribute handlers */
static struct ib_sma_handler ib_sma_handlers[] = {
	{ htons ( IB_SMP_ATTR_NODE_INFO ),
	  ib_sma_get_node_info, NULL },
	{ htons ( IB_SMP_ATTR_GUID_INFO ),
	  ib_sma_get_guid_info, NULL },
	{ htons ( IB_SMP_ATTR_PORT_INFO ),
	  ib_sma_get_port_info, ib_sma_set_port_info },
	{ htons ( IB_SMP_ATTR_PKEY_TABLE ),
	  ib_sma_get_pkey_table, ib_sma_set_pkey_table },
};

/**
 * Identify attribute handler
 *
 * @v attr_id		Attribute ID (in network byte order)
 * @ret handler		Attribute handler (or NULL)
 */
static struct ib_sma_handler * ib_sma_handler ( uint16_t attr_id ) {
	struct ib_sma_handler *handler;
	unsigned int i;

	for ( i = 0 ; i < ( sizeof ( ib_sma_handlers ) /
			    sizeof ( ib_sma_handlers[0] ) ) ; i++ ) {
		handler = &ib_sma_handlers[i];
		if ( handler->attr_id == attr_id )
			return handler;
	}

	return NULL;
}

/**
 * Respond to management datagram
 *
 * @v sma		Subnet management agent
 * @v mad		Management datagram
 * @ret rc		Return status code
 */
static int ib_sma_mad ( struct ib_sma *sma, union ib_mad *mad ) {
	struct ib_mad_hdr *hdr = &mad->hdr;
	struct ib_mad_smp *smp = &mad->smp;
	struct ib_sma_handler *handler = NULL;
	int rc;

	DBGC ( sma, "SMA %p received SMP with bv=%02x mc=%02x cv=%02x "
	       "meth=%02x attr=%04x mod=%08lx\n", sma, hdr->base_version,
	       hdr->mgmt_class, hdr->class_version, hdr->method,
	       ntohs ( hdr->attr_id ), hdr->attr_mod );
	DBGC2_HDA ( sma, 0, smp, sizeof ( *smp ) );

	/* Sanity checks */
	if ( hdr->base_version != IB_MGMT_BASE_VERSION ) {
		DBGC ( sma, "SMA %p unsupported base version %x\n",
		       sma, hdr->base_version );
		hdr->status = htons ( IB_MGMT_STATUS_BAD_VERSION );
		goto respond_without_data;
	}
	if ( ( hdr->mgmt_class != IB_MGMT_CLASS_SUBN_DIRECTED_ROUTE ) &&
	     ( hdr->mgmt_class != IB_MGMT_CLASS_SUBN_LID_ROUTED ) ) {
		DBGC ( sma, "SMA %p unsupported management class %x\n",
		       sma, hdr->mgmt_class );
		hdr->status = htons ( IB_MGMT_STATUS_BAD_VERSION );
		goto respond_without_data;
	}
	if ( hdr->class_version != IB_SMP_CLASS_VERSION ) {
		DBGC ( sma, "SMA %p unsupported class version %x\n",
		       sma, hdr->class_version );
		hdr->status = htons ( IB_MGMT_STATUS_BAD_VERSION );
		goto respond_without_data;
	}
	if ( ( hdr->method != IB_MGMT_METHOD_GET ) &&
	     ( hdr->method != IB_MGMT_METHOD_SET ) ) {
		DBGC ( sma, "SMA %p unsupported method %x\n",
		       sma, hdr->method );
		hdr->status = htons ( IB_MGMT_STATUS_UNSUPPORTED_METHOD );
		goto respond_without_data;
	}

	/* Identify handler */
	if ( ! ( handler = ib_sma_handler ( hdr->attr_id ) ) ) {
		DBGC ( sma, "SMA %p unsupported attribute %x\n",
		       sma, ntohs ( hdr->attr_id ) );
		hdr->status = htons ( IB_MGMT_STATUS_UNSUPPORTED_METHOD_ATTR );
		goto respond_without_data;
	}

	/* Set attribute (if applicable) */
	if ( hdr->method != IB_MGMT_METHOD_SET ) {
		hdr->status = htons ( IB_MGMT_STATUS_OK );
		goto respond;
	}
	if ( ! handler->set ) {
		DBGC ( sma, "SMA %p attribute %x is unsettable\n",
		       sma, ntohs ( hdr->attr_id ) );
		hdr->status = htons ( IB_MGMT_STATUS_UNSUPPORTED_METHOD_ATTR );
		goto respond;
	}
	if ( ( rc = handler->set ( sma, &smp->smp_data ) ) != 0 ) {
		DBGC ( sma, "SMA %p could not set attribute %x: %s\n",
		       sma, ntohs ( hdr->attr_id ), strerror ( rc ) );
		hdr->status = htons ( IB_MGMT_STATUS_UNSUPPORTED_METHOD_ATTR );
		goto respond;
	}

	hdr->status = htons ( IB_MGMT_STATUS_OK );

 respond:
	/* Get attribute */
	handler->get ( sma, &smp->smp_data );

 respond_without_data:

	/* Set method to "Get Response" */
	hdr->method = IB_MGMT_METHOD_GET_RESP;

	/* For directed route SMPs, we must set the D bit */
	if ( hdr->mgmt_class == IB_MGMT_CLASS_SUBN_DIRECTED_ROUTE )
		hdr->status |= htons ( IB_SMP_STATUS_D_INBOUND );

	DBGC ( sma, "SMA %p responding with status=%04x\n",
	       sma, ntohs ( hdr->status ) );
	DBGC2_HDA ( sma, 0, smp, sizeof ( *smp ) );

	return 0;
}

/**
 * Refill SMA receive ring
 *
 * @v sma		Subnet management agent
 */
static void ib_sma_refill_recv ( struct ib_sma *sma ) {
	struct ib_device *ibdev = sma->ibdev;
	struct io_buffer *iobuf;
	int rc;

	while ( sma->qp->recv.fill < IB_SMA_NUM_SEND_WQES ) {

		/* Allocate I/O buffer */
		iobuf = alloc_iob ( IB_SMA_PAYLOAD_LEN );
		if ( ! iobuf ) {
			/* Non-fatal; we will refill on next attempt */
			return;
		}

		/* Post I/O buffer */
		if ( ( rc = ib_post_recv ( ibdev, sma->qp, iobuf ) ) != 0 ) {
			DBGC ( sma, "SMA %p could not refill: %s\n",
			       sma, strerror ( rc ) );
			free_iob ( iobuf );
			/* Give up */
			return;
		}
	}
}

/**
 * Complete SMA send
 *
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v iobuf		I/O buffer
 * @v rc		Completion status code
 */
static void ib_sma_complete_send ( struct ib_device *ibdev __unused,
				   struct ib_queue_pair *qp,
				   struct io_buffer *iobuf, int rc ) {
	struct ib_sma *sma = ib_qp_get_ownerdata ( qp );

	if ( rc != 0 ) {
		DBGC ( sma, "SMA %p send completion error: %s\n",
		       sma, strerror ( rc ) );
	}
	free_iob ( iobuf );
}

/**
 * Complete SMA receive
 *
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v rqp		Remote queue pair
 * @v iobuf		I/O buffer
 * @v rc		Completion status code
 */
static void ib_sma_complete_recv ( struct ib_device *ibdev,
				   struct ib_queue_pair *qp,
				   struct ib_remote_queue_pair *rqp,
				   struct io_buffer *iobuf, int rc ) {
	struct ib_sma *sma = ib_qp_get_ownerdata ( qp );
	union ib_mad *mad;

	/* Ignore errors */
	if ( rc != 0 ) {
		DBGC ( sma, "SMA %p RX error: %s\n", sma, strerror ( rc ) );
		goto err;
	}

	/* Sanity check */
	if ( iob_len ( iobuf ) < sizeof ( *mad ) ) {
		DBGC ( sma, "SMA %p RX too short (%zd bytes)\n",
		       sma, iob_len ( iobuf ) );
		goto err;
	}
	mad = iobuf->data;

	/* Construct MAD response */
	if ( ( rc = ib_sma_mad ( sma, mad ) ) != 0 ) {
		DBGC ( sma, "SMA %p could not construct MAD response: %s\n",
		       sma, strerror ( rc ) );
		goto err;
	}

	/* Send MAD response */
	if ( ( rc = ib_post_send ( ibdev, qp, rqp, iobuf ) ) != 0 ) {
		DBGC ( sma, "SMA %p could not send MAD response: %s\n",
		       sma, strerror ( rc ) );
		goto err;
	}

	return;

 err:
	free_iob ( iobuf );
}

/** SMA completion operations */
static struct ib_completion_queue_operations ib_sma_completion_ops = {
	.complete_send = ib_sma_complete_send,
	.complete_recv = ib_sma_complete_recv,
};

/**
 * Poll SMA
 *
 * @v process		Process
 */
static void ib_sma_step ( struct process *process ) {
	struct ib_sma *sma =
		container_of ( process, struct ib_sma, poll );
	struct ib_device *ibdev = sma->ibdev;

	/* Poll the kernel completion queue */
	ib_poll_cq ( ibdev, sma->cq );

	/* Refill the receive ring */
	ib_sma_refill_recv ( sma );
}

/**
 * Create SMA
 *
 * @v sma		Subnet management agent
 * @v ibdev		Infiniband device
 * @v op		Subnet management operations
 * @ret rc		Return status code
 */
int ib_create_sma ( struct ib_sma *sma, struct ib_device *ibdev,
		    struct ib_sma_operations *op ) {
	int rc;

	/* Initialise fields */
	memset ( sma, 0, sizeof ( *sma ) );
	sma->ibdev = ibdev;
	sma->op = op;
	process_init ( &sma->poll, ib_sma_step, &ibdev->refcnt );

	/* Create completion queue */
	sma->cq = ib_create_cq ( ibdev, 0, &ib_sma_completion_ops );
	if ( ! sma->cq ) {
		rc = -ENOMEM;
		goto err_create_cq;
	}

	/* Create queue pair */
	sma->qp = ib_create_qp ( ibdev, IB_SMA_NUM_SEND_WQES, sma->cq,
				 IB_SMA_NUM_RECV_WQES, sma->cq, 0 );
	if ( ! sma->qp ) {
		rc = -ENOMEM;
		goto err_create_qp;
	}
	ib_qp_set_ownerdata ( sma->qp, sma );

	/* Fill receive ring */
	ib_sma_refill_recv ( sma );
	return 0;

	ib_destroy_qp ( ibdev, sma->qp );
 err_create_qp:
	ib_destroy_cq ( ibdev, sma->cq );
 err_create_cq:
	process_del ( &sma->poll );
	return rc;
}

/**
 * Destroy SMA
 *
 * @v sma		Subnet management agent
 */
void ib_destroy_sma ( struct ib_sma *sma ) {
	struct ib_device *ibdev = sma->ibdev;

	ib_destroy_qp ( ibdev, sma->qp );
	ib_destroy_cq ( ibdev, sma->cq );
	process_del ( &sma->poll );
}
