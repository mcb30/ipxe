/*
 * Copyright (C) 2009 Fen Systems Ltd <mbrown@fensystems.co.uk>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

FILE_LICENCE ( BSD2 );

#include <stdlib.h>
#include <errno.h>
#include <gpxe/iobuf.h>
#include <gpxe/srp.h>
#include <gpxe/infiniband.h>
#include <gpxe/ib_cm.h>
#include <gpxe/ib_srp.h>

/**
 * @file
 *
 * SCSI RDMA Protocol over Infiniband
 *
 */

/** SRP number of send WQEs
 *
 * This is a policy decision.
 */
#define IB_SRP_NUM_SEND_WQES 4

/** SRP number of receive WQEs
 *
 * This is a policy decision.
 */
#define IB_SRP_NUM_RECV_WQES 2

/** SRP number of completion queue entries
 *
 * This is a policy decision
 */
#define IB_SRP_NUM_CQES 8

static void ib_srp_cm_notify ( struct ib_queue_pair *qp, int rc,
			       void *private_data,
			       size_t private_data_len ) {
	struct io_buffer *iobuf;
	struct ib_srp_device *ib_srp = ib_qp_get_ownerdata ( qp );
	struct srp_device *srp = ib_srp->srp;

	/* Flag as connected or disconnected */
	ib_srp->qp_connected = ( rc == 0 );
	DBGC ( ib_srp, "IBSRP %p %s\n", ib_srp,
	       ( ib_srp->qp_connected ? "connected" : "disconnected" ));

	iobuf = alloc_iob ( private_data_len );
	if ( ! iobuf ) {
		DBGC ( ib_srp, "IBSRP %p could not allocate CM iobuf\n",
		       ib_srp );
		return;
	}
	memcpy ( iob_put ( iobuf, private_data_len ),
		 private_data, private_data_len );

	srp->recv ( srp, iobuf );
}

static void ib_srp_complete_recv ( struct ib_device *ibdev __unused,
				   struct ib_queue_pair *qp,
				   struct ib_address_vector *av __unused,
				   struct io_buffer *iobuf, int rc ) {
	struct ib_srp_device *ib_srp = ib_qp_get_ownerdata ( qp );
	struct srp_device *srp = ib_srp->srp;

	DBGC2 ( ib_srp, "IBSRP %p receive (%s):\n",
		ib_srp, strerror ( rc ) );
	DBGC2_HDA ( ib_srp, 0, iobuf->data, iob_len ( iobuf ) );

	if ( rc != 0 ) {
		DBGC ( ib_srp, "IBSRP %p RX error: %s\n",
		       ib_srp, strerror ( rc ) );
		goto out;
	}

	srp->recv ( srp, iob_disown ( iobuf ) );

 out:
	free_iob ( iobuf );
}

static int ib_srp_send ( struct srp_device *srp, struct io_buffer *iobuf ) {
	struct ib_srp_device *ib_srp =
		container_of ( srp->backend, struct ib_srp_device, refcnt );
	int rc;

	if ( ib_srp->qp_connected ) {

		/* Already connected; send IU via QP */
		if ( ( rc = ib_post_send ( ib_srp->ibdev, ib_srp->qp,
					   NULL, iobuf ) ) != 0 ) {
			DBGC ( ib_srp, "IBSRP %p could not send IU: %s\n",
			       ib_srp, strerror ( rc ) );
			goto err;
		}

	} else {

		/* Not yet connected; send IU via CM connection request */
		if ( ( rc = ib_cm_connect ( ib_srp->qp, &ib_srp->target.dgid,
					    &ib_srp->target.service_id,
					    iobuf->data, iob_len ( iobuf ),
					    ib_srp_cm_notify ) ) != 0 ) {
			DBGC ( ib_srp, "IBSRP %p could not connect: %s\n",
			       ib_srp, strerror ( rc ) );
			goto err;
		}
		free_iob ( iobuf );
	}

	return 0;

 err:
	free_iob ( iobuf );
	return rc;
}

/** Infiniband SRP completion operations */
static struct ib_completion_queue_operations ib_srp_completion_ops = {
	.complete_recv = ib_srp_complete_recv,
};

struct ib_srp_root_path_component {
	const char *prefix;
	size_t offset;
	size_t len;
};
#define IB_SRP_ROOT_PATH_COMPONENT( _name ) {				\
	.prefix = # _name "=",						\
	.offset = offsetof ( struct ib_srp_target, _name ),		\
	.len = sizeof ( ( ( struct ib_srp_target * ) NULL )->_name ),	\
	}

static struct ib_srp_root_path_component ib_srp_rp_comp[] = {
	IB_SRP_ROOT_PATH_COMPONENT ( id_ext ),
	IB_SRP_ROOT_PATH_COMPONENT ( ioc_guid ),
	IB_SRP_ROOT_PATH_COMPONENT ( dgid ),
	IB_SRP_ROOT_PATH_COMPONENT ( pkey ),
	IB_SRP_ROOT_PATH_COMPONENT ( service_id ),
};

static int ib_srp_parse_rp_comp ( struct ib_srp_device *ib_srp,
				  const char *root_path,
				  const char **root_path_next ) {
	struct ib_srp_root_path_component *rp_comp;
	unsigned int i;
	size_t prefix_len;
	const char *value;
	uint8_t *data;
	size_t data_len;
	char buf[3];
	char *buf_end;

	for ( i = 0 ; i < ( sizeof ( ib_srp_rp_comp ) /
			    sizeof ( ib_srp_rp_comp[0] ) ) ; i++ ) {
		rp_comp = &ib_srp_rp_comp[i];
		prefix_len = strlen ( rp_comp->prefix );
		if ( strncmp ( root_path, rp_comp->prefix, prefix_len ) != 0 )
			continue;
		value = &root_path[prefix_len];
		if ( strlen ( value ) < ( 2 * rp_comp->len ) ) {
			DBGC ( ib_srp, "IBSRP %p underlength value in "
			       "\"%s\"\n", ib_srp, root_path );
			return -EINVAL;
		}
		data = ( ( ( uint8_t * ) &ib_srp->target ) + rp_comp->offset );
		for ( data_len = rp_comp->len ; data_len ; data_len-- ) {
			memcpy ( buf, value, 2 );
			buf[2] = '\0';
			*data = strtoul ( buf, &buf_end, 16 );
			if ( buf_end != &buf[2] ) {
				DBGC ( ib_srp, "IBSRP %p invalid value in "
				       "\"%s\"\n", ib_srp, root_path );
				return -EINVAL;
			}
			data++;
			value += 2;
		}
		*root_path_next = value;
		return 0;
	}

	DBGC ( ib_srp, "IBSRP %p no match for root path component \"%s\"\n",
	       ib_srp, root_path );
	return -EINVAL;
}

static int ib_srp_parse_root_path ( struct ib_srp_device *ib_srp,
				    const char *root_path ) {
	const char *rp;
	int rc;

	if ( strncmp ( root_path, "ib_srp:", 7 ) != 0 ) {
		DBGC ( ib_srp, "IBSRP %p invalid root path prefix in \"%s\"\n",
		       ib_srp, root_path );
		return -EINVAL;
	}
	rp = ( root_path + 7 );

	while ( 1 ) {
		if ( ( rc = ib_srp_parse_rp_comp ( ib_srp, rp, &rp ) ) != 0 )
			return rc;
		if ( *rp == '\0' )
			break;
		if ( *rp == ',' ) {
			rp++;
			continue;
		}
		DBGC ( ib_srp, "IBSRP %p invalid separator '%c' in \"%s\"\n",
		       ib_srp, *rp, root_path );
		return -EINVAL;
	}

	return 0;
}

int ib_srp_attach ( struct scsi_device *scsi,
		    struct ib_device *ibdev,
		    const char *root_path __unused ) {
	struct ib_srp_device *ib_srp;
	struct srp_device *srp;
	struct ib_srp_initiator_port_id *initiator_port_id;
	struct ib_srp_target_port_id *target_port_id;
	int rc;

	/* Allocate and initialise structure */
	ib_srp = zalloc ( sizeof ( *ib_srp ) );
	if ( ! ib_srp ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	ib_srp->ibdev = ibdev;

	/* Parse root path */
	if ( ( rc = ib_srp_parse_root_path ( ib_srp, root_path ) ) != 0 )
		goto err_parse_root_path;

	/* Create completion queue */
	ib_srp->cq = ib_create_cq ( ibdev, IB_SRP_NUM_CQES,
				    &ib_srp_completion_ops );
	if ( ! ib_srp->cq ) {
		DBGC ( ib_srp, "IBSRP %p could not allocate completion "
		       "queue\n", ib_srp );
		rc = -ENOMEM;
		goto err_create_cq;
	}

	/* Create queue pair */
	ib_srp->qp = ib_create_qp ( ibdev, IB_QPT_RC,
				    IB_SRP_NUM_SEND_WQES, ib_srp->cq,
				    IB_SRP_NUM_RECV_WQES, ib_srp->cq );
	if ( ! ib_srp->qp ) {
		DBGC ( ib_srp, "IBSRP %p could not allocate queue pair\n",
		       ib_srp );
		rc = -ENOMEM;
		goto err_create_qp;
	}
	ib_qp_set_ownerdata ( ib_srp->qp, ib_srp );
	DBGC ( ib_srp, "IBSRP %p using QPN %lx\n", ib_srp, ib_srp->qp->qpn );

	if ( ( rc = srp_attach ( scsi ) ) != 0 )
		goto err_srp_attach;
	srp = container_of ( scsi->backend, struct srp_device, refcnt );
	ib_srp->srp = srp;
	initiator_port_id = ( ( struct ib_srp_initiator_port_id * )
			      &srp->initiator_port_id );
	target_port_id = ( ( struct ib_srp_target_port_id * )
			   &srp->target_port_id );
	ib_get_hca_info ( ibdev, &initiator_port_id->hca_guid );
	memcpy ( &target_port_id->id_ext, &ib_srp->target.id_ext,
		 sizeof ( target_port_id->id_ext ) );
	memcpy ( &target_port_id->ioc_guid, &ib_srp->target.ioc_guid,
		 sizeof ( target_port_id->ioc_guid ) );
	srp->memory_handle = ibdev->rdma_key;
	srp->send = ib_srp_send;
	srp->backend = ref_get ( &ib_srp->refcnt );
	ref_put ( &ib_srp->refcnt );
	return 0;

	srp_detach ( scsi );
 err_srp_attach:
	ib_destroy_qp ( ibdev, ib_srp->qp );
 err_create_qp:
	ib_destroy_cq ( ibdev, ib_srp->cq );
 err_create_cq:
 err_parse_root_path:
	ref_put ( &ib_srp->refcnt );
 err_alloc:
	return rc;
}

void ib_srp_detach ( struct scsi_device *scsi ) {
	struct srp_device *srp =
		container_of ( scsi->backend, struct srp_device, refcnt );
	struct ib_srp_device *ib_srp =
		container_of ( srp->backend, struct ib_srp_device, refcnt );
	struct ib_device *ibdev = ib_srp->ibdev;

	srp_detach ( scsi );
	ib_destroy_qp ( ibdev, ib_srp->qp );
	ib_destroy_cq ( ibdev, ib_srp->cq );
	ref_put ( &ib_srp->refcnt );
}
