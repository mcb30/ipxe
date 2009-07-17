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
#include <string.h>
#include <errno.h>
#include <gpxe/scsi.h>
#include <gpxe/srp.h>

/**
 * @file
 *
 * SCSI RDMA Protocol
 *
 */

static void srp_scsi_done ( struct srp_device *srp, int rc ) {

	if ( srp->command )
		srp->command->rc = rc;
}

static int srp_send ( struct srp_device *srp, struct io_buffer *iobuf ) {
	struct srp_common *common = iobuf->data;
	int rc;

	DBGC2 ( srp, "SRP %p TX IU tag %08x%08x type %02x\n",
		srp, ntohl ( common->tag.dwords[0] ),
		ntohl ( common->tag.dwords[1] ), common->type );
	DBGC2_HDA ( srp, 0, iobuf->data, iob_len ( iobuf ) );

	/** Send the IU */
	if ( ( rc = srp->send ( srp, iob_disown ( iobuf ) ) ) != 0 ) {
		DBGC ( srp, "SRP %p could not send IU type %02x: %s\n",
		       srp, common->type, strerror ( rc ) );
		return rc;
	}

	return 0;
}

static int srp_request ( struct srp_device *srp, struct io_buffer *iobuf ) {
	struct srp_common *common = iobuf->data;
	static unsigned int srp_tag = 0;

	/** Assign a tag */
	common->tag.dwords[1] = htonl ( ++srp_tag );

	return srp_send ( srp, iobuf );
}

static int srp_cmd ( struct srp_device *srp ) {
	struct io_buffer *iobuf;
	struct srp_cmd *cmd;
	struct srp_memory_descriptor *data_out;
	struct srp_memory_descriptor *data_in;

	/* Allocate I/O buffer */
	iobuf = alloc_iob ( SRP_MAX_I_T_IU_LEN );
	if ( ! iobuf )
		return -ENOMEM;

	/* Construct base portion */
	cmd = iob_put ( iobuf, sizeof ( *cmd ) );
	memset ( cmd, 0, sizeof ( *cmd ) );
	cmd->type = SRP_CMD;
	cmd->lun = srp->lun;
	memcpy ( &cmd->cdb, &srp->command->cdb, sizeof ( cmd->cdb ) );

	/* Construct data-out descriptor, if present */
	if ( srp->command->data_out ) {
		cmd->data_buffer_formats |= SRP_CMD_DO_FMT_DIRECT;
		data_out = iob_put ( iobuf, sizeof ( *data_out ) );
		data_out->address =
		    cpu_to_be64 ( user_to_phys ( srp->command->data_out, 0 ) );
		data_out->handle = ntohl ( srp->memory_handle );
		data_out->len = ntohl ( srp->command->data_out_len );
	}

	/* Construct data-in descriptor, if present */
	if ( srp->command->data_in ) {
		cmd->data_buffer_formats |= SRP_CMD_DI_FMT_DIRECT;
		data_in = iob_put ( iobuf, sizeof ( *data_in ) );
		data_in->address =
		     cpu_to_be64 ( user_to_phys ( srp->command->data_in, 0 ) );
		data_in->handle = ntohl ( srp->memory_handle );
		data_in->len = ntohl ( srp->command->data_in_len );
	}

	return srp_request ( srp, iobuf );
}

static int srp_rsp ( struct srp_device *srp, struct srp_rsp *rsp ) {
	int rc;

	if ( rsp->status == 0 ) {
		rc = 0;
	} else {
		rc = -EIO;
		DBGC ( srp, "SRP %p response status %02x\n",
		       srp, rsp->status );
		if ( srp_rsp_sense_data ( rsp ) ) {
			DBGC ( srp, "SRP %p sense data:\n", srp );
			DBGC_HDA ( srp, 0, srp_rsp_sense_data ( rsp ),
				   srp_rsp_sense_data_len ( rsp ) );
		}
	}

	if ( rsp->valid & ( SRP_RSP_VALID_DOUNDER | SRP_RSP_VALID_DOOVER ) ) {
		DBGC ( srp, "SRP %p response data-out %srun by %#x bytes\n",
		       srp, ( ( rsp->valid & SRP_RSP_VALID_DOUNDER )
			      ? "under" : "over" ),
		       ntohl ( rsp->data_out_residual_count ) );
	}
	if ( rsp->valid & ( SRP_RSP_VALID_DIUNDER | SRP_RSP_VALID_DIOVER ) ) {
		DBGC ( srp, "SRP %p response data-in %srun by %#x bytes\n",
		       srp, ( ( rsp->valid & SRP_RSP_VALID_DIUNDER )
			      ? "under" : "over" ),
		       ntohl ( rsp->data_in_residual_count ) );
	}

	srp_scsi_done ( srp, rc );

	return 0;
}

static int srp_login_req ( struct srp_device *srp ) {
	struct io_buffer *iobuf;
	struct srp_login_req *login_req;

	/* Allocate I/O buffer */
	iobuf = alloc_iob ( sizeof ( *login_req ) );
	if ( ! iobuf )
		return -ENOMEM;

	/* Construct IU */
	login_req = iob_put ( iobuf, sizeof ( *login_req ) );
	memset ( login_req, 0, sizeof ( *login_req ) );
	login_req->type = SRP_LOGIN_REQ;
	login_req->max_i_t_iu_len = htonl ( SRP_MAX_I_T_IU_LEN );
	login_req->required_buffer_formats = SRP_LOGIN_REQ_FMT_DDBD;
	memcpy ( &login_req->initiator_port_id, &srp->initiator_port_id,
		 sizeof ( login_req->initiator_port_id ) );
	memcpy ( &login_req->target_port_id, &srp->target_port_id,
		 sizeof ( login_req->target_port_id ) );

	return srp_request ( srp, iobuf );
}

static void srp_login_rsp ( struct srp_device *srp,
			    struct srp_login_rsp *login_rsp __unused ) {

	DBGC ( srp, "SRP %p logged in\n", srp );

	/* Mark as logged in */
	srp->logged_in = 1;

	/* Issue pending command, if any */
	if ( srp->command )
		srp_cmd ( srp );
}

static int srp_recv ( struct srp_device *srp, struct io_buffer *iobuf ) {
	struct srp_common *common = iobuf->data;

	DBGC2 ( srp, "SRP %p RX IU tag %08x%08x type %02x\n",
		srp, ntohl ( common->tag.dwords[0] ),
		ntohl ( common->tag.dwords[1] ), common->type );

	switch ( common->type ) {
	case SRP_RSP:
		srp_rsp ( srp, iobuf->data );
		break;
	case SRP_LOGIN_RSP:
		srp_login_rsp ( srp, iobuf->data );
		break;
	default:
		DBGC ( srp, "SRP %p unrecognised IU type %02x\n",
		       srp, common->type );
		break;
	}

	free_iob ( iobuf );
	return 0;
}

static int srp_command ( struct scsi_device *scsi,
			 struct scsi_command *command ) {
	struct srp_device *srp =
		container_of ( scsi->backend, struct srp_device, refcnt );

	/* Store SCSI command */
	srp->command = command;

	/* Log in or issue command as appropriate */
	if ( srp->logged_in ) {
		return srp_cmd ( srp );
	} else {
		return srp_login_req ( srp );
	}
}

int srp_attach ( struct scsi_device *scsi ) {
	struct srp_device *srp;

	/* Allocate and initialise structure */
	srp = zalloc ( sizeof ( *srp ) );
	if ( ! srp )
		return -ENOMEM;

	/* Attach parent interface, mortalise self, and return */
	scsi->backend = ref_get ( &srp->refcnt );
	scsi->command = srp_command;
	scsi->lun = srp->lun;
	srp->recv = srp_recv;
	ref_put ( &srp->refcnt );
	return 0;
}

void srp_detach ( struct scsi_device *scsi ) {
	( void ) scsi;
}
