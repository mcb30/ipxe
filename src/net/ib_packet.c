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
#include <byteswap.h>
#include <gpxe/iobuf.h>
#include <gpxe/infiniband.h>
#include <gpxe/ib_packet.h>

/**
 * @file
 *
 * Infiniband Packet Formats
 *
 */

/**
 * Add IB headers
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v iobuf		I/O buffer containing headers
 * @v payload		I/O buffer containing payload
 * @v av		Address vector to fill in
 */
int ib_push ( struct ib_device *ibdev, struct ib_queue_pair *qp,
	      struct io_buffer *iobuf, struct io_buffer *payload,
	      const struct ib_address_vector *av ) {
	struct ib_local_route_header *lrh;
	struct ib_global_route_header *grh;
	struct ib_base_transport_header *bth;
	struct ib_datagram_extended_transport_header *deth;
	size_t payload_len;
	size_t pad_len;
	size_t lrh_len;
	size_t grh_len;
	unsigned int lnh;

	/* Calculate packet length */
	payload_len = iob_len ( payload );
	pad_len = ( (-payload_len) & 0x3 );
	payload_len += pad_len;
	payload_len += 4; /* ICRC */

	/* Reserve space for headers */
	deth = iob_push ( iobuf, sizeof ( *deth ) );
	bth = iob_push ( iobuf, sizeof ( *bth ) );
	grh_len = ( payload_len + iob_len ( iobuf ) );
	grh = ( av->gid_present ?
		iob_push ( iobuf, sizeof ( *grh ) ) : NULL );
	lrh = iob_push ( iobuf, sizeof ( *lrh ) );
	lrh_len = ( payload_len + iob_len ( iobuf ) );

	/* Construct LRH */
	lrh->vl__lver = 0xf0; // ??
	lnh = ( grh ? IB_LNH_GRH : IB_LNH_BTH );
	lrh->sl__lnh = ( ( av->sl << 4 ) | lnh );
	lrh->dlid = htons ( av->lid );
	lrh->length = htons ( lrh_len >> 2 );
	lrh->slid = htons ( ibdev->lid );

	/* Construct GRH, if required */
	if ( grh ) {
		grh->ipver__tclass__flowlabel =
			htonl ( IB_GRH_IPVER_IPv6 << 28 );
		grh->paylen = htons ( grh_len );
		grh->nxthdr = IB_GRH_NXTHDR_IBA;
		grh->hoplmt = IB_GRH_HOPLMT_MAX;
		memcpy ( &grh->sgid, &ibdev->gid, sizeof ( grh->sgid ) );
		memcpy ( &grh->dgid, &av->gid, sizeof ( grh->dgid ) );
	}

	/* Construct BTH */
	bth->opcode = BTH_OPCODE_UD_SEND;
	bth->tver__padcnt__m__se = ( ( 1 /* SE */ << 7 ) | ( pad_len << 4 ) );
	bth->pkey = htons ( ibdev->pkey );
	bth->dest_qp = htonl ( av->qpn );
	bth->psn__ack = htonl ( ( ibdev->psn++ ) & 0xffffffUL );

	/* Construct DETH */
	deth->qkey = htonl ( av->qkey );
	deth->src_qp = htonl ( qp->qpn );

	return 0;
}

/**
 * Remove IB headers
 *
 * @v ibdev		Infiniband device
 * @v iobuf		I/O buffer containing headers
 * @v av		Address vector to fill in
 */
int ib_pull ( struct ib_device *ibdev, struct io_buffer *iobuf,
	      struct ib_address_vector *av ) {
	struct ib_local_route_header *lrh;
	struct ib_global_route_header *grh;
	struct ib_base_transport_header *bth;
	struct ib_datagram_extended_transport_header *deth;
	unsigned int lnh;

	/* Clear address vector */
	memset ( av, 0, sizeof ( *av ) );

	/* Extract LRH */
	if ( iob_len ( iobuf ) < sizeof ( *lrh ) ) {
		DBGC ( ibdev, "IBDEV %p RX too short (%zd bytes) for LRH\n",
		       ibdev, iob_len ( iobuf ) );
		return -EINVAL;
	}
	lrh = iobuf->data;
	iob_pull ( iobuf, sizeof ( *lrh ) );
	av->lid = ntohs ( lrh->slid );
	av->sl = ( lrh->sl__lnh >> 4 );
	lnh = ( lrh->sl__lnh & 0x3 );

	/* Reject unsupported packets */
	if ( ! ( ( lnh == IB_LNH_BTH ) || ( lnh == IB_LNH_GRH ) ) ) {
		DBGC ( ibdev, "IBDEV %p RX unsupported LNH %x\n",
		       ibdev, lnh );
		return -ENOTSUP;
	}

	/* Extract GRH, if present */
	if ( lnh == IB_LNH_GRH ) {
		if ( iob_len ( iobuf ) < sizeof ( *grh ) ) {
			DBGC ( ibdev, "IIBDEV %p RX too short (%zd bytes) "
			       "for GRH\n", ibdev, iob_len ( iobuf ) );
			return -EINVAL;
		}
		grh = iobuf->data;
		iob_pull ( iobuf, sizeof ( *grh ) );
		av->gid_present = 1;
		memcpy ( &av->gid, &grh->sgid, sizeof ( av->gid ) );
	}

	/* Extract BTH */
	if ( iob_len ( iobuf ) < sizeof ( *bth ) ) {
		DBGC ( ibdev, "IBDEV %p RX too short (%zd bytes) for BTH\n",
		       ibdev, iob_len ( iobuf ) );
		return -EINVAL;
	}
	bth = iobuf->data;
	iob_pull ( iobuf, sizeof ( *bth ) );
	if ( bth->opcode != BTH_OPCODE_UD_SEND ) {
		DBGC ( ibdev, "IBDEV %p unsupported BTH opcode %x\n",
		       ibdev, bth->opcode );
		return -ENOTSUP;
	}

	/* Extract DETH */
	if ( iob_len ( iobuf ) < sizeof ( *deth ) ) {
		DBGC ( ibdev, "IBDEV %p RX too short (%zd bytes) for DETH\n",
		       ibdev, iob_len ( iobuf ) );
		return -EINVAL;
	}
	deth = iobuf->data;
	iob_pull ( iobuf, sizeof ( *deth ) );
	av->qpn = ntohl ( deth->src_qp );
	av->qkey = ntohl ( deth->qkey );

	return 0;
}
