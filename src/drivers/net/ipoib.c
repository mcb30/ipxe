/*
 * Copyright (C) 2007 Michael Brown <mbrown@fensystems.co.uk>.
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
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <byteswap.h>
#include <errno.h>
#include <gpxe/if_arp.h>
#include <gpxe/iobuf.h>
#include <gpxe/netdevice.h>
#include <gpxe/infiniband.h>
#include <gpxe/ipoib.h>

/** @file
 *
 * IP over Infiniband
 */

/** Number of IPoIB data send work queue entries */
#define IPOIB_DATA_NUM_SEND_WQES 2

/** Number of IPoIB data receive work queue entries */
#define IPOIB_DATA_NUM_RECV_WQES 4

/** Number of IPoIB data completion entries */
#define IPOIB_DATA_NUM_CQES 8

/** Number of IPoIB metadata send work queue entries */
#define IPOIB_META_NUM_SEND_WQES 2

/** Number of IPoIB metadata receive work queue entries */
#define IPOIB_META_NUM_RECV_WQES 2

/** Number of IPoIB metadata completion entries */
#define IPOIB_META_NUM_CQES 8

/** An IPoIB queue set */
struct ipoib_queue_set {
	/** Completion queue */
	struct ib_completion_queue *cq;
	/** Queue pair */
	struct ib_queue_pair *qp;
	/** Receive work queue maximum fill level */
	unsigned int recv_max_fill;
};

/** An IPoIB device */
struct ipoib_device {
	/** Network device */
	struct net_device *netdev;
	/** Underlying Infiniband device */
	struct ib_device *ibdev;
	/** Data queue set */
	struct ipoib_queue_set data;
	/** Data queue set */
	struct ipoib_queue_set meta;
	/** Broadcast GID */
	struct ib_gid broadcast_gid;
	/** Broadcast LID */
	unsigned int broadcast_lid;
	/** Data queue key */
	unsigned long data_qkey;
	/** Attached to multicast group
	 *
	 * This flag indicates whether or not we have attached our
	 * data queue pair to the broadcast multicast GID.
	 */
	int broadcast_attached;
};

/**
 * IPoIB path cache entry
 *
 * This serves a similar role to the ARP cache for Ethernet.  (ARP
 * *is* used on IPoIB; we have two caches to maintain.)
 */
struct ipoib_cached_path {
	/** Destination GID */
	struct ib_gid gid;
	/** Destination LID */
	unsigned int dlid;
	/** Service level */
	unsigned int sl;
	/** Rate */
	unsigned int rate;
};

/** Number of IPoIB path cache entries */
#define IPOIB_NUM_CACHED_PATHS 2

/** IPoIB path cache */
static struct ipoib_cached_path ipoib_path_cache[IPOIB_NUM_CACHED_PATHS];

/** Oldest IPoIB path cache entry index */
static unsigned int ipoib_path_cache_idx = 0;

/** TID half used to identify get path record replies */
#define IPOIB_TID_GET_PATH_REC 0x11111111UL

/** TID half used to identify multicast member record replies */
#define IPOIB_TID_MC_MEMBER_REC 0x22222222UL

/** IPoIB metadata TID */
static uint32_t ipoib_meta_tid = 0;

/** IPv4 broadcast GID */
static const struct ib_gid ipv4_broadcast_gid = {
	{ { 0xff, 0x12, 0x40, 0x1b, 0x00, 0x00, 0x00, 0x00,
	    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff } }
};

/** Maximum time we will wait for the broadcast join to succeed */
#define IPOIB_JOIN_MAX_DELAY_MS 1000

/****************************************************************************
 *
 * IPoIB link layer
 *
 ****************************************************************************
 */

/** Broadcast QPN used in IPoIB MAC addresses
 *
 * This is a guaranteed invalid real QPN
 */
#define IPOIB_BROADCAST_QPN 0xffffffffUL

/** Broadcast IPoIB address */
static struct ipoib_mac ipoib_broadcast = {
	.qpn = ntohl ( IPOIB_BROADCAST_QPN ),
};

/**
 * Add IPoIB link-layer header
 *
 * @v iobuf		I/O buffer
 * @v ll_dest		Link-layer destination address
 * @v ll_source		Source link-layer address
 * @v net_proto		Network-layer protocol, in network-byte order
 * @ret rc		Return status code
 */
static int ipoib_push ( struct io_buffer *iobuf, const void *ll_dest,
			const void *ll_source __unused, uint16_t net_proto ) {
	struct ipoib_hdr *ipoib_hdr =
		iob_push ( iobuf, sizeof ( *ipoib_hdr ) );

	/* Build IPoIB header */
	memcpy ( &ipoib_hdr->pseudo.peer, ll_dest,
		 sizeof ( ipoib_hdr->pseudo.peer ) );
	ipoib_hdr->real.proto = net_proto;
	ipoib_hdr->real.reserved = 0;

	return 0;
}

/**
 * Remove IPoIB link-layer header
 *
 * @v iobuf		I/O buffer
 * @ret ll_dest		Link-layer destination address
 * @ret ll_source	Source link-layer address
 * @ret net_proto	Network-layer protocol, in network-byte order
 * @ret rc		Return status code
 */
static int ipoib_pull ( struct io_buffer *iobuf, const void **ll_dest,
			const void **ll_source, uint16_t *net_proto ) {
	struct ipoib_hdr *ipoib_hdr = iobuf->data;

	/* Sanity check */
	if ( iob_len ( iobuf ) < sizeof ( *ipoib_hdr ) ) {
		DBG ( "IPoIB packet too short for link-layer header\n" );
		DBG_HD ( iobuf->data, iob_len ( iobuf ) );
		return -EINVAL;
	}

	/* Strip off IPoIB header */
	iob_pull ( iobuf, sizeof ( *ipoib_hdr ) );

	/* Fill in required fields */
	*ll_dest = &ipoib_broadcast; /* Doesn't really exist in packet */
	*ll_source = &ipoib_hdr->pseudo.peer;
	*net_proto = ipoib_hdr->real.proto;

	return 0;
}

/**
 * Transcribe IPoIB address
 *
 * @v ll_addr	Link-layer address
 * @ret string	Link-layer address in human-readable format
 */
const char * ipoib_ntoa ( const void *ll_addr ) {
	static char buf[45];
	const struct ipoib_mac *mac = ll_addr;

	snprintf ( buf, sizeof ( buf ), "%08lx:%08lx:%08lx:%08lx:%08lx",
		   htonl ( mac->qpn ), htonl ( mac->gid.u.dwords[0] ),
		   htonl ( mac->gid.u.dwords[1] ),
		   htonl ( mac->gid.u.dwords[2] ),
		   htonl ( mac->gid.u.dwords[3] ) );
	return buf;
}

/**
 * Hash multicast address
 *
 * @v af		Address family
 * @v net_addr		Network-layer address
 * @v ll_addr		Link-layer address to fill in
 * @ret rc		Return status code
 */
static int ipoib_mc_hash ( unsigned int af __unused,
			   const void *net_addr __unused,
			   void *ll_addr __unused ) {

	return -ENOTSUP;
}

/** IPoIB protocol */
struct ll_protocol ipoib_protocol __ll_protocol = {
	.name		= "IPoIB",
	.ll_proto	= htons ( ARPHRD_INFINIBAND ),
	.ll_addr_len	= IPOIB_ALEN,
	.ll_header_len	= IPOIB_HLEN,
	.ll_broadcast	= ( uint8_t * ) &ipoib_broadcast,
	.push		= ipoib_push,
	.pull		= ipoib_pull,
	.ntoa		= ipoib_ntoa,
	.mc_hash	= ipoib_mc_hash,
};

/****************************************************************************
 *
 * IPoIB network device
 *
 ****************************************************************************
 */

/**
 * Destroy queue set
 *
 * @v ipoib		IPoIB device
 * @v qset		Queue set
 */
static void ipoib_destroy_qset ( struct ipoib_device *ipoib,
				 struct ipoib_queue_set *qset ) {
	struct ib_device *ibdev = ipoib->ibdev;

	if ( qset->qp )
		ib_destroy_qp ( ibdev, qset->qp );
	if ( qset->cq )
		ib_destroy_cq ( ibdev, qset->cq );
	memset ( qset, 0, sizeof ( *qset ) );
}

/**
 * Create queue set
 *
 * @v ipoib		IPoIB device
 * @v qset		Queue set
 * @v num_cqes		Number of completion queue entries
 * @v cq_op		Completion queue operations
 * @v num_send_wqes	Number of send work queue entries
 * @v num_recv_wqes	Number of receive work queue entries
 * @v qkey		Queue key
 * @ret rc		Return status code
 */
static int ipoib_create_qset ( struct ipoib_device *ipoib,
			       struct ipoib_queue_set *qset,
			       unsigned int num_cqes,
			       struct ib_completion_queue_operations *cq_op,
			       unsigned int num_send_wqes,
			       unsigned int num_recv_wqes,
			       unsigned long qkey ) {
	struct ib_device *ibdev = ipoib->ibdev;
	int rc;

	/* Sanity check */
	assert ( qset->cq == NULL );
	assert ( qset->qp == NULL );

	/* Store queue parameters */
	qset->recv_max_fill = num_recv_wqes;

	/* Allocate completion queue */
	qset->cq = ib_create_cq ( ibdev, num_cqes, cq_op );
	if ( ! qset->cq ) {
		DBGC ( ipoib, "IPoIB %p could not allocate completion queue\n",
		       ipoib );
		rc = -ENOMEM;
		goto err;
	}

	/* Allocate queue pair */
	qset->qp = ib_create_qp ( ibdev, num_send_wqes, qset->cq,
				  num_recv_wqes, qset->cq, qkey );
	if ( ! qset->qp ) {
		DBGC ( ipoib, "IPoIB %p could not allocate queue pair\n",
		       ipoib );
		rc = -ENOMEM;
		goto err;
	}
	ib_qp_set_ownerdata ( qset->qp, ipoib->netdev );

	return 0;

 err:
	ipoib_destroy_qset ( ipoib, qset );
	return rc;
}

/**
 * Find path cache entry by GID
 *
 * @v gid		GID
 * @ret entry		Path cache entry, or NULL
 */
static struct ipoib_cached_path *
ipoib_find_cached_path ( struct ib_gid *gid ) {
	struct ipoib_cached_path *path;
	unsigned int i;

	for ( i = 0 ; i < IPOIB_NUM_CACHED_PATHS ; i++ ) {
		path = &ipoib_path_cache[i];
		if ( memcmp ( &path->gid, gid, sizeof ( *gid ) ) == 0 )
			return path;
	}
	DBG ( "IPoIB %08lx:%08lx:%08lx:%08lx cache miss\n",
	      htonl ( gid->u.dwords[0] ), htonl ( gid->u.dwords[1] ),
	      htonl ( gid->u.dwords[2] ), htonl ( gid->u.dwords[3] ) );
	return NULL;
}

/**
 * Transmit path record request
 *
 * @v ipoib		IPoIB device
 * @v gid		Destination GID
 * @ret rc		Return status code
 */
static int ipoib_get_path_record ( struct ipoib_device *ipoib,
				   struct ib_gid *gid ) {
	struct ib_device *ibdev = ipoib->ibdev;
	struct io_buffer *iobuf;
	struct ib_mad_path_record *path_record;
	struct ib_address_vector av;
	int rc;

	/* Allocate I/O buffer */
	iobuf = alloc_iob ( sizeof ( *path_record ) );
	if ( ! iobuf )
		return -ENOMEM;
	iob_put ( iobuf, sizeof ( *path_record ) );
	path_record = iobuf->data;
	memset ( path_record, 0, sizeof ( *path_record ) );

	/* Construct path record request */
	path_record->mad_hdr.base_version = IB_MGMT_BASE_VERSION;
	path_record->mad_hdr.mgmt_class = IB_MGMT_CLASS_SUBN_ADM;
	path_record->mad_hdr.class_version = 2;
	path_record->mad_hdr.method = IB_MGMT_METHOD_GET;
	path_record->mad_hdr.attr_id = htons ( IB_SA_ATTR_PATH_REC );
	path_record->mad_hdr.tid[0] = IPOIB_TID_GET_PATH_REC;
	path_record->mad_hdr.tid[1] = ipoib_meta_tid++;
	path_record->sa_hdr.comp_mask[1] =
		htonl ( IB_SA_PATH_REC_DGID | IB_SA_PATH_REC_SGID );
	memcpy ( &path_record->dgid, gid, sizeof ( path_record->dgid ) );
	memcpy ( &path_record->sgid, &ibdev->port_gid,
		 sizeof ( path_record->sgid ) );

	/* Construct address vector */
	memset ( &av, 0, sizeof ( av ) );
	av.lid = ibdev->sm_lid;
	av.qpn = IB_SA_QPN;
	av.qkey = IB_GLOBAL_QKEY;

	/* Post send request */
	if ( ( rc = ib_post_send ( ibdev, ipoib->meta.qp, &av,
				   iobuf ) ) != 0 ) {
		DBGC ( ipoib, "IPoIB %p could not send get path record: %s\n",
		       ipoib, strerror ( rc ) );
		free_iob ( iobuf );
		return rc;
	}

	return 0;
}

/**
 * Transmit multicast group membership request
 *
 * @v ipoib		IPoIB device
 * @v gid		Multicast GID
 * @v join		Join (rather than leave) group
 * @ret rc		Return status code
 */
static int ipoib_mc_member_record ( struct ipoib_device *ipoib,
				    struct ib_gid *gid, int join ) {
	struct ib_device *ibdev = ipoib->ibdev;
	struct io_buffer *iobuf;
	struct ib_mad_mc_member_record *mc_member_record;
	struct ib_address_vector av;
	int rc;

	/* Allocate I/O buffer */
	iobuf = alloc_iob ( sizeof ( *mc_member_record ) );
	if ( ! iobuf )
		return -ENOMEM;
	iob_put ( iobuf, sizeof ( *mc_member_record ) );
	mc_member_record = iobuf->data;
	memset ( mc_member_record, 0, sizeof ( *mc_member_record ) );

	/* Construct path record request */
	mc_member_record->mad_hdr.base_version = IB_MGMT_BASE_VERSION;
	mc_member_record->mad_hdr.mgmt_class = IB_MGMT_CLASS_SUBN_ADM;
	mc_member_record->mad_hdr.class_version = 2;
	mc_member_record->mad_hdr.method = 
		( join ? IB_MGMT_METHOD_SET : IB_MGMT_METHOD_DELETE );
	mc_member_record->mad_hdr.attr_id = htons ( IB_SA_ATTR_MC_MEMBER_REC );
	mc_member_record->mad_hdr.tid[0] = IPOIB_TID_MC_MEMBER_REC;
	mc_member_record->mad_hdr.tid[1] = ipoib_meta_tid++;
	mc_member_record->sa_hdr.comp_mask[1] =
		htonl ( IB_SA_MCMEMBER_REC_MGID | IB_SA_MCMEMBER_REC_PORT_GID |
			IB_SA_MCMEMBER_REC_JOIN_STATE );
	mc_member_record->scope__join_state = 1;
	memcpy ( &mc_member_record->mgid, gid,
		 sizeof ( mc_member_record->mgid ) );
	memcpy ( &mc_member_record->port_gid, &ibdev->port_gid,
		 sizeof ( mc_member_record->port_gid ) );

	/* Construct address vector */
	memset ( &av, 0, sizeof ( av ) );
	av.lid = ibdev->sm_lid;
	av.qpn = IB_SA_QPN;
	av.qkey = IB_GLOBAL_QKEY;

	/* Post send request */
	if ( ( rc = ib_post_send ( ibdev, ipoib->meta.qp, &av,
				   iobuf ) ) != 0 ) {
		DBGC ( ipoib, "IPoIB %p could not send get path record: %s\n",
		       ipoib, strerror ( rc ) );
		free_iob ( iobuf );
		return rc;
	}

	return 0;
}

/**
 * Transmit packet via IPoIB network device
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int ipoib_transmit ( struct net_device *netdev,
			    struct io_buffer *iobuf ) {
	struct ipoib_device *ipoib = netdev->priv;
	struct ib_device *ibdev = ipoib->ibdev;
	struct ipoib_pseudo_hdr *ipoib_pshdr = iobuf->data;
	struct ib_address_vector av;
	struct ib_gid *gid;
	struct ipoib_cached_path *path;
	int rc;

	/* Sanity check */
	if ( iob_len ( iobuf ) < sizeof ( *ipoib_pshdr ) ) {
		DBGC ( ipoib, "IPoIB %p buffer too short\n", ipoib );
		return -EINVAL;
	}
	iob_pull ( iobuf, ( sizeof ( *ipoib_pshdr ) ) );

	/* Attempting transmission while link is down will put the
	 * queue pair into an error state, so don't try it.
	 */
	if ( ! ibdev->link_up )
		return -ENETUNREACH;

	/* Construct address vector */
	memset ( &av, 0, sizeof ( av ) );
	av.qkey = IB_GLOBAL_QKEY;
	av.gid_present = 1;
	if ( ipoib_pshdr->peer.qpn == htonl ( IPOIB_BROADCAST_QPN ) ) {
		/* Broadcast address */
		av.qpn = IB_BROADCAST_QPN;
		av.lid = ipoib->broadcast_lid;
		gid = &ipoib->broadcast_gid;
	} else {
		/* Unicast - look in path cache */
		path = ipoib_find_cached_path ( &ipoib_pshdr->peer.gid );
		if ( ! path ) {
			/* No path entry - get path record */
			rc = ipoib_get_path_record ( ipoib,
						     &ipoib_pshdr->peer.gid );
			netdev_tx_complete ( netdev, iobuf );
			return rc;
		}
		av.qpn = ntohl ( ipoib_pshdr->peer.qpn );
		av.lid = path->dlid;
		av.rate = path->rate;
		av.sl = path->sl;
		gid = &ipoib_pshdr->peer.gid;
	}
	memcpy ( &av.gid, gid, sizeof ( av.gid ) );

	return ib_post_send ( ibdev, ipoib->data.qp, &av, iobuf );
}

/**
 * Handle IPoIB data send completion
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v iobuf		I/O buffer
 * @v rc		Completion status code
 */
static void ipoib_data_complete_send ( struct ib_device *ibdev __unused,
				       struct ib_queue_pair *qp,
				       struct io_buffer *iobuf, int rc ) {
	struct net_device *netdev = ib_qp_get_ownerdata ( qp );

	netdev_tx_complete_err ( netdev, iobuf, rc );
}

/**
 * Handle IPoIB data receive completion
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector, or NULL
 * @v iobuf		I/O buffer
 * @v rc		Completion status code
 */
static void ipoib_data_complete_recv ( struct ib_device *ibdev __unused,
				       struct ib_queue_pair *qp,
				       struct ib_address_vector *av __unused,
				       struct io_buffer *iobuf, int rc ) {
	struct net_device *netdev = ib_qp_get_ownerdata ( qp );
	struct ipoib_device *ipoib = netdev->priv;
	struct ipoib_pseudo_hdr *ipoib_pshdr;

	if ( rc != 0 ) {
		netdev_rx_err ( netdev, iobuf, rc );
		return;
	}

	if ( iob_len ( iobuf ) < sizeof ( struct ipoib_real_hdr ) ) {
		DBGC ( ipoib, "IPoIB %p received data packet too short to "
		       "contain IPoIB header\n", ipoib );
		DBGC_HD ( ipoib, iobuf->data, iob_len ( iobuf ) );
		netdev_rx_err ( netdev, iobuf, -EIO );
		return;
	}

	ipoib_pshdr = iob_push ( iobuf, sizeof ( *ipoib_pshdr ) );
	/* FIXME: fill in a MAC address for the sake of AoE! */

	netdev_rx ( netdev, iobuf );
}

/** IPoIB data completion operations */
static struct ib_completion_queue_operations ipoib_data_cq_op = {
	.complete_send = ipoib_data_complete_send,
	.complete_recv = ipoib_data_complete_recv,
};

/**
 * Handle IPoIB metadata send completion
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v iobuf		I/O buffer
 * @v rc		Completion status code
 */
static void ipoib_meta_complete_send ( struct ib_device *ibdev __unused,
				       struct ib_queue_pair *qp,
				       struct io_buffer *iobuf, int rc ) {
	struct net_device *netdev = ib_qp_get_ownerdata ( qp );
	struct ipoib_device *ipoib = netdev->priv;

	if ( rc != 0 ) {
		DBGC ( ipoib, "IPoIB %p metadata TX completion error: %s\n",
		       ipoib, strerror ( rc ) );
	}
	free_iob ( iobuf );
}

/**
 * Handle received IPoIB path record
 *
 * @v ipoib		IPoIB device
 * @v path_record	Path record
 */
static void ipoib_recv_path_record ( struct ipoib_device *ipoib __unused,
				     struct ib_mad_path_record *path_record ) {
	struct ipoib_cached_path *path;

	/* Update path cache entry */
	path = &ipoib_path_cache[ipoib_path_cache_idx];
	memcpy ( &path->gid, &path_record->dgid, sizeof ( path->gid ) );
	path->dlid = ntohs ( path_record->dlid );
	path->sl = ( path_record->reserved__sl & 0x0f );
	path->rate = ( path_record->rate_selector__rate & 0x3f );

	DBG ( "IPoIB %08lx:%08lx:%08lx:%08lx dlid %x sl %x rate %x\n",
	      htonl ( path->gid.u.dwords[0] ), htonl ( path->gid.u.dwords[1] ),
	      htonl ( path->gid.u.dwords[2] ), htonl ( path->gid.u.dwords[3] ),
	      path->dlid, path->sl, path->rate );
	
	/* Update path cache index */
	ipoib_path_cache_idx++;
	if ( ipoib_path_cache_idx == IPOIB_NUM_CACHED_PATHS )
		ipoib_path_cache_idx = 0;
}

/**
 * Handle received IPoIB multicast membership record
 *
 * @v ipoib		IPoIB device
 * @v mc_member_record	Multicast membership record
 */
static void ipoib_recv_mc_member_record ( struct ipoib_device *ipoib,
			  struct ib_mad_mc_member_record *mc_member_record ) {
	int joined;
	int rc;

	/* Record parameters */
	joined = ( mc_member_record->scope__join_state & 0x0f );
	ipoib->data_qkey = ntohl ( mc_member_record->qkey );
	ipoib->broadcast_lid = ntohs ( mc_member_record->mlid );
	DBGC ( ipoib, "IPoIB %p %s broadcast group: qkey %lx mlid %x\n",
	       ipoib, ( joined ? "joined" : "left" ), ipoib->data_qkey,
	       ipoib->broadcast_lid );

	/* Update data queue pair qkey */
	if ( ( rc = ib_modify_qp ( ipoib->ibdev, ipoib->data.qp,
				   IB_MODIFY_QKEY, ipoib->data_qkey ) ) != 0 ){
		DBGC ( ipoib, "IPoIB %p could not update data qkey: %s\n",
		       ipoib, strerror ( rc ) );
		return;
	}
}

/**
 * Handle IPoIB metadata receive completion
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector, or NULL
 * @v iobuf		I/O buffer
 * @v rc		Completion status code
 */
static void
ipoib_meta_complete_recv ( struct ib_device *ibdev __unused,
			   struct ib_queue_pair *qp,
			   struct ib_address_vector *av __unused,
			   struct io_buffer *iobuf, int rc ) {
	struct net_device *netdev = ib_qp_get_ownerdata ( qp );
	struct ipoib_device *ipoib = netdev->priv;
	union ib_mad *mad;

	if ( rc != 0 ) {
		DBGC ( ipoib, "IPoIB %p metadata RX completion error: %s\n",
		       ipoib, strerror ( rc ) );
		goto done;
	}

	if ( iob_len ( iobuf ) < sizeof ( *mad ) ) {
		DBGC ( ipoib, "IPoIB %p received metadata packet too short "
		       "to contain reply\n", ipoib );
		DBGC_HD ( ipoib, iobuf->data, iob_len ( iobuf ) );
		goto done;
	}
	mad = iobuf->data;

	if ( mad->mad_hdr.status != 0 ) {
		DBGC ( ipoib, "IPoIB %p metadata RX err status %04x\n",
		       ipoib, ntohs ( mad->mad_hdr.status ) );
		goto done;
	}

	switch ( mad->mad_hdr.tid[0] ) {
	case IPOIB_TID_GET_PATH_REC:
		ipoib_recv_path_record ( ipoib, &mad->path_record );
		break;
	case IPOIB_TID_MC_MEMBER_REC:
		ipoib_recv_mc_member_record ( ipoib, &mad->mc_member_record );
		break;
	default:
		DBGC ( ipoib, "IPoIB %p unwanted response:\n",
		       ipoib );
		DBGC_HD ( ipoib, mad, sizeof ( *mad ) );
		break;
	}

 done:
	free_iob ( iobuf );
}

/** IPoIB metadata completion operations */
static struct ib_completion_queue_operations ipoib_meta_cq_op = {
	.complete_send = ipoib_meta_complete_send,
	.complete_recv = ipoib_meta_complete_recv,
};

/**
 * Refill IPoIB receive ring
 *
 * @v ipoib		IPoIB device
 */
static void ipoib_refill_recv ( struct ipoib_device *ipoib,
				struct ipoib_queue_set *qset ) {
	struct ib_device *ibdev = ipoib->ibdev;
	struct io_buffer *iobuf;
	int rc;

	while ( qset->qp->recv.fill < qset->recv_max_fill ) {
		iobuf = alloc_iob ( IPOIB_PKT_LEN );
		if ( ! iobuf )
			break;
		if ( ( rc = ib_post_recv ( ibdev, qset->qp, iobuf ) ) != 0 ) {
			free_iob ( iobuf );
			break;
		}
	}
}

/**
 * Poll IPoIB network device
 *
 * @v netdev		Network device
 */
static void ipoib_poll ( struct net_device *netdev ) {
	struct ipoib_device *ipoib = netdev->priv;
	struct ib_device *ibdev = ipoib->ibdev;

	ib_poll_cq ( ibdev, ipoib->meta.cq );
	ib_poll_cq ( ibdev, ipoib->data.cq );
	ipoib_refill_recv ( ipoib, &ipoib->meta );
	ipoib_refill_recv ( ipoib, &ipoib->data );
}

/**
 * Enable/disable interrupts on IPoIB network device
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void ipoib_irq ( struct net_device *netdev __unused,
			int enable __unused ) {
	/* No implementation */
}

/**
 * Join IPv4 broadcast multicast group
 *
 * @v ipoib		IPoIB device
 * @ret rc		Return status code
 */
static int ipoib_join_broadcast_group ( struct ipoib_device *ipoib ) {
	int rc;

	/* Sanity check */
	if ( ! ipoib->data.qp )
		return 0;

	/* Attach data queue to broadcast multicast GID */
	assert ( ipoib->broadcast_attached == 0 );
	if ( ( rc = ib_mcast_attach ( ipoib->ibdev, ipoib->data.qp,
				      &ipoib->broadcast_gid ) ) != 0 ){
		DBGC ( ipoib, "IPoIB %p could not attach to broadcast GID: "
		       "%s\n", ipoib, strerror ( rc ) );
		return rc;
	}
	ipoib->broadcast_attached = 1;

	/* Initiate broadcast group join */
	if ( ( rc = ipoib_mc_member_record ( ipoib, &ipoib->broadcast_gid,
					     1 ) ) != 0 ) {
		DBGC ( ipoib, "IPoIB %p could not send broadcast join: %s\n",
		       ipoib, strerror ( rc ) );
		return rc;
	}

	/* We will set link up on the network device when we receive
	 * the broadcast join response.
	 */

	return 0;
}

/**
 * Leave IPv4 broadcast multicast group
 *
 * @v ipoib		IPoIB device
 */
static void ipoib_leave_broadcast_group ( struct ipoib_device *ipoib ) {

	/* Detach data queue from broadcast multicast GID */
	if ( ipoib->broadcast_attached ) {
		assert ( ipoib->data.qp != NULL );
		ib_mcast_detach ( ipoib->ibdev, ipoib->data.qp,
				  &ipoib->broadcast_gid );
		ipoib->broadcast_attached = 0;
	}
}

/**
 * Open IPoIB network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int ipoib_open ( struct net_device *netdev ) {
	struct ipoib_device *ipoib = netdev->priv;
	struct ipoib_mac *mac = ( ( struct ipoib_mac * ) netdev->ll_addr );
	int rc;

	/* Allocate metadata queue set */
	if ( ( rc = ipoib_create_qset ( ipoib, &ipoib->meta,
					IPOIB_META_NUM_CQES,
					&ipoib_meta_cq_op,
					IPOIB_META_NUM_SEND_WQES,
					IPOIB_META_NUM_RECV_WQES,
					IB_GLOBAL_QKEY ) ) != 0 ) {
		DBGC ( ipoib, "IPoIB %p could not allocate metadata QP: %s\n",
		       ipoib, strerror ( rc ) );
		goto err_create_meta_qset;
	}

	/* Allocate data queue set */
	if ( ( rc = ipoib_create_qset ( ipoib, &ipoib->data,
					IPOIB_DATA_NUM_CQES,
					&ipoib_data_cq_op,
					IPOIB_DATA_NUM_SEND_WQES,
					IPOIB_DATA_NUM_RECV_WQES,
					IB_GLOBAL_QKEY ) ) != 0 ) {
		DBGC ( ipoib, "IPoIB %p could not allocate data QP: %s\n",
		       ipoib, strerror ( rc ) );
		goto err_create_data_qset;
	}

	/* Update MAC address with data QPN */
	mac->qpn = htonl ( ipoib->data.qp->qpn );

	/* Fill receive rings */
	ipoib_refill_recv ( ipoib, &ipoib->meta );
	ipoib_refill_recv ( ipoib, &ipoib->data );

	/* Join broadcast group */
	if ( ( rc = ipoib_join_broadcast_group ( ipoib ) ) != 0 ) {
		DBGC ( ipoib, "IPoIB %p could not join broadcast group: %s\n",
		       ipoib, strerror ( rc ) );
		goto err_join_broadcast;
	}

	return 0;

 err_join_broadcast:
	ipoib_destroy_qset ( ipoib, &ipoib->data );
 err_create_data_qset:
	ipoib_destroy_qset ( ipoib, &ipoib->meta );
 err_create_meta_qset:
	return rc;
}

/**
 * Close IPoIB network device
 *
 * @v netdev		Network device
 */
static void ipoib_close ( struct net_device *netdev ) {
	struct ipoib_device *ipoib = netdev->priv;
	struct ipoib_mac *mac = ( ( struct ipoib_mac * ) netdev->ll_addr );

	/* Leave broadcast group */
	ipoib_leave_broadcast_group ( ipoib );

	/* Remove data QPN from MAC address */
	mac->qpn = 0;

	/* Tear down the queues */
	ipoib_destroy_qset ( ipoib, &ipoib->data );
	ipoib_destroy_qset ( ipoib, &ipoib->meta );
}

/** IPoIB network device operations */
static struct net_device_operations ipoib_operations = {
	.open		= ipoib_open,
	.close		= ipoib_close,
	.transmit	= ipoib_transmit,
	.poll		= ipoib_poll,
	.irq		= ipoib_irq,
};

/**
 * Update IPoIB dynamic Infiniband parameters
 *
 * @v ipoib		IPoIB device
 *
 * The Infiniband port GID and partition key will change at runtime,
 * when the link is established (or lost).  The MAC address is based
 * on the port GID, and the broadcast GID is based on the partition
 * key.  This function recalculates these IPoIB device parameters.
 */
static void ipoib_set_ib_params ( struct ipoib_device *ipoib ) {
	struct ib_device *ibdev = ipoib->ibdev;
	struct net_device *netdev = ipoib->netdev;
	struct ipoib_mac *mac;

	/* Calculate GID portion of MAC address based on port GID */
	mac = ( ( struct ipoib_mac * ) netdev->ll_addr );
	memcpy ( &mac->gid, &ibdev->port_gid, sizeof ( mac->gid ) );

	/* Calculate broadcast GID based on partition key */
	memcpy ( &ipoib->broadcast_gid, &ipv4_broadcast_gid,
		 sizeof ( ipoib->broadcast_gid ) );
	ipoib->broadcast_gid.u.words[2] = htons ( ibdev->pkey );

	/* Set net device link state to reflect Infiniband link state */
	if ( ibdev->link_up ) {
		netdev_link_up ( netdev );
	} else {
		netdev_link_down ( netdev );
	}
}

/**
 * Handle link status change
 *
 * @v ibdev		Infiniband device
 */
void ipoib_link_state_changed ( struct ib_device *ibdev ) {
	struct net_device *netdev = ib_get_ownerdata ( ibdev );
	struct ipoib_device *ipoib = netdev->priv;
	int rc;

	/* Leave existing broadcast group */
	ipoib_leave_broadcast_group ( ipoib );

	/* Update MAC address and broadcast GID based on new port GID
	 * and partition key.
	 */
	ipoib_set_ib_params ( ipoib );

	/* Join new broadcast group */
	if ( ( rc = ipoib_join_broadcast_group ( ipoib ) ) != 0 ) {
		DBGC ( ipoib, "IPoIB %p could not rejoin broadcast group: "
		       "%s\n", ipoib, strerror ( rc ) );
		return;
	}
}

/**
 * Probe IPoIB device
 *
 * @v ibdev		Infiniband device
 * @ret rc		Return status code
 */
int ipoib_probe ( struct ib_device *ibdev ) {
	struct net_device *netdev;
	struct ipoib_device *ipoib;
	int rc;

	/* Allocate network device */
	netdev = alloc_ipoibdev ( sizeof ( *ipoib ) );
	if ( ! netdev )
		return -ENOMEM;
	netdev_init ( netdev, &ipoib_operations );
	ipoib = netdev->priv;
	ib_set_ownerdata ( ibdev, netdev );
	netdev->dev = ibdev->dev;
	memset ( ipoib, 0, sizeof ( *ipoib ) );
	ipoib->netdev = netdev;
	ipoib->ibdev = ibdev;

	/* Calculate as much of the broadcast GID and the MAC address
	 * as we can.  We won't know either of these in full until we
	 * have link-up.
	 */
	ipoib_set_ib_params ( ipoib );

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	return 0;

 err_register_netdev:
	netdev_nullify ( netdev );
	netdev_put ( netdev );
	return rc;
}

/**
 * Remove IPoIB device
 *
 * @v ibdev		Infiniband device
 */
void ipoib_remove ( struct ib_device *ibdev ) {
	struct net_device *netdev = ib_get_ownerdata ( ibdev );

	unregister_netdev ( netdev );
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}
