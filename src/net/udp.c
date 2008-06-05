#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <byteswap.h>
#include <errno.h>
#include <gpxe/tcpip.h>
#include <gpxe/iobuf.h>
#include <gpxe/xfer.h>
#include <gpxe/open.h>
#include <gpxe/uri.h>
#include <gpxe/udp.h>

/** @file
 *
 * UDP protocol
 */

/**
 * A UDP connection
 *
 */
struct udp_connection {
	/** Reference counter */
	struct refcnt refcnt;
	/** List of UDP connections */
	struct list_head list;

	/** Data transfer interface */
	struct xfer_interface xfer;

	/** Remote socket address */
	struct sockaddr_tcpip peer;
	/** Local port on which the connection receives packets */
	unsigned int local_port;
};

/**
 * List of registered UDP connections
 */
static LIST_HEAD ( udp_conns );

/* Forward declatations */
static struct xfer_interface_operations udp_xfer_operations;
struct tcpip_protocol udp_protocol;

/**
 * Bind UDP connection to local port
 *
 * @v udp		UDP connection
 * @v port		Local port, in network byte order, or zero
 * @ret rc		Return status code
 *
 * Opens the UDP connection and binds to a local port.  If no local
 * port is specified, the first available port will be used.
 */
static int udp_bind ( struct udp_connection *udp, unsigned int port ) {
	struct udp_connection *existing;
	static uint16_t try_port = 1024;

	/* If no port specified, find the first available port */
	if ( ! port ) {
		for ( ; try_port ; try_port++ ) {
			if ( try_port < 1024 )
				continue;
			if ( udp_bind ( udp, htons ( try_port ) ) == 0 )
				return 0;
		}
		return -EADDRINUSE;
	}

	/* Attempt bind to local port */
	list_for_each_entry ( existing, &udp_conns, list ) {
		if ( existing->local_port == port ) {
			DBGC ( udp, "UDP %p could not bind: port %d in use\n",
			       udp, ntohs ( port ) );
			return -EADDRINUSE;
		}
	}
	udp->local_port = port;

	/* Add to UDP connection list */
	DBGC ( udp, "UDP %p bound to port %d\n", udp, ntohs ( port ) );

	return 0;
}

/**
 * Open a UDP connection
 *
 * @v xfer		Data transfer interface
 * @v peer		Peer socket address, or NULL
 * @v local		Local socket address, or NULL
 * @v promisc		Socket is promiscuous
 * @ret rc		Return status code
 */
static int udp_open_common ( struct xfer_interface *xfer,
			     struct sockaddr *peer, struct sockaddr *local,
			     int promisc ) {
	struct sockaddr_tcpip *st_peer = ( struct sockaddr_tcpip * ) peer;
	struct sockaddr_tcpip *st_local = ( struct sockaddr_tcpip * ) local;
	struct udp_connection *udp;
	unsigned int bind_port;
	int rc;

	/* Allocate and initialise structure */
	udp = zalloc ( sizeof ( *udp ) );
	if ( ! udp )
		return -ENOMEM;
	DBGC ( udp, "UDP %p allocated\n", udp );
	xfer_init ( &udp->xfer, &udp_xfer_operations, &udp->refcnt );
	if ( st_peer )
		memcpy ( &udp->peer, st_peer, sizeof ( udp->peer ) );

	/* Bind to local port */
	if ( ! promisc ) {
		bind_port = ( st_local ? st_local->st_port : 0 );
		if ( ( rc = udp_bind ( udp, bind_port ) ) != 0 )
			goto err;
	}

	/* Attach parent interface, transfer reference to connection
	 * list and return
	 */
	xfer_plug_plug ( &udp->xfer, xfer );
	list_add ( &udp->list, &udp_conns );
	return 0;

 err:
	ref_put ( &udp->refcnt );
	return rc;
}

/**
 * Open a UDP connection
 *
 * @v xfer		Data transfer interface
 * @v peer		Peer socket address
 * @v local		Local socket address, or NULL
 * @ret rc		Return status code
 */
int udp_open ( struct xfer_interface *xfer, struct sockaddr *peer,
	       struct sockaddr *local ) {
	return udp_open_common ( xfer, peer, local, 0 );
}

/**
 * Open a promiscuous UDP connection
 *
 * @v xfer		Data transfer interface
 * @ret rc		Return status code
 *
 * Promiscuous UDP connections are required in order to support the
 * PXE API.
 */
int udp_open_promisc ( struct xfer_interface *xfer ) {
	return udp_open_common ( xfer, NULL, NULL, 1 );
}

/**
 * Close a UDP connection
 *
 * @v udp		UDP connection
 * @v rc		Reason for close
 */
static void udp_close ( struct udp_connection *udp, int rc ) {

	/* Close data transfer interface */
	xfer_nullify ( &udp->xfer );
	xfer_close ( &udp->xfer, rc );

	/* Remove from list of connections and drop list's reference */
	list_del ( &udp->list );
	ref_put ( &udp->refcnt );

	DBGC ( udp, "UDP %p closed\n", udp );
}

/**
 * Allocate I/O buffer for UDP
 *
 * @v xfer		Data transfer interface
 * @v len		Payload size
 * @ret iobuf		I/O buffer, or NULL
 */
struct io_buffer * udp_alloc_iob ( size_t len ) {
	struct io_buffer *iobuf;

	iobuf = alloc_iob ( UDP_MAX_HLEN + len );
	if ( iobuf )
		iob_reserve ( iobuf, UDP_MAX_HLEN );
	return iobuf;
}

/**
 * Transmit data via a UDP connection to a specified address
 *
 * @v udp		UDP connection
 * @v iobuf		I/O buffer
 * @v src_port		Source port, or 0 to use default
 * @v dest		Destination address, or NULL to use default
 * @v netdev		Network device, or NULL to use default
 * @ret rc		Return status code
 */
static int udp_tx ( struct udp_connection *udp, struct io_buffer *iobuf,
		    unsigned int src_port, struct sockaddr_tcpip *dest,
		    struct net_device *netdev ) {
       	struct udp_header *udphdr;
	size_t len;
	int rc;

	/* Check we can accommodate the header */
	if ( ( rc = iob_ensure_headroom ( iobuf, UDP_MAX_HLEN ) ) != 0 ) {
		free_iob ( iobuf );
		return rc;
	}

	/* Fill in default values if not explicitly provided */
	if ( ! src_port )
		src_port = udp->local_port;
	if ( ! dest )
		dest = &udp->peer;

	/* Add the UDP header */
	udphdr = iob_push ( iobuf, sizeof ( *udphdr ) );
	len = iob_len ( iobuf );
	udphdr->dest = dest->st_port;
	udphdr->src = src_port;
	udphdr->len = htons ( len );
	udphdr->chksum = 0;
	udphdr->chksum = tcpip_chksum ( udphdr, len );

	/* Dump debugging information */
	DBGC ( udp, "UDP %p TX %d->%d len %d\n", udp,
	       ntohs ( udphdr->src ), ntohs ( udphdr->dest ),
	       ntohs ( udphdr->len ) );

	/* Send it to the next layer for processing */
	if ( ( rc = tcpip_tx ( iobuf, &udp_protocol, dest, netdev,
			       &udphdr->chksum ) ) != 0 ) {
		DBGC ( udp, "UDP %p could not transmit packet: %s\n",
		       udp, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Identify UDP connection by local port number
 *
 * @v local_port	Local port (in network-endian order)
 * @ret udp		UDP connection, or NULL
 */
static struct udp_connection * udp_demux ( unsigned int local_port ) {
	struct udp_connection *udp;

	list_for_each_entry ( udp, &udp_conns, list ) {
		if ( ( udp->local_port == local_port ) ||
		     ( udp->local_port == 0 ) ) {
			return udp;
		}
	}
	return NULL;
}

/**
 * Process a received packet
 *
 * @v iobuf		I/O buffer
 * @v st_src		Partially-filled source address
 * @v st_dest		Partially-filled destination address
 * @v pshdr_csum	Pseudo-header checksum
 * @ret rc		Return status code
 */
static int udp_rx ( struct io_buffer *iobuf, struct sockaddr_tcpip *st_src,
		    struct sockaddr_tcpip *st_dest, uint16_t pshdr_csum ) {
	struct udp_header *udphdr = iobuf->data;
	struct udp_connection *udp;
	struct xfer_metadata meta;
	size_t ulen;
	unsigned int csum;
	int rc = 0;

	/* Sanity check packet */
	if ( iob_len ( iobuf ) < sizeof ( *udphdr ) ) {
		DBG ( "UDP packet too short at %zd bytes (min %zd bytes)\n",
		      iob_len ( iobuf ), sizeof ( *udphdr ) );
		
		rc = -EINVAL;
		goto done;
	}
	ulen = ntohs ( udphdr->len );
	if ( ulen < sizeof ( *udphdr ) ) {
		DBG ( "UDP length too short at %zd bytes "
		      "(header is %zd bytes)\n", ulen, sizeof ( *udphdr ) );
		rc = -EINVAL;
		goto done;
	}
	if ( ulen > iob_len ( iobuf ) ) {
		DBG ( "UDP length too long at %zd bytes (packet is %zd "
		      "bytes)\n", ulen, iob_len ( iobuf ) );
		rc = -EINVAL;
		goto done;
	}
	if ( udphdr->chksum ) {
		csum = tcpip_continue_chksum ( pshdr_csum, iobuf->data, ulen );
		if ( csum != 0 ) {
			DBG ( "UDP checksum incorrect (is %04x including "
			      "checksum field, should be 0000)\n", csum );
			rc = -EINVAL;
			goto done;
		}
	}

	/* Parse parameters from header and strip header */
	st_src->st_port = udphdr->src;
	st_dest->st_port = udphdr->dest;
	udp = udp_demux ( udphdr->dest );
	iob_unput ( iobuf, ( iob_len ( iobuf ) - ulen ) );
	iob_pull ( iobuf, sizeof ( *udphdr ) );

	/* Dump debugging information */
	DBGC ( udp, "UDP %p RX %d<-%d len %zd\n", udp,
	       ntohs ( udphdr->dest ), ntohs ( udphdr->src ), ulen );

	/* Ignore if no matching connection found */
	if ( ! udp ) {
		DBG ( "No UDP connection listening on port %d\n",
		      ntohs ( udphdr->dest ) );
		rc = -ENOTCONN;
		goto done;
	}

	/* Pass data to application */
	memset ( &meta, 0, sizeof ( meta ) );
	meta.src = ( struct sockaddr * ) st_src;
	meta.dest = ( struct sockaddr * ) st_dest;
	rc = xfer_deliver_iob_meta ( &udp->xfer, iobuf, &meta );
	iobuf = NULL;

 done:
	free_iob ( iobuf );
	return rc;
}

struct tcpip_protocol udp_protocol __tcpip_protocol = {
	.name = "UDP",
	.rx = udp_rx,
	.tcpip_proto = IP_UDP,
};

/***************************************************************************
 *
 * Data transfer interface
 *
 ***************************************************************************
 */

/**
 * Close interface
 *
 * @v xfer		Data transfer interface
 * @v rc		Reason for close
 */
static void udp_xfer_close ( struct xfer_interface *xfer, int rc ) {
	struct udp_connection *udp =
		container_of ( xfer, struct udp_connection, xfer );

	/* Close connection */
	udp_close ( udp, rc );
}

/**
 * Deliver datagram as I/O buffer
 *
 * @v xfer		Data transfer interface
 * @v iobuf		Datagram I/O buffer
 * @v meta		Data transfer metadata, or NULL
 * @ret rc		Return status code
 */
static int udp_xfer_deliver_iob ( struct xfer_interface *xfer,
				  struct io_buffer *iobuf,
				  struct xfer_metadata *meta ) {
	struct udp_connection *udp =
		container_of ( xfer, struct udp_connection, xfer );
	struct sockaddr_tcpip *src;
	struct sockaddr_tcpip *dest = NULL;
	struct net_device *netdev = NULL;
	unsigned int src_port = 0;

	/* Apply xfer metadata */
	if ( meta ) {
		src = ( struct sockaddr_tcpip * ) meta->src;
		if ( src )
			src_port = src->st_port;
		dest = ( struct sockaddr_tcpip * ) meta->dest;
		netdev = meta->netdev;
	}

	/* Transmit data, if possible */
	udp_tx ( udp, iobuf, src_port, dest, netdev );

	return 0;
}

/**
 * Deliver datagram as raw data
 *
 * @v xfer		Data transfer interface
 * @v iobuf		Datagram I/O buffer
 * @v meta		Data transfer metadata, or NULL
 * @ret rc		Return status code
 */
static int udp_xfer_deliver_raw ( struct xfer_interface *xfer,
				  const void *data, size_t len ) {
	struct io_buffer *iobuf;

	iobuf = udp_alloc_iob ( len );
	if ( ! iobuf )
		return -ENOMEM;

	memcpy ( iob_put ( iobuf, len ), data, len );
	return udp_xfer_deliver_iob ( xfer, iobuf, NULL );
}

/** UDP data transfer interface operations */
static struct xfer_interface_operations udp_xfer_operations = {
	.close		= udp_xfer_close,
	.vredirect	= ignore_xfer_vredirect,
	.window		= unlimited_xfer_window,
	.deliver_iob	= udp_xfer_deliver_iob,
	.deliver_raw	= udp_xfer_deliver_raw,
};

/***************************************************************************
 *
 * Openers
 *
 ***************************************************************************
 */

/** UDP socket opener */
struct socket_opener udp_socket_opener __socket_opener = {
	.semantics	= SOCK_DGRAM,
	.family		= AF_INET,
	.open		= udp_open,
};

char UDP_SOCK_DGRAM[1];

/**
 * Open UDP URI
 *
 * @v xfer		Data transfer interface
 * @v uri		URI
 * @ret rc		Return status code
 */
static int udp_open_uri ( struct xfer_interface *xfer, struct uri *uri ) {
	struct sockaddr_tcpip peer;

	/* Sanity check */
	if ( ! uri->host )
		return -EINVAL;

	memset ( &peer, 0, sizeof ( peer ) );
	peer.st_port = htons ( uri_port ( uri, 0 ) );
	return xfer_open_named_socket ( xfer, SOCK_DGRAM,
					( struct sockaddr * ) &peer,
					uri->host, NULL );
}

/** UDP URI opener */
struct uri_opener udp_uri_opener __uri_opener = {
	.scheme		= "udp",
	.open		= udp_open_uri,
};
