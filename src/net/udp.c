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
#include <gpxe/resolv.h>
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

	/** UDP interface */
	struct udp_interface udp;
	/** Data transfer interface */
	struct xfer_interface xfer;
	/** Pass up received data
	 *
	 * @v conn		UDP connection
	 * @v iobuf		Datagram I/O buffer
	 * @v peer		Peer address, or NULL
	 * @v local		Local address, or NULL
	 * @v netdev		Network device, or NULL
	 * @ret rc		Return status code
	 */
	int ( * rx ) ( struct udp_connection *conn, struct io_buffer *iobuf,
		       struct sockaddr_tcpip *peer,
		       struct sockaddr_tcpip *local,
		       struct net_device *netdev );

	/** Local port on which the connection receives packets */
	unsigned int local_port;
	/** Remote socket address */
	struct sockaddr_tcpip peer;
	/** Name resolver */
	struct resolv_interface resolv;
};

/**
 * List of registered UDP connections
 */
static LIST_HEAD ( udp_conns );

/***************************************************************************
 *
 * UDP interface operations
 *
 ***************************************************************************
 */

/**
 * Close UDP interface
 *
 * @v udp		Data transfer interface
 * @v rc		Reason for close
 */
void udp_close ( struct udp_interface *udp, int rc ) {
	struct udp_interface *dest = udp_get_dest ( udp );

	DBGC ( udp, "UDP %p->%p close\n", udp, dest );

	udp_unplug ( udp );
	dest->op->close ( dest, rc );
	udp_put ( dest );
}

/**
 * Deliver datagram
 *
 *
 * @v udp		UDP interface
 * @v iobuf		Datagram I/O buffer
 * @v peer		Peer address, or NULL
 * @v local		Local address, or NULL
 * @v netdev		Network device, or NULL
 * @ret rc		Return status code
 */
int udp_deliver ( struct udp_interface *udp, struct io_buffer *iobuf,
		  struct sockaddr_tcpip *peer, struct sockaddr_tcpip *local,
		  struct net_device *netdev ) {
	struct udp_interface *dest = udp_get_dest ( udp );
	int rc;

	DBGC ( udp, "UDP %p->%p deliver %zd\n", udp, dest,
	       iob_len ( iobuf ) );

	rc = dest->op->deliver ( dest, iobuf, peer, local, netdev );

	if ( rc != 0 ) {
		DBGC ( udp, "UDP %p<-%p deliver_iob: %s\n", udp, dest,
		       strerror ( rc ) );
	}
	udp_put ( dest );
	return rc;
}

/**
 * Ignore closing a UDP connection
 *
 * @v udp		UDP interface
 * @v rc		Reason for close
 */
void ignore_udp_close ( struct udp_interface *udp __unused, int rc __unused ) {
	/* Do nothing */
}

/**
 * Ignore delivered datagram
 *
 * @v udp		UDP interface
 * @v iobuf		Datagram I/O buffer
 * @v peer		Peer address, or NULL
 * @v local		Local address, or NULL
 * @v netdev		TX network device, or NULL
 * @ret rc		Return status code
 */
int ignore_udp_deliver ( struct udp_interface *udp __unused,
			 struct io_buffer *iobuf,
			 struct sockaddr_tcpip *peer __unused,
			 struct sockaddr_tcpip *local __unused,
			 struct net_device *netdev __unused ) {
	free_iob ( iobuf );
	return 0;
}

/** Null UDP interface operations */
struct udp_interface_operations null_udp_ops = {
	.close		= ignore_udp_close,
	.deliver	= ignore_udp_deliver,
};

/**
 * Null UDP interface
 *
 * This is the interface to which UDP interfaces are
 * connected when unplugged.  It will never generate messages, and
 * will silently absorb all received messages.
 */
struct udp_interface null_udp = UDP_INIT ( null_udp, &null_udp_ops );

/***************************************************************************
 *
 * Open/close operations
 *
 ***************************************************************************
 */

static struct udp_interface_operations udp_udp_operations;
static struct xfer_interface_operations udp_xfer_operations;
static struct resolv_interface_operations udp_resolv_operations;

/**
 * Bind UDP connection to local port
 *
 * @v conn		UDP connection
 * @v port		Local port, in network byte order, or zero
 * @ret rc		Return status code
 *
 * Opens the UDP connection and binds to a local port.  If no local
 * port is specified, the first available port will be used.
 */
static int udp_bind ( struct udp_connection *conn, unsigned int port ) {
	struct udp_connection *existing;
	static uint16_t try_port = 1024;

	/* If no port specified, find the first available port */
	if ( ! port ) {
		for ( ; try_port ; try_port++ ) {
			if ( try_port < 1024 )
				continue;
			if ( udp_bind ( conn, htons ( try_port ) ) == 0 )
				return 0;
		}
		return -EADDRINUSE;
	}

	/* Attempt bind to local port */
	list_for_each_entry ( existing, &udp_conns, list ) {
		if ( existing->local_port == port ) {
			DBGC ( conn, "UDP %p could not bind: port %d in use\n",
			       conn, ntohs ( port ) );
			return -EADDRINUSE;
		}
	}
	conn->local_port = port;

	/* Add to UDP connection list */
	DBGC ( conn, "UDP %p bound to port %d\n", conn, ntohs ( port ) );

	return 0;
}

/**
 * Open a UDP connection
 *
 * @v udp		UDP interface
 * @v xfer		Data transfer interface
 * @v rx		Received data handler
 * @v peer		Peer socket address, or NULL
 * @v name		Name to resolve, or NULL
 * @v local		Local socket address, or NULL
 * @v promisc		Socket is promiscuous
 * @ret rc		Return status code
 *
 * Promiscuous UDP connections are required in order to support the
 * PXE API.
 */
static int udp_open_connection ( struct udp_interface *udp,
				 struct xfer_interface *xfer,
				 int ( * rx ) ( struct udp_connection *conn,
						struct io_buffer *iobuf,
						struct sockaddr_tcpip *peer,
						struct sockaddr_tcpip *local,
						struct net_device *netdev ),
				 struct sockaddr_tcpip *peer,
				 const char *name,
				 struct sockaddr_tcpip *local,
				 int promisc ) {
	struct udp_connection *conn;
	unsigned int bind_port;
	int rc;

	/* Allocate and initialise structure */
	conn = zalloc ( sizeof ( *conn ) );
	if ( ! conn )
		return -ENOMEM;
	DBGC ( conn, "UDP %p allocated\n", conn );
	udp_init ( &conn->udp, &udp_udp_operations, &conn->refcnt );
	xfer_init ( &conn->xfer, &udp_xfer_operations, &conn->refcnt );
	conn->rx = rx;
	if ( peer )
		memcpy ( &conn->peer, peer, sizeof ( conn->peer ) );
	resolv_init ( &conn->resolv, &udp_resolv_operations, &conn->refcnt );

	/* Start name resolution if required */
	if ( name ) {
		if ( ( rc = resolv ( &conn->resolv, name,
				 ( struct sockaddr * ) &conn->peer ) ) != 0 ) {
			goto err;
		}
	}

	/* Bind to local port */
	if ( ! promisc ) {
		bind_port = ( local ? local->st_port : 0 );
		if ( ( rc = udp_bind ( conn, bind_port ) ) != 0 )
			goto err;
	}

	/* Attach parent interface, transfer reference to connection
	 * list and return
	 */
	if ( udp )
		udp_plug_plug ( &conn->udp, udp );
	if ( xfer )
		xfer_plug_plug ( &conn->xfer, xfer );
	list_add ( &conn->list, &udp_conns );
	return 0;

 err:
	ref_put ( &conn->refcnt );
	return rc;
}

/**
 * Close a UDP connection
 *
 * @v conn		UDP connection
 * @v rc		Reason for close
 */
static void udp_close_connection ( struct udp_connection *conn, int rc ) {

	/* Close UDP interface */
	udp_nullify ( &conn->udp );
	udp_close ( &conn->udp, rc );

	/* Close data-transfer interface */
	xfer_nullify ( &conn->xfer );
	xfer_close ( &conn->xfer, rc );

	/* Remove from list of connections and drop list's reference */
	list_del ( &conn->list );
	ref_put ( &conn->refcnt );

	DBGC ( conn, "UDP %p closed\n", conn );
}

/***************************************************************************
 *
 * Interface to TCP/IP core
 *
 ***************************************************************************
 */

struct tcpip_protocol udp_protocol __tcpip_protocol;

/**
 * Allocate I/O buffer for UDP
 *
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
 * Transmit data via a UDP connection
 *
 * @v conn		UDP connection
 * @v iobuf		I/O buffer
 * @v peer		Peer address, or NULL
 * @v local		Local address, or NULL
 * @v netdev		Network device, or NULL
 * @ret rc		Return status code
 */
static int udp_tx ( struct udp_connection *conn, struct io_buffer *iobuf,
		    struct sockaddr_tcpip *peer, struct sockaddr_tcpip *local,
		    struct net_device *netdev ) {
       	struct udp_header *udphdr;
	unsigned int src_port;
	size_t len;
	int rc;

	/* Determine source port */
	src_port = ( local ? local->st_port : conn->local_port );

	/* Determine destination */
	if ( ! peer )
		peer = &conn->peer;

	/* Discard packet if we don't have a valid peer address */
	if ( ! peer->st_family ) {
		rc = -ENOTCONN;
		goto discard;
	}

	/* Check we can accommodate the header */
	if ( ( rc = iob_ensure_headroom ( iobuf, UDP_MAX_HLEN ) ) != 0 )
		goto discard;

	/* Add the UDP header */
	udphdr = iob_push ( iobuf, sizeof ( *udphdr ) );
	len = iob_len ( iobuf );
	udphdr->dest = peer->st_port;
	udphdr->src = src_port;
	udphdr->len = htons ( len );
	udphdr->chksum = 0;
	udphdr->chksum = tcpip_chksum ( udphdr, len );

	/* Dump debugging information */
	DBGC ( conn, "UDP %p TX %d->%d len %d\n", conn,
	       ntohs ( udphdr->src ), ntohs ( udphdr->dest ),
	       ntohs ( udphdr->len ) );

	/* Send it to the next layer for processing */
	if ( ( rc = tcpip_tx ( iobuf, &udp_protocol, peer, netdev,
			       &udphdr->chksum ) ) != 0 ) {
		DBGC ( conn, "UDP %p could not transmit packet: %s\n",
		       conn, strerror ( rc ) );
		return rc;
	}

	return 0;

 discard:
	free_iob ( iobuf );
	return rc;
}

/**
 * Identify UDP connection by local port number
 *
 * @v local_port	Local port (in network-endian order)
 * @ret conn		UDP connection, or NULL
 */
static struct udp_connection * udp_demux ( unsigned int local_port ) {
	struct udp_connection *conn;

	list_for_each_entry ( conn, &udp_conns, list ) {
		if ( ( conn->local_port == local_port ) ||
		     ( conn->local_port == 0 ) ) {
			return conn;
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
	struct udp_connection *conn;
	size_t ulen;
	unsigned int csum;
	int rc = 0;

	/* Sanity check packet */
	if ( iob_len ( iobuf ) < sizeof ( *udphdr ) ) {
		DBG ( "UDP packet too short at %zd bytes (min %zd bytes)\n",
		      iob_len ( iobuf ), sizeof ( *udphdr ) );
		
		rc = -EINVAL;
		goto discard;
	}
	ulen = ntohs ( udphdr->len );
	if ( ulen < sizeof ( *udphdr ) ) {
		DBG ( "UDP length too short at %zd bytes "
		      "(header is %zd bytes)\n", ulen, sizeof ( *udphdr ) );
		rc = -EINVAL;
		goto discard;
	}
	if ( ulen > iob_len ( iobuf ) ) {
		DBG ( "UDP length too long at %zd bytes (packet is %zd "
		      "bytes)\n", ulen, iob_len ( iobuf ) );
		rc = -EINVAL;
		goto discard;
	}
	if ( udphdr->chksum ) {
		csum = tcpip_continue_chksum ( pshdr_csum, iobuf->data, ulen );
		if ( csum != 0 ) {
			DBG ( "UDP checksum incorrect (is %04x including "
			      "checksum field, should be 0000)\n", csum );
			rc = -EINVAL;
			goto discard;
		}
	}

	/* Parse parameters from header and strip header */
	st_src->st_port = udphdr->src;
	st_dest->st_port = udphdr->dest;
	conn = udp_demux ( udphdr->dest );
	iob_unput ( iobuf, ( iob_len ( iobuf ) - ulen ) );
	iob_pull ( iobuf, sizeof ( *udphdr ) );

	/* Dump debugging information */
	DBGC ( conn, "UDP %p RX %d<-%d len %zd\n", conn,
	       ntohs ( udphdr->dest ), ntohs ( udphdr->src ), ulen );

	/* Ignore if no matching connection found */
	if ( ! conn ) {
		DBG ( "No UDP connection listening on port %d\n",
		      ntohs ( udphdr->dest ) );
		rc = -ENOTCONN;
		goto discard;
	}

	/* Pass data via socket */
	return conn->rx ( conn, iobuf, st_src, st_dest, NULL );

 discard:
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
 * UDP interface
 *
 ***************************************************************************
 */

/**
 * Close a UDP connection
 *
 * @v udp		UDP interface
 * @v rc		Reason for close
 */
static void udp_udp_close ( struct udp_interface *udp, int rc ) {
	struct udp_connection *conn =
		container_of ( udp, struct udp_connection, udp );

	udp_close_connection ( conn, rc );
}

/**
 * Deliver datagram
 *
 * @v udp		UDP interface
 * @v iobuf		Datagram I/O buffer
 * @v peer		Peer address, or NULL
 * @v local		Local address, or NULL
 * @v netdev		TX network device, or NULL
 * @ret rc		Return status code
 */
static int udp_udp_deliver ( struct udp_interface *udp,
			     struct io_buffer *iobuf,
			     struct sockaddr_tcpip *peer,
			     struct sockaddr_tcpip *local,
			     struct net_device *netdev ) {
	struct udp_connection *conn =
		container_of ( udp, struct udp_connection, udp );

	return udp_tx ( conn, iobuf, peer, local, netdev );
}

/** UDP socket interface */
static struct udp_interface_operations udp_udp_operations = {
	.close		= udp_udp_close,
	.deliver	= udp_udp_deliver,
};

/**
 * Pass received data up via UDP interface
 *
 * @v conn		UDP connection
 * @v iobuf		Datagram I/O buffer
 * @v peer		Peer address, or NULL
 * @v local		Local address, or NULL
 * @v netdev		Network device, or NULL
 * @ret rc		Return status code
 */
static int udp_udp_rx ( struct udp_connection *conn, struct io_buffer *iobuf,
			struct sockaddr_tcpip *peer,
			struct sockaddr_tcpip *local,
			struct net_device *netdev ) {
	return udp_deliver ( &conn->udp, iobuf, peer, local, netdev );
}

/**
 * Open a UDP connection via a UDP interface
 *
 * @v udp		UDP interface
 * @v peer		Peer socket address
 * @v local		Local socket address, or NULL
 * @ret rc		Return status code
 */
int udp_open ( struct udp_interface *udp, struct sockaddr_tcpip *peer,
	       struct sockaddr_tcpip *local ) {
	return udp_open_connection ( udp, NULL, udp_udp_rx,
				     peer, NULL, local, 0 );
}

/**
 * Open a promiscuous UDP connection via a UDP interface
 *
 * @v udp		UDP interface
 * @ret rc		Return status code
 */
int udp_open_promisc ( struct udp_interface *udp ) {
	return udp_open_connection ( udp, NULL, udp_udp_rx,
				     NULL, NULL, NULL, 0 );
}

/***************************************************************************
 *
 * Name resolution
 *
 ***************************************************************************
 */

/**
 * Handle completed UDP name resolution
 *
 * @v resolv		UDP name resolution interface
 * @v sa		Completed socket address
 * @v rc		Resolution status
 */
static void udp_resolv_done ( struct resolv_interface *resolv,
			      struct sockaddr *sa, int rc ) {
	struct udp_connection *conn =
		container_of ( resolv, struct udp_connection, resolv );

	/* Unplug resolver */
	resolv_unplug ( &conn->resolv );

	if ( rc ) {
		/* Close connection if name resolution failed */
		udp_close_connection ( conn, rc );
	} else {
		/* Store completed peer address */
		memcpy ( &conn->peer, sa, sizeof ( conn->peer ) );
	}
}

/** UDP name resolution operations */
static struct resolv_interface_operations udp_resolv_operations = {
	.done		= udp_resolv_done,
};

/**
 * Open a named UDP connection via a UDP interface
 *
 * @v udp		UDP interface
 * @v peer		Peer partial socket address
 * @v name		Peer name
 * @v local		Local socket address, or NULL
 * @ret rc		Return status code
 */
int udp_open_named ( struct udp_interface *udp, struct sockaddr_tcpip *peer,
		     const char *name, struct sockaddr_tcpip *local ) {
	return udp_open_connection ( udp, NULL, udp_udp_rx,
				     peer, name, local, 0 );
}

/***************************************************************************
 *
 * Data transfer interface
 *
 ***************************************************************************
 */

/**
 * Close a UDP connection
 *
 * @v xfer		Data transfer interface
 * @v rc		Reason for close
 */
static void udp_xfer_close ( struct xfer_interface *xfer, int rc ) {
	struct udp_connection *conn =
		container_of ( xfer, struct udp_connection, xfer );

	udp_close_connection ( conn, rc );
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
	struct udp_connection *conn =
		container_of ( xfer, struct udp_connection, xfer );
	struct io_buffer *iobuf;

	iobuf = udp_alloc_iob ( len );
	if ( ! iobuf )
		return -ENOMEM;

	memcpy ( iob_put ( iobuf, len ), data, len );
	return udp_tx ( conn, iobuf, NULL, NULL, NULL );
}

/** UDP data transfer interface operations */
static struct xfer_interface_operations udp_xfer_operations = {
	.close		= udp_xfer_close,
	.vredirect	= ignore_xfer_vredirect,
	.window		= unlimited_xfer_window,
	.deliver_iob	= xfer_deliver_as_raw,
	.deliver_raw	= udp_xfer_deliver_raw,
};

/**
 * Pass received data up via UDP interface
 *
 * @v conn		UDP connection
 * @v iobuf		Datagram I/O buffer
 * @v peer		Peer address, or NULL
 * @v local		Local address, or NULL
 * @v netdev		Network device, or NULL
 * @ret rc		Return status code
 */
static int udp_xfer_rx ( struct udp_connection *conn, struct io_buffer *iobuf,
			 struct sockaddr_tcpip *peer __unused,
			 struct sockaddr_tcpip *local __unused,
			 struct net_device *netdev __unused ) {
	return xfer_deliver_iob ( &conn->xfer, iobuf );
}

/**
 * Open a UDP connection via a data transfer interface
 *
 * @v xfer		Data transfer interface
 * @v peer		Peer socket address
 * @v local		Local socket address, or NULL
 * @ret rc		Return status code
 */
static int udp_open_xfer ( struct xfer_interface *xfer, struct sockaddr *peer,
			   struct sockaddr *local ) {
	struct sockaddr_tcpip *st_peer = ( struct sockaddr_tcpip * ) peer;
	struct sockaddr_tcpip *st_local = ( struct sockaddr_tcpip * ) local;

	return udp_open_connection ( NULL, xfer, udp_xfer_rx,
				     st_peer, NULL, st_local, 0 );
}

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
	.open		= udp_open_xfer,
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
