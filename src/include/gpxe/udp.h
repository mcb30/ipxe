#ifndef _GPXE_UDP_H
#define _GPXE_UDP_H

/** @file
 *
 * UDP protocol
 *
 * This file defines the gPXE UDP API.
 *
 */

#include <stddef.h>
#include <gpxe/iobuf.h>
#include <gpxe/tcpip.h>
#include <gpxe/if_ether.h>
#include <gpxe/interface.h>

struct sockaddr_tcpip;
struct net_device;

/**
 * UDP constants
 */

#define UDP_MAX_HLEN	72
#define UDP_MAX_TXIOB	ETH_MAX_MTU
#define UDP_MIN_TXIOB	ETH_ZLEN

/**
 * A UDP header
 */
struct udp_header {
	/** Source port */
	uint16_t src;
	/** Destination port */
	uint16_t dest;
	/** Length */
	uint16_t len;
	/** Checksum */
	uint16_t chksum;
};

struct udp_interface;

/** UDP interface operations */
struct udp_interface_operations {
	/** Close interface
	 *
	 * @v udp		UDP interface
	 * @v rc		Reason for close
	 */
	void ( * close ) ( struct udp_interface *udp, int rc );
	/** Deliver datagram
	 *
	 * @v udp		UDP interface
	 * @v iobuf		Datagram I/O buffer
	 * @v peer		Peer address, or NULL
	 * @v local		Local address, or NULL
	 * @v netdev		Network device, or NULL
	 * @ret rc		Return status code
	 */
	int ( * deliver ) ( struct udp_interface *udp,
			    struct io_buffer *iobuf,
			    struct sockaddr_tcpip *peer,
			    struct sockaddr_tcpip *local,
			    struct net_device *netdev );
};

/** A UDP interface */
struct udp_interface {
	/** Generic object communication interface */
	struct interface intf;
	/** Operations for received messages */
	struct udp_interface_operations *op;
};

extern struct udp_interface null_udp;
extern struct udp_interface_operations null_udp_ops;

extern void udp_close ( struct udp_interface *udp, int rc );
extern int udp_deliver ( struct udp_interface *udp, struct io_buffer *iobuf,
			 struct sockaddr_tcpip *peer,
			 struct sockaddr_tcpip *local,
			 struct net_device *netdev );
extern void ignore_udp_close ( struct udp_interface *udp, int rc );
extern int ignore_udp_deliver ( struct udp_interface *udp,
				struct io_buffer *iobuf,
				struct sockaddr_tcpip *peer,
				struct sockaddr_tcpip *local,
				struct net_device *netdev );
extern int udp_open ( struct udp_interface *udp, struct sockaddr_tcpip *peer,
		      struct sockaddr_tcpip *local );
extern int udp_open_promisc ( struct udp_interface *udp );
extern int udp_open_named ( struct udp_interface *udp,
			    struct sockaddr_tcpip *peer,
			    const char *name, struct sockaddr_tcpip *local );
extern struct io_buffer * udp_alloc_iob ( size_t len );

/**
 * Initialise a UDP interface
 *
 * @v udp		UDP interface
 * @v op		UDP interface operations
 * @v refcnt		Containing object reference counter, or NULL
 */
static inline void udp_init ( struct udp_interface *udp,
			      struct udp_interface_operations *op,
			      struct refcnt *refcnt ) {
	udp->intf.dest = &null_udp.intf;
	udp->intf.refcnt = refcnt;
	udp->op = op;
}

/**
 * Initialise a static UDP interface
 *
 * @v _name		UDP interface
 * @v _op		UDP interface operations
 */
#define UDP_INIT( _name, _op ) {		\
		.intf = {			\
			.dest = &null_udp.intf,	\
			.refcnt = NULL,		\
		},				\
		.op = _op,			\
	}

/**
 * Get UDP interface from generic object communication interface
 *
 * @v intf		Generic object communication interface
 * @ret udp		UDP interface
 */
static inline __attribute__ (( always_inline )) struct udp_interface *
intf_to_udp ( struct interface *intf ) {
	return container_of ( intf, struct udp_interface, intf );
}

/**
 * Get reference to destination UDP interface
 *
 * @v udp		UDP interface
 * @ret dest		Destination interface
 */
static inline __attribute__ (( always_inline )) struct udp_interface *
udp_get_dest ( struct udp_interface *udp ) {
	return intf_to_udp ( intf_get ( udp->intf.dest ) );
}

/**
 * Drop reference to UDP interface
 *
 * @v udp		UDP interface
 */
static inline __attribute__ (( always_inline )) void
udp_put ( struct udp_interface *udp ) {
	intf_put ( &udp->intf );
}

/**
 * Plug a UDP interface into a new destination interface
 *
 * @v udp		UDP interface
 * @v dest		New destination interface
 */
static inline __attribute__ (( always_inline )) void
udp_plug ( struct udp_interface *udp, struct udp_interface *dest ) {
	plug ( &udp->intf, &dest->intf );
}

/**
 * Plug two UDP interfaces together
 *
 * @v a			UDP interface A
 * @v b			UDP interface B
 */
static inline __attribute__ (( always_inline )) void
udp_plug_plug ( struct udp_interface *a, struct udp_interface *b ) {
	plug_plug ( &a->intf, &b->intf );
}

/**
 * Unplug a UDP interface
 *
 * @v udp		UDP interface
 */
static inline __attribute__ (( always_inline )) void
udp_unplug ( struct udp_interface *udp ) {
	plug ( &udp->intf, &null_udp.intf );
}

/**
 * Stop using a UDP interface
 *
 * @v udp		UDP interface
 *
 * After calling this method, no further messages will be received via
 * the interface.
 */
static inline void udp_nullify ( struct udp_interface *udp ) {
	udp->op = &null_udp_ops;
};

#endif /* _GPXE_UDP_H */

