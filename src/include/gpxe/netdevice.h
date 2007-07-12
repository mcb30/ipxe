#ifndef _GPXE_NETDEVICE_H
#define _GPXE_NETDEVICE_H

/** @file
 *
 * Network device management
 *
 */

#include <stdint.h>
#include <gpxe/list.h>
#include <gpxe/tables.h>
#include <gpxe/refcnt.h>

struct io_buffer;
struct net_device;
struct net_protocol;
struct ll_protocol;
struct device;

/** Maximum length of a link-layer address */
#define MAX_LL_ADDR_LEN 6

/** Maximum length of a link-layer header */
#define MAX_LL_HEADER_LEN 16

/** Maximum length of a network-layer address */
#define MAX_NET_ADDR_LEN 4

/**
 * A network-layer protocol
 *
 */
struct net_protocol {
	/** Protocol name */
	const char *name;
	/**
	 * Process received packet
	 *
	 * @v iobuf	I/O buffer
	 * @v netdev	Network device
	 * @v ll_source	Link-layer source address
	 *
	 * This method takes ownership of the I/O buffer.
	 */
	int ( * rx ) ( struct io_buffer *iobuf, struct net_device *netdev,
		       const void *ll_source );
	/**
	 * Transcribe network-layer address
	 *
	 * @v net_addr	Network-layer address
	 * @ret string	Human-readable transcription of address
	 *
	 * This method should convert the network-layer address into a
	 * human-readable format (e.g. dotted quad notation for IPv4).
	 *
	 * The buffer used to hold the transcription is statically
	 * allocated.
	 */
	const char * ( *ntoa ) ( const void * net_addr );
	/** Network-layer protocol
	 *
	 * This is an ETH_P_XXX constant, in network-byte order
	 */
	uint16_t net_proto;
	/** Network-layer address length */
	uint8_t net_addr_len;
};

/**
 * A link-layer protocol
 *
 */
struct ll_protocol {
	/** Protocol name */
	const char *name;
	/**
	 * Transmit network-layer packet via network device
	 *
	 * @v iobuf		I/O buffer
	 * @v netdev		Network device
	 * @v net_protocol	Network-layer protocol
	 * @v ll_dest		Link-layer destination address
	 * @ret rc		Return status code
	 *
	 * This method should prepend in the link-layer header
	 * (e.g. the Ethernet DIX header) and transmit the packet.
	 * This method takes ownership of the I/O buffer.
	 */
	int ( * tx ) ( struct io_buffer *iobuf, struct net_device *netdev,
		       struct net_protocol *net_protocol,
		       const void *ll_dest );
	/**
	 * Handle received packet
	 *
	 * @v iobuf	I/O buffer
	 * @v netdev	Network device
	 *
	 * This method should strip off the link-layer header
	 * (e.g. the Ethernet DIX header) and pass the packet to
	 * net_rx().  This method takes ownership of the packet
	 * buffer.
	 */
	int ( * rx ) ( struct io_buffer *iobuf, struct net_device *netdev );
	/**
	 * Transcribe link-layer address
	 *
	 * @v ll_addr	Link-layer address
	 * @ret string	Human-readable transcription of address
	 *
	 * This method should convert the link-layer address into a
	 * human-readable format.
	 *
	 * The buffer used to hold the transcription is statically
	 * allocated.
	 */
	const char * ( * ntoa ) ( const void * ll_addr );
	/** Link-layer protocol
	 *
	 * This is an ARPHRD_XXX constant, in network byte order.
	 */
	uint16_t ll_proto;
	/** Link-layer address length */
	uint8_t ll_addr_len;
	/** Link-layer header length */
	uint8_t ll_header_len;
	/** Link-layer broadcast address */
	const uint8_t *ll_broadcast;
};

/** Network device operations */
struct net_device_operations {
	/** Open network device
	 *
	 * @v netdev	Network device
	 * @ret rc	Return status code
	 *
	 * This method should allocate RX I/O buffers and enable
	 * the hardware to start transmitting and receiving packets.
	 */
	int ( * open ) ( struct net_device *netdev );
	/** Close network device
	 *
	 * @v netdev	Network device
	 *
	 * This method should stop the flow of packets, and free up
	 * any packets that are currently in the device's TX queue.
	 */
	void ( * close ) ( struct net_device *netdev );
	/** Transmit packet
	 *
	 * @v netdev	Network device
	 * @v iobuf	I/O buffer
	 * @ret rc	Return status code
	 *
	 * This method should cause the hardware to initiate
	 * transmission of the I/O buffer.
	 *
	 * If this method returns success, the I/O buffer remains
	 * owned by the net device's TX queue, and the net device must
	 * eventually call netdev_tx_complete() to free the buffer.
	 * If this method returns failure, the I/O buffer is
	 * immediately released; the failure is interpreted as
	 * "failure to enqueue buffer".
	 *
	 * This method is guaranteed to be called only when the device
	 * is open.
	 */
	int ( * transmit ) ( struct net_device *netdev,
			     struct io_buffer *iobuf );
	/** Poll for completed and received packets
	 *
	 * @v netdev	Network device
	 *
	 * This method should cause the hardware to check for
	 * completed transmissions and received packets.  Any received
	 * packets should be delivered via netdev_rx().
	 *
	 * This method is guaranteed to be called only when the device
	 * is open.
	 */
	void ( * poll ) ( struct net_device *netdev );
	/** Enable or disable interrupts
	 *
	 * @v netdev	Network device
	 * @v enable	Interrupts should be enabled
	 */
	void ( * irq ) ( struct net_device *netdev, int enable );
};

/** Network device error breakdown */
struct net_device_error {
	/** Error number */
	int errno;
	/** Error count */
	unsigned int count;
};

/** Network device statistics */
struct net_device_stats {
	/** Count of successfully completed transmissions */
	unsigned int tx_ok;
	/** Count of transmission errors */
	unsigned int tx_err;
	/** Count of successfully received packets */
	unsigned int rx_ok;
	/** Count of reception errors */
	unsigned int rx_err;
};

/**
 * A network device
 *
 * This structure represents a piece of networking hardware.  It has
 * properties such as a link-layer address and methods for
 * transmitting and receiving raw packets.
 *
 * Note that this structure must represent a generic network device,
 * not just an Ethernet device.
 */
struct net_device {
	/** Reference counter */
	struct refcnt refcnt;
	/** List of network devices */
	struct list_head list;
	/** Name of this network device */
	char name[8];
	/** Underlying hardware device */
	struct device *dev;

	/** Network device operations */
	struct net_device_operations *op;

	/** Link-layer protocol */
	struct ll_protocol *ll_protocol;
	/** Link-layer address
	 *
	 * For Ethernet, this is the MAC address.
	 */
	uint8_t ll_addr[MAX_LL_ADDR_LEN];

	/** Current device state
	 *
	 * This is the bitwise-OR of zero or more NETDEV_XXX constants.
	 */
	unsigned int state;
	/** TX packet queue */
	struct list_head tx_queue;
	/** RX packet queue */
	struct list_head rx_queue;
	/** Device statistics */
	struct net_device_stats stats;

	/** Driver private data */
	void *priv;
};

/** Network device is open */
#define NETDEV_OPEN 0x0001

/** Declare a link-layer protocol */
#define __ll_protocol  __table ( struct ll_protocol, ll_protocols, 01 )

/** Declare a network-layer protocol */
#define __net_protocol __table ( struct net_protocol, net_protocols, 01 )

extern struct list_head net_devices;
extern struct net_device_operations null_netdev_operations;

/**
 * Initialise a network device
 *
 * @v netdev		Network device
 * @v op		Network device operations
 */
static inline void netdev_init ( struct net_device *netdev,
				 struct net_device_operations *op ) {
	netdev->op = op;
}

/**
 * Stop using a network device
 *
 * @v netdev		Network device
 *
 * Drivers should call this method immediately before the final call
 * to netdev_put().
 */
static inline void netdev_nullify ( struct net_device *netdev ) {
	netdev->op = &null_netdev_operations;
}

/**
 * Get printable network device hardware address
 *
 * @v netdev		Network device
 * @ret name		Hardware address
 */
static inline const char * netdev_hwaddr ( struct net_device *netdev ) {
	return netdev->ll_protocol->ntoa ( netdev->ll_addr );
}

/** Iterate over all network devices */
#define for_each_netdev( netdev ) \
	list_for_each_entry ( (netdev), &net_devices, list )

/** There exist some network devices
 *
 * @ret existence	Existence of network devices
 */
static inline int have_netdevs ( void ) {
	return ( ! list_empty ( &net_devices ) );
}

/**
 * Get reference to network device
 *
 * @v netdev		Network device
 * @ret netdev		Network device
 */
static inline __attribute__ (( always_inline )) struct net_device *
netdev_get ( struct net_device *netdev ) {
	ref_get ( &netdev->refcnt );
	return netdev;
}

/**
 * Drop reference to network device
 *
 * @v netdev		Network device
 */
static inline __attribute__ (( always_inline )) void
netdev_put ( struct net_device *netdev ) {
	ref_put ( &netdev->refcnt );
}

extern int netdev_tx ( struct net_device *netdev, struct io_buffer *iobuf );
extern void netdev_tx_complete_err ( struct net_device *netdev,
				 struct io_buffer *iobuf, int rc );
extern void netdev_tx_complete_next_err ( struct net_device *netdev, int rc );
extern void netdev_rx ( struct net_device *netdev, struct io_buffer *iobuf );
extern void netdev_rx_err ( struct net_device *netdev,
			    struct io_buffer *iobuf, int rc );
extern void netdev_poll ( struct net_device *netdev );
extern struct io_buffer * netdev_rx_dequeue ( struct net_device *netdev );
extern struct net_device * alloc_netdev ( size_t priv_size );
extern int register_netdev ( struct net_device *netdev );
extern int netdev_open ( struct net_device *netdev );
extern void netdev_close ( struct net_device *netdev );
extern void unregister_netdev ( struct net_device *netdev );
extern void netdev_irq ( struct net_device *netdev, int enable );
extern struct net_device * find_netdev ( const char *name );
extern struct net_device * find_netdev_by_location ( unsigned int bus_type,
						     unsigned int location );
extern int net_tx ( struct io_buffer *iobuf, struct net_device *netdev,
		    struct net_protocol *net_protocol, const void *ll_dest );
extern int net_rx ( struct io_buffer *iobuf, struct net_device *netdev,
		    uint16_t net_proto, const void *ll_source );

/**
 * Complete network transmission
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 *
 * The packet must currently be in the network device's TX queue.
 */
static inline void netdev_tx_complete ( struct net_device *netdev,
					struct io_buffer *iobuf ) {
	netdev_tx_complete_err ( netdev, iobuf, 0 );
}

/**
 * Complete network transmission
 *
 * @v netdev		Network device
 *
 * Completes the oldest outstanding packet in the TX queue.
 */
static inline void netdev_tx_complete_next ( struct net_device *netdev ) {
	netdev_tx_complete_next_err ( netdev, 0 );
}

#endif /* _GPXE_NETDEVICE_H */
