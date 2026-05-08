#ifndef _VIRTIO_NET_H
#define _VIRTIO_NET_H

/** @file
 *
 * Virtual I/O network device
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );
FILE_SECBOOT ( PERMITTED );

#include <ipxe/virtio.h>

/** Receive queue index */
#define VIRTIO_NET_RX_INDEX 0

/** Requested number of receive descriptors */
#define VIRTIO_NET_RX_COUNT 128

/** Transmit queue index */
#define VIRTIO_NET_TX_INDEX 1

/** Requested number of transmit descriptors */
#define VIRTIO_NET_TX_COUNT 128

/** A virtio network device */
struct virtio_net {
	/** Underlying virtio device */
	struct virtio_device virtio;
	/** Receive queue */
	struct virtio_queue rx;
	/** Transmit queue */
	struct virtio_queue tx;
};

#endif /* _VIRTIO_NET_H */
