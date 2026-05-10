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

/** Receive queue requested queue size */
#define VIRTIO_NET_RX_COUNT 128

/** Receive queue maximum fill level */
#define VIRTIO_NET_RX_MAX 16

/** Transmit queue index */
#define VIRTIO_NET_TX_INDEX 1

/** Transmit queue requested queue size */
#define VIRTIO_NET_TX_COUNT 128

/** Transmit queue maximum fill level */
#define VIRTIO_NET_TX_MAX 32

/** A virtio network queue */
struct virtio_net_queue {
	/** Underlying virtio queue */
	struct virtio_queue queue;
	/** Requested queue size */
	unsigned int count;
	/** Maximum fill level */
	unsigned int max;
	/** Effective fill level */
	unsigned int fill;
	/** Buffer ID ring mask */
	unsigned int mask;
	/** Buffer ID ring */
	uint8_t *ids;
};

/**
 * Initialise virtio network queue
 *
 * @v queue		Virtio network queue
 * @v index		Queue index
 * @v count		Requested queue size
 * @v max		Maximum fill level
 * @v ids		Buffer ID ring
 */
static inline __attribute__ (( always_inline )) void
virtio_net_queue_init ( struct virtio_net_queue *queue, unsigned int index,
			unsigned int count, unsigned int max, uint8_t *ids ) {

	queue->queue.index = index;
	queue->count = count;
	queue->max = max;
	queue->ids = ids;
}

/** A virtio network device */
struct virtio_net {
	/** Underlying virtio device */
	struct virtio_device virtio;
	/** Receive queue */
	struct virtio_net_queue rx;
	/** Transmit queue */
	struct virtio_net_queue tx;
	/** Receive buffer ID ring */
	uint8_t rx_ids[VIRTIO_NET_RX_MAX];
	/** Transmit buffer ID ring */
	uint8_t tx_ids[VIRTIO_NET_TX_MAX];
};

#endif /* _VIRTIO_NET_H */
