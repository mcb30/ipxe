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

/** A virtio network packet header */
union virtio_net_header {
	/** Legacy interface */
	uint8_t legacy[10];
	/** Modern (version 1.0) interface */
	uint8_t modern[12];
} __attribute__ (( packed ));

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

/** Number of descriptors per packet */
#define VIRTIO_NET_DESCS 2

/** A virtio network queue */
struct virtio_net_queue {
	/** Underlying virtio queue */
	struct virtio_queue queue;
	/** Descriptor slot ring */
	uint8_t *slot;
	/** Effective fill level */
	unsigned int fill;
	/** Descriptor index ring mask */
	unsigned int mask;

	/** Shared packet header */
	union virtio_net_header hdr;
	/** DMA mapping for packet header */
	struct dma_mapping map;

	/** DMA direction for packet header */
	uint8_t dma;
	/** Buffer writability flag for packet header */
	uint8_t write;
	/** Requested queue size */
	uint8_t count;
	/** Maximum fill level */
	uint8_t max;
};

/**
 * Initialise virtio network queue
 *
 * @v queue		Virtio network queue
 * @v index		Queue index
 * @v slot		Descriptor slot ring
 * @v dma		DMA direction for packet header
 * @v write		Writability flag for packet header
 * @v count		Requested queue size
 * @v max		Maximum fill level
 */
static inline __attribute__ (( always_inline )) void
virtio_net_queue_init ( struct virtio_net_queue *queue, uint8_t *slot,
			unsigned int index, unsigned int count,
			unsigned int max, unsigned int dma,
			unsigned int write ) {

	queue->queue.index = index;
	queue->slot = slot;
	queue->dma = dma;
	queue->write = write;
	queue->count = count;
	queue->max = max;
}

/** A virtio network device */
struct virtio_net {
	/** Underlying virtio device */
	struct virtio_device virtio;
	/** Receive queue */
	struct virtio_net_queue rx;
	/** Transmit queue */
	struct virtio_net_queue tx;
	/** Maximum frame size */
	size_t mfs;

	/** Receive descriptor slot ring */
	uint8_t rx_slot[VIRTIO_NET_RX_MAX];
	/** Receive I/O buffers */
	struct io_buffer *rx_iobuf[VIRTIO_NET_RX_MAX];

	/** Transmit descriptor slot ring */
	uint8_t tx_slot[VIRTIO_NET_TX_MAX];
	/** Transmit I/O buffers */
	struct io_buffer *tx_iobuf[VIRTIO_NET_TX_MAX];
};

#endif /* _VIRTIO_NET_H */
