#ifndef _IPXE_VIRTIO_H
#define _IPXE_VIRTIO_H

/** @file
 *
 * Virtual I/O Device
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );
FILE_SECBOOT ( PERMITTED );

#include <stdint.h>
#include <ipxe/dma.h>

/** Virtio page alignment */
#define VIRTIO_ALIGN 4096

/** Maximum time to wait for reset (in ms) */
#define VIRTIO_RESET_MAX_WAIT_MS 100

/**
 * @defgroup virtio_legacy Original ("legacy") common device registers
 * @{
 */

/** Legacy device supported features register */
#define VIRTIO_LEG_FEAT 0x00

/** Legacy negotiated in-use features register */
#define VIRTIO_LEG_USED 0x04

/** Legacy queue base address register */
#define VIRTIO_LEG_BASE 0x08

/** Legacy queue size register */
#define VIRTIO_LEG_SIZE 0x0c

/** Legacy queue select register */
#define VIRTIO_LEG_SEL 0x0e

/** Legacy queue doorbell notification register */
#define VIRTIO_LEG_DB 0x10

/** Legacy device status register */
#define VIRTIO_LEG_STAT 0x12
#define VIRTIO_STAT_FOUND	0x0001	/**< Guest has found device */
#define VIRTIO_STAT_DRIVER	0x0002	/**< Guest driver exists */
#define VIRTIO_STAT_READY	0x0004	/**< Guest driver is ready */
#define VIRTIO_STAT_FAIL	0x0080	/**< Guest driver has failed */

/** @} */

/**
 * @defgroup virtio_pci PCI common device registers
 * @{
 */

/** PCI device supported features select register */
#define VIRTIO_PCI_FEAT_SEL 0x00

/** PCI device supported features register */
#define VIRTIO_PCI_FEAT 0x04

/** PCI negotiated in-use features select register */
#define VIRTIO_PCI_USED_SEL 0x08

/** PCI negotiated in-use features register */
#define VIRTIO_PCI_USED 0x0c

/** PCI device status register */
#define VIRTIO_PCI_STAT 0x14

/** PCI configuration generation register */
#define VIRTIO_PCI_GEN 0x15

/** PCI queue select register */
#define VIRTIO_PCI_SEL 0x16

/** PCI queue size register */
#define VIRTIO_PCI_SIZE 0x18

/** PCI queue enable register */
#define VIRTIO_PCI_ENABLE 0x1c

/** PCI queue doorbell notification offset register */
#define VIRTIO_PCI_DBOFF 0x1e

/** PCI queue buffer descriptor array base address register */
#define VIRTIO_PCI_BUFS 0x20

/** PCI queue submission queue base address register */
#define VIRTIO_PCI_SQES 0x28

/** PCI queue completion queue base address register */
#define VIRTIO_PCI_CQES 0x30

/** @} */

/** A virtio buffer descriptor */
struct virtio_buf {
	/** Buffer address */
	uint64_t addr;
	/** Buffer length */
	uint32_t len;
	/** Flags */
	uint16_t flags;
	/** Next buffer index */
	uint16_t next;
} __attribute__ (( packed ));

/** Next buffer index is valid */
#define VIRTIO_BUF_FL_NEXT 0x0001

/** Buffer is write-only */
#define VIRTIO_BUF_FL_WRITE 0x0002

/** A virtio submission queue entry */
struct virtio_sqe {
	/** Starting buffer index */
	uint16_t index;
} __attribute__ (( packed ));

/** A virtio submission ("available") queue */
struct virtio_sq {
	/** Flags */
	uint16_t flags;
	/** Producer index */
	uint16_t index;
	/** Queue entries */
	struct virtio_sqe sqe[];
} __attribute__ (( packed ));

/** Do not generate interrupt */
#define VIRTIO_SQ_FL_NO_INTERRUPT 0x0001

/** A virtio completion queue entry */
struct virtio_cqe {
	/** Starting buffer index */
	uint32_t index;
	/** Length written */
	uint32_t len;
} __attribute__ (( packed ));

/** A virtio completion ("used") queue */
struct virtio_cq {
	/** Flags */
	uint16_t flags;
	/** Consumer index */
	uint16_t index;
	/** Queue entries */
	struct virtio_cqe cqe[];
} __attribute__ (( packed ));

/** A virtio queue */
struct virtio_queue {
	/** Queue index */
	unsigned int index;
	/** Queue size (must be a power of two) */
	unsigned int count;
	/** Total length of queue */
	size_t len;
	/** DMA mapping */
	struct dma_mapping map;
	/** Buffer descriptor array (and start of DMA allocation) */
	struct virtio_buf *buf;
	/** Submission queue */
	struct virtio_sq *sq;
	/** Completion queue */
	struct virtio_cq *cq;
};

/**
 * Initialise virtio queue
 *
 * @v vq		Virtio queue
 * @v index		Queue index
 */
static inline __attribute__ (( always_inline )) void
virtio_queue_init ( struct virtio_queue *vq, unsigned int index ) {

	vq->index = index;
}

/** A virtio feature set */
struct virtio_features {
	/** Feature bits */
	uint32_t feat[2];
};

/** A virtio device */
struct virtio_device {
	/** Device name */
	const char *name;
	/** Device operations */
	struct virtio_operations *op;
	/** Common registers */
	void *common;
	/** Device-specific registers */
	void *device;
	/** Device supported features */
	struct virtio_features supported;
	/** Negotiated features */
	struct virtio_features negotiated;
};

/** Virtio device operations */
struct virtio_operations {
	/**
	 * Set device status
	 *
	 * @v virtio		Virtio device
	 * @v stat		Device status (0 to reset device)
	 */
	void ( * status ) ( struct virtio_device *virtio, unsigned int stat );
	/**
	 * Get supported features
	 *
	 * @v virtio		Virtio device
	 */
	void ( * supported ) ( struct virtio_device *virtio );
	/**
	 * Set negotiated features
	 *
	 * @v virtio		Virtio device
	 */
	void ( * negotiate ) ( struct virtio_device *virtio );
	/**
	 * Set queue size
	 *
	 * @v virtio		Virtio device
	 * @v queue		Virtio queue
	 * @v count		Requested size
	 */
	void ( * size ) ( struct virtio_device *virtio,
			  struct virtio_queue *queue, unsigned int count );
	/**
	 * Enable queue
	 *
	 * @v virtio		Virtio device
	 * @v queue		Virtio queue
	 * @ret rc		Return status code
	 */
	int ( * enable ) ( struct virtio_device *virtio,
			   struct virtio_queue *queue );
	/**
	 * Disable queue
	 *
	 * @v virtio		Virtio device
	 * @v queue		Virtio queue
	 */
	void ( * disable ) ( struct virtio_device *virtio,
			     struct virtio_queue *queue );
};

#endif /* _IPXE_VIRTIO_H */
