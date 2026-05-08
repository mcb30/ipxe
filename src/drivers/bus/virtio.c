/*
 * Copyright (C) 2026 Michael Brown <mbrown@fensystems.co.uk>.
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * You can also choose to distribute this program under the terms of
 * the Unmodified Binary Distribution Licence (as given in the file
 * COPYING.UBDL), provided that you have satisfied its requirements.
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );
FILE_SECBOOT ( PERMITTED );

/** @file
 *
 * Virtual I/O Device
 *
 */

#include <assert.h>
#include <unistd.h>
#include <ipxe/virtio.h>

/******************************************************************************
 *
 * Original ("legacy") device operations
 *
 ******************************************************************************
 */

/**
 * Set device status
 *
 * @v virtio		Virtio device
 * @v stat		Device status (0 to reset device)
 */
static void virtio_legacy_status ( struct virtio_device *virtio,
				   unsigned int stat ) {

	/* Write to status register */
	iowrite8 ( stat, virtio->common + VIRTIO_LEG_STAT );
}

/**
 * Get supported features
 *
 * @v virtio		Virtio device
 */
static void virtio_legacy_supported ( struct virtio_device *virtio ) {
	struct virtio_features *supported = &virtio->supported;
	unsigned int i;

	/* Get device supported features */
	supported->feat[0] = ioread32 ( virtio->common + VIRTIO_LEG_FEAT );

	/* Legacy devices have only a single 32-bit feature register */
	for ( i = 1 ; i < ( sizeof ( supported->feat ) /
			    sizeof ( supported->feat[0] ) ) ; i++ ) {
		supported->feat[i] = 0;
	}
}

/**
 * Negotiate device features
 *
 * @v virtio		Virtio device
 */
static void virtio_legacy_negotiate ( struct virtio_device *virtio ) {
	struct virtio_features *used = &virtio->negotiated;
	unsigned int i;

	/* Set device supported features */
	iowrite32 ( used->feat[0], virtio->common + VIRTIO_LEG_FEAT );

	/* Legacy devices have only a single 32-bit feature register */
	for ( i = 1 ; i < ( sizeof ( used->feat ) /
			    sizeof ( used->feat[0] ) ) ; i++ ) {
		assert ( used->feat[i] == 0 );
	}
}

/**
 * Set queue size
 *
 * @v virtio		Virtio device
 * @v queue		Virtio queue
 * @v count		Requested size
 */
static void virtio_legacy_size ( struct virtio_device *virtio,
				 struct virtio_queue *queue,
				 unsigned int count ) {
	struct virtio_buf *buf;
	struct virtio_sq *sq;
	struct virtio_cq *cq;
	size_t len;

	/* Select queue */
	iowrite16 ( queue->index, virtio->common + VIRTIO_LEG_SEL );

	/* Get (fixed) queue size */
	count = ioread16 ( virtio->common + VIRTIO_LEG_SIZE );

	/* Calculate queue length */
	len = ( count * sizeof ( buf[0] ) );
	len += ( sizeof ( *sq ) + ( count * sizeof ( sq->sqe[0] ) ) );
	len = ( ( len + VIRTIO_ALIGN - 1 ) & ~( VIRTIO_ALIGN - 1 ) );
	len += ( sizeof ( *cq ) + ( count * sizeof ( cq->cqe[0] ) ) );
	len = ( ( len + VIRTIO_ALIGN - 1 ) & ~( VIRTIO_ALIGN - 1 ) );

	/* Record queue size */
	queue->count = count;
	queue->len = len;
}

/**
 * Enable queue
 *
 * @v virtio		Virtio device
 * @v queue		Virtio queue
 * @ret rc		Return status code
 */
static int virtio_legacy_enable ( struct virtio_device *virtio,
				  struct virtio_queue *queue ) {
	struct virtio_buf *buf;
	struct virtio_sq *sq;
	struct virtio_cq *cq;
	unsigned int count;
	void *base;
	size_t len;

	/* Select queue */
	iowrite16 ( queue->index, virtio->common + VIRTIO_LEG_SEL );

	/* Program queue base page address */
	base = queue->buf;
	iowrite32 ( ( dma ( &queue->map, base ) / VIRTIO_ALIGN ),
		    virtio->common + VIRTIO_LEG_BASE );

	/* Lay out queue regions */
	count = queue->count;
	len = ( count * sizeof ( buf[0] ) );
	queue->sq = ( base + len );
	len += ( sizeof ( *sq ) + ( count * sizeof ( sq->sqe[0] ) ) );
	len = ( ( len + VIRTIO_ALIGN - 1 ) & ~( VIRTIO_ALIGN - 1 ) );
	queue->cq = ( base + len );
	len += ( sizeof ( *cq ) + ( count * sizeof ( cq->cqe[0] ) ) );
	len = ( ( len + VIRTIO_ALIGN - 1 ) & ~( VIRTIO_ALIGN - 1 ) );
	assert ( len == queue->len );

	return 0;
}

/**
 * Disable queue
 *
 * @v virtio		Virtio device
 * @v queue		Virtio queue
 */
static void virtio_legacy_disable ( struct virtio_device *virtio,
				    struct virtio_queue *queue ) {
	unsigned int i;
	uint32_t base;

	/* Select queue */
	iowrite16 ( queue->index, virtio->common + VIRTIO_LEG_SEL );

	/* Zero queue base page address */
	iowrite32 ( 0, virtio->common + VIRTIO_LEG_BASE );

	/* Wait for queue to be disabled */
	for ( i = 0 ; i < VIRTIO_RESET_MAX_WAIT_MS ; i++ ) {
		base = ioread32 ( virtio->common + VIRTIO_LEG_BASE );
		if ( ! base )
			return;
		mdelay ( 1 );
	}

	DBGC ( virtio, "VIRTIO %s could not disable queue %d\n",
	       virtio->name, queue->index );
}

/** Original ("legacy") device operations */
static struct virtio_operations virtio_legacy_operations = {
	.status = virtio_legacy_status,
	.supported = virtio_legacy_supported,
	.negotiate = virtio_legacy_negotiate,
	.size = virtio_legacy_size,
	.enable = virtio_legacy_enable,
	.disable = virtio_legacy_disable,
};

/******************************************************************************
 *
 * PCI ("modern") device operations
 *
 ******************************************************************************
 */

/**
 * Set device status
 *
 * @v virtio		Virtio device
 * @v stat		Device status (0 to reset device)
 */
static void virtio_pci_status ( struct virtio_device *virtio,
				unsigned int stat ) {

	/* Write to status register */
	iowrite8 ( stat, virtio->common + VIRTIO_PCI_STAT );
}

/**
 * Get supported features
 *
 * @v virtio		Virtio device
 */
static void virtio_pci_supported ( struct virtio_device *virtio ) {
	struct virtio_features *supported = &virtio->supported;
	unsigned int i;

	/* Get device supported features */
	for ( i = 0 ; i < ( sizeof ( supported->feat ) /
			    sizeof ( supported->feat[0] ) ) ; i++ ) {
		iowrite32 ( i, virtio->common + VIRTIO_PCI_FEAT_SEL );
		supported->feat[i] =
			ioread32 ( virtio->common + VIRTIO_PCI_FEAT );
	}
}

/**
 * Negotiate device features
 *
 * @v virtio		Virtio device
 */
static void virtio_pci_negotiate ( struct virtio_device *virtio ) {
	struct virtio_features *used = &virtio->negotiated;
	unsigned int i;

	/* Set device supported features */
	for ( i = 0 ; i < ( sizeof ( used->feat ) /
			    sizeof ( used->feat[0] ) ) ; i++ ) {
		iowrite32 ( i, virtio->common + VIRTIO_PCI_USED_SEL );
		iowrite32 ( used->feat[i], virtio->common + VIRTIO_PCI_USED );
	}
}



/** PCI ("modern") device operations */
static struct virtio_operations virtio_pci_operations = {
	.status = virtio_pci_status,
	.supported = virtio_pci_supported,
	.negotiate = virtio_pci_negotiate,
	.size = virtio_pci_size,
	.enable = virtio_pci_enable,
	.disable = virtio_pci_disable,
};



/******************************************************************************
 *
 * Transport-independent operations
 *
 ******************************************************************************
 */

/**
 * Negotiate features
 *
 * @v virtio		Virtio device
 * @v driver		Driver supported features
 */
void virtio_negotiate ( struct virtio_device *virtio,
			const struct virtio_features *driver ) {
	struct virtio_features *supported = &virtio->supported;
	struct virtio_features *negotiated = &virtio->negotiated;
	unsigned int i;

	/* Get device supported features */
	virtio->op->supported ( virtio );
	DBGC ( virtio, "VIRTIO %s supported ", virtio->name );
	for ( i = 0 ; i < ( sizeof ( supported->feat ) /
			    sizeof ( supported->feat[0] ) ) ; i++ ) {
		DBGC ( virtio, "%s%08x", ( i ? ":" : "" ),
		       supported->feat[i] );
	}
	DBGC ( virtio, "\n" );

	/* Negotiate features */
	DBGC ( virtio, "VIRTIO %s negotiated ", virtio->name );
	for ( i = 0 ; i < ( sizeof ( supported->feat ) /
			    sizeof ( supported->feat[0] ) ) ; i++ ) {
		negotiated->feat[i] = ( supported->feat[i] & driver->feat[i] );
		DBGC ( virtio, "%s%08x", ( i ? ":" : "" ),
		       negotiated->feat[i] );
	}
	DBGC ( virtio, "\n" );
	virtio->op->negotiate ( virtio );
}
