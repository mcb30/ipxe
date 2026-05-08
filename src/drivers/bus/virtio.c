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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <ipxe/pci.h>
#include <ipxe/virtio.h>

/******************************************************************************
 *
 * Original ("legacy") device operations
 *
 ******************************************************************************
 */

/**
 * Reset device
 *
 * @v virtio		Virtio device
 * @ret rc		Return status code
 */
static int virtio_legacy_reset ( struct virtio_device *virtio ) {
	uint8_t stat;
	unsigned int i;

	/* Reset device */
	iowrite8 ( 0, virtio->common + VIRTIO_LEG_STAT );

	/* Wait for reset to complete */
	for ( i = 0 ; i < VIRTIO_RESET_MAX_WAIT_MS ; i++ ) {
		stat = ioread8 ( virtio->common + VIRTIO_LEG_STAT );
		if ( ! stat )
			return 0;
		mdelay ( 1 );
	}

	DBGC ( virtio, "VIRTIO %s could not reset device\n", virtio->name );
	return -ETIMEDOUT;
}

/**
 * Report driver status
 *
 * @v virtio		Virtio device
 */
static void virtio_legacy_status ( struct virtio_device *virtio ) {

	/* Report device status */
	iowrite8 ( virtio->stat, virtio->common + VIRTIO_LEG_STAT );
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
	size_t len;

	/* Select queue */
	iowrite16 ( queue->index, virtio->common + VIRTIO_LEG_SEL );

	/* Get (fixed) queue size */
	count = ioread16 ( virtio->common + VIRTIO_LEG_SIZE );

	/* Calculate queue length */
	len = virtio_desc_size ( count );
	len = virtio_align ( len + virtio_sq_size ( count ) );
	len = virtio_align ( len + virtio_cq_size ( count ) );

	/* Record queue size */
	queue->count = count;
	queue->len = len;
}

/**
 * Enable queue
 *
 * @v virtio		Virtio device
 * @v queue		Virtio queue
 */
static void virtio_legacy_enable ( struct virtio_device *virtio,
				   struct virtio_queue *queue ) {
	unsigned int count = queue->count;
	void *base = queue->desc;
	size_t len;

	/* Select queue */
	iowrite16 ( queue->index, virtio->common + VIRTIO_LEG_SEL );

	/* Lay out queue regions */
	len = virtio_desc_size ( count );
	queue->sq = ( base + len );
	len = virtio_align ( len + virtio_sq_size ( count ) );
	queue->cq = ( base + len );
	len = virtio_align ( len + virtio_cq_size ( count ) );
	assert ( len == queue->len );

	/* Program queue base page address */
	iowrite32 ( ( dma ( &queue->map, queue->desc ) / VIRTIO_PAGE ),
		    virtio->common + VIRTIO_LEG_BASE );
}

/** Original ("legacy") device operations */
static struct virtio_operations virtio_legacy_operations = {
	.reset = virtio_legacy_reset,
	.status = virtio_legacy_status,
	.supported = virtio_legacy_supported,
	.negotiate = virtio_legacy_negotiate,
	.size = virtio_legacy_size,
	.enable = virtio_legacy_enable,
};

/**
 * Map legacy device
 *
 * @v virtio		Virtio device
 * @v pci		PCI device
 * @ret rc		Return status code
 */
static int virtio_legacy_map ( struct virtio_device *virtio,
			       struct pci_device *pci ) {
	int rc;

	/* Map common device region */
	if ( pci->ioaddr ) {
		// update pci_ioremap() to handle I/O addresses
		virtio->common = ( ( void * ) pci->ioaddr );
	} else {
		virtio->common = pci_ioremap ( pci, pci->membase,
					       VIRTIO_PAGE );
	}
	if ( ! virtio->common ) {
		DBGC ( virtio, "VIRTIO %s has no BARs\n", virtio->name );
		rc = -ENODEV;
		goto err_common;
	}

	/* Set device operations */
	virtio->op = &virtio_legacy_operations;

	return 0;

	iounmap ( virtio->common );
 err_common:
	return rc;
}

/******************************************************************************
 *
 * PCI ("modern") device operations
 *
 ******************************************************************************
 */

/**
 * Reset device
 *
 * @v virtio		Virtio device
 * @ret rc		Return status code
 */
static int virtio_pci_reset ( struct virtio_device *virtio ) {
	uint8_t stat;
	unsigned int i;

	/* Reset device */
	iowrite8 ( 0, virtio->common + VIRTIO_PCI_STAT );

	/* Wait for reset to complete */
	for ( i = 0 ; i < VIRTIO_RESET_MAX_WAIT_MS ; i++ ) {
		stat = ioread8 ( virtio->common + VIRTIO_PCI_STAT );
		if ( ! stat )
			return 0;
		mdelay ( 1 );
	}

	DBGC ( virtio, "VIRTIO %s could not reset device\n", virtio->name );
	return -ETIMEDOUT;
}

/**
 * Report driver status
 *
 * @v virtio		Virtio device
 */
static void virtio_pci_status ( struct virtio_device *virtio ) {

	/* Report device status */
	iowrite8 ( virtio->stat, virtio->common + VIRTIO_PCI_STAT );
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

/**
 * Set queue size
 *
 * @v virtio		Virtio device
 * @v queue		Virtio queue
 * @v count		Requested size
 */
static void virtio_pci_size ( struct virtio_device *virtio,
			      struct virtio_queue *queue,
			      unsigned int count ) {
	unsigned int max;
	size_t len;

	/* Select queue */
	iowrite16 ( queue->index, virtio->common + VIRTIO_PCI_SEL );

	/* Set queue size */
	max = ioread16 ( virtio->common + VIRTIO_PCI_SIZE );
	if ( count > max )
		count = max;
	iowrite16 ( count, virtio->common + VIRTIO_PCI_SIZE );

	/* Calculate queue length */
	len = virtio_align ( virtio_desc_size ( count ) );
	len = virtio_align ( len + virtio_sq_size ( count ) );
	len = virtio_align ( len + virtio_cq_size ( count ) );

	/* Record queue size */
	queue->count = count;
	queue->len = len;
}

/**
 * Program queue address
 *
 * @v virtio		Virtio device
 * @v queue		Virtio queue
 * @v addr		Address
 * @v offset		Register offset
 */
static void virtio_pci_address ( struct virtio_device *virtio,
				 struct virtio_queue *queue,
				 void *addr, unsigned int offset ) {
	physaddr_t phys;

	/* Program address */
	phys = dma ( &queue->map, addr );
	iowrite32 ( ( phys & 0xffffffffUL ), ( virtio->common + offset + 0 ) );
	if ( sizeof ( physaddr_t ) > sizeof ( uint32_t ) ) {
		iowrite32 ( ( ( ( uint64_t ) phys ) >> 32 ),
			    ( virtio->common + offset + 4 ) );
	} else {
		iowrite32 ( 0, ( virtio->common + offset + 4 ) );
	}
}

/**
 * Enable queue
 *
 * @v virtio		Virtio device
 * @v queue		Virtio queue
 */
static void virtio_pci_enable ( struct virtio_device *virtio,
				struct virtio_queue *queue ) {
	unsigned int count = queue->count;
	void *base = queue->desc;
	size_t len;

	/* Select queue */
	iowrite16 ( queue->index, virtio->common + VIRTIO_PCI_SEL );

	/* Lay out queue regions */
	len = virtio_align ( virtio_desc_size ( count ) );
	queue->sq = ( base + len );
	len = virtio_align ( len + virtio_sq_size ( count ) );
	queue->cq = ( base + len );
	len = virtio_align ( len + virtio_cq_size ( count ) );
	assert ( len == queue->len );

	/* Program queue addresses */
	virtio_pci_address ( virtio, queue, queue->desc, VIRTIO_PCI_DESC );
	virtio_pci_address ( virtio, queue, queue->sq, VIRTIO_PCI_SQ );
	virtio_pci_address ( virtio, queue, queue->cq, VIRTIO_PCI_CQ );

	/* Enable queue */
	iowrite16 ( 1, virtio->common + VIRTIO_PCI_ENABLE );
}

/** PCI ("modern") device operations */
static struct virtio_operations virtio_pci_operations = {
	.reset = virtio_pci_reset,
	.status = virtio_pci_status,
	.supported = virtio_pci_supported,
	.negotiate = virtio_pci_negotiate,
	.size = virtio_pci_size,
	.enable = virtio_pci_enable,
};

/**
 * Map PCI capability
 *
 * @v virtio		Virtio device
 * @v pci		PCI device
 * @v type		Capability type
 * @ret io_addr		I/O address, or NULL on error
 */
static void * virtio_pci_map_cap ( struct virtio_device *virtio,
				   struct pci_device *pci,
				   unsigned int type ) {
	uint8_t len;
	uint8_t typ;
	uint8_t bar;
	uint32_t offset;
	unsigned int reg;
	unsigned long start;
	void *io_addr;
	int pos;

	/* Scan through vendor capabilities */
	for ( pos = pci_find_capability ( pci, PCI_CAP_ID_VNDR ) ; pos > 0 ;
	      pos = pci_find_next_capability ( pci, pos, PCI_CAP_ID_VNDR ) ) {

		/* Check length */
		pci_read_config_byte ( pci, ( pos + PCI_CAP_LEN ), &len );
		if ( len < VIRTIO_PCI_CAP_END ) {
			DBGC ( virtio, "VIRTIO %s capability +%#02x too "
			       "short (%zd bytes)\n", virtio->name, pos, len );
			continue;
		}

		/* Read values */
		pci_read_config_byte ( pci, ( pos + VIRTIO_PCI_CAP_TYPE ),
				       &typ );
		pci_read_config_byte ( pci, ( pos + VIRTIO_PCI_CAP_BAR ),
				       &bar );
		pci_read_config_dword ( pci, ( pos + VIRTIO_PCI_CAP_OFFSET ),
					&offset );

		/* Check type */
		if ( typ != type )
			continue;
		DBGC2 ( virtio, "VIRTIO %s capability type %d BAR%d+%#x\n",
			virtio->name, type, bar, offset );

		/* Check BAR */
		reg = PCI_BASE_ADDRESS ( bar );
		if ( reg > PCI_BASE_ADDRESS_5 )
			continue;
		start = pci_bar_start ( pci, reg );
		if ( ! start )
			continue;

		/* Map BAR region */
		io_addr = pci_ioremap ( pci, start, VIRTIO_PAGE );
		if ( ! io_addr )
			continue;

		return io_addr;
	}

	DBGC ( virtio, "VIRTIO %s has no usable capability type %d\n",
	       virtio->name, type );
	return NULL;
}

/**
 * Map PCI device
 *
 * @v virtio		Virtio device
 * @v pci		PCI device
 * @ret rc		Return status code
 */
int virtio_pci_map ( struct virtio_device *virtio, struct pci_device *pci ) {
	int rc;

	/* Initialise device */
	virtio->name = pci->dev.name;
	virtio->dma = &pci->dma;

	/* Try mapping common capability */
	virtio->common = virtio_pci_map_cap ( virtio, pci,
					      VIRTIO_PCI_CAP_TYPE_COMMON );
	if ( ! virtio->common ) {
		/* Assume this must be a legacy device */
		return virtio_legacy_map ( virtio, pci );
	}

	/* Map device capability */
	virtio->device = virtio_pci_map_cap ( virtio, pci,
					      VIRTIO_PCI_CAP_TYPE_DEVICE );
	if ( ! virtio->device ) {
		rc = -ENODEV;
		goto err_device;
	}

	/* Set device operations and DMA mask */
	virtio->op = &virtio_pci_operations;
	dma_set_mask_64bit ( virtio->dma );

	return 0;

	iounmap ( virtio->device );
 err_device:
	iounmap ( virtio->common );
	return rc;
}

/******************************************************************************
 *
 * Transport-independent operations
 *
 ******************************************************************************
 */

/**
 * Reset device
 *
 * @v virtio		Virtio device
 * @ret rc		Return status code
 */
int virtio_reset ( struct virtio_device *virtio ) {
	int rc;

	/* Clear driver status */
	virtio->stat = 0;

	/* Reset device */
	if ( ( rc = virtio->op->reset ( virtio ) ) != 0 ) {
		DBGC ( virtio, "VIRTIO %s could not reset: %s\n",
		       virtio->name, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Report driver status
 *
 * @v virtio		Virtio device
 * @v stat		Additional driver status bits
 */
static void virtio_status ( struct virtio_device *virtio, unsigned int stat ) {

	/* Set new driver status bits */
	virtio->stat |= stat;

	/* Report driver status */
	virtio->op->status ( virtio );
}

/**
 * Negotiate features
 *
 * @v virtio		Virtio device
 * @v driver		Driver supported features
 */
static void virtio_negotiate ( struct virtio_device *virtio,
			       const struct virtio_features *driver ) {
	struct virtio_features *supported = &virtio->supported;
	struct virtio_features *negotiated = &virtio->negotiated;
	int words;
	int i;

	/* Get device supported features */
	virtio->op->supported ( virtio );

	/* Negotiate mutually supported features */
	words = ( sizeof ( supported->feat ) / sizeof ( supported->feat[0] ) );
	for ( i = 0 ; i < words ; i++ )
		negotiated->feat[i] = ( supported->feat[i] & driver->feat[i] );
	virtio->op->negotiate ( virtio );

	DBGC ( virtio, "VIRTIO %s features ", virtio->name );
	for ( i = ( words - 1) ; i >= 0 ; i-- ) {
		DBGC ( virtio, "%08x%s", supported->feat[i],
		       ( i ? ":" : "" ) );
	}
	DBGC ( virtio, " -> " );
	for ( i = ( words - 1) ; i >= 0 ; i-- ) {
		DBGC ( virtio, "%08x%s", negotiated->feat[i],
		       ( i ? ":" : "" ) );
	}
	DBGC ( virtio, "\n" );
}

/**
 * Initialise device
 *
 * @v virtio		Virtio device
 * @v driver		Driver supported features
 * @ret rc		Return status code
 */
int virtio_init ( struct virtio_device *virtio,
		  const struct virtio_features *driver ) {
	int rc;

	/* Reset device */
	if ( ( rc = virtio_reset ( virtio ) ) != 0 )
		goto err_reset;

	/* Acknowledge device existence */
	virtio_status ( virtio, VIRTIO_STAT_FOUND );

	/* Report driver existence */
	virtio_status ( virtio, VIRTIO_STAT_DRIVER );

	/* Negotiate features */
	virtio_negotiate ( virtio, driver );

	/* Report driver readiness */
	virtio_status ( virtio, VIRTIO_STAT_READY );

	return 0;

	virtio_reset ( virtio );
 err_reset:
	virtio_status ( virtio, VIRTIO_STAT_FAIL );
	return rc;
}

/**
 * Enable queue
 *
 * @v virtio		Virtio device
 * @v queue		Virtio queue
 * @v count		Requested queue size
 * @ret rc		Return status code
 */
int virtio_enable ( struct virtio_device *virtio, struct virtio_queue *queue,
		    unsigned int count ) {
	int rc;

	/* Determine queue size */
	virtio->op->size ( virtio, queue, count );
	if ( ! queue->count ) {
		DBGC ( virtio, "VIRTIO %s Q%d does not exist\n",
		       virtio->name, queue->index );
		rc = -ENODEV;
		goto err_count;
	}

	/* Allocate and initialise queue */
	queue->desc = dma_alloc ( virtio->dma, &queue->map, queue->len,
				  VIRTIO_PAGE );
	if ( ! queue->desc ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	memset ( queue->desc, 0, queue->len );

	/* Enable queue */
	virtio->op->enable ( virtio, queue );
	DBGC ( virtio, "VIRTIO %s Q%d %dx descriptors at [%#08lx,%#08lx)\n",
	       virtio->name, queue->index, queue->count,
	       virt_to_phys ( queue->desc ),
	       ( virt_to_phys ( queue->desc ) +
		 virtio_desc_size ( queue->count ) ) );
	DBGC ( virtio, "VIRTIO %s Q%d %dx submissions at [%#08lx,%#08lx)\n",
	       virtio->name, queue->index, queue->count,
	       virt_to_phys ( queue->sq ),
	       ( virt_to_phys ( queue->sq ) +
		 virtio_sq_size ( queue->count ) ) );
	DBGC ( virtio, "VIRTIO %s Q%d %dx completions at [%#08lx,%#08lx)\n",
	       virtio->name, queue->index, queue->count,
	       virt_to_phys ( queue->cq ),
	       ( virt_to_phys ( queue->cq ) +
		 virtio_cq_size ( queue->count ) ) );

	return 0;

	dma_free ( &queue->map, queue->desc, queue->len );
 err_alloc:
 err_count:
	return rc;
}
