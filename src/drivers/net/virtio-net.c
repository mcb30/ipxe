/*
 * Copyright (C) 2026 Michael Brown <mbrown@fensystems.co.uk>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
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

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <byteswap.h>
#include <ipxe/netdevice.h>
#include <ipxe/ethernet.h>
#include <ipxe/if_ether.h>
#include <ipxe/iobuf.h>
#include <ipxe/malloc.h>
#include <ipxe/pci.h>
#include "virtio-net.h"

/** @file
 *
 * Virtual I/O network device
 *
 */

/** Supported features */
const struct virtio_features virtio_net_features = {
	.word = { 0, VIRTIO_FEAT1_MODERN },
};

/******************************************************************************
 *
 * Network device interface
 *
 ******************************************************************************
 */

/**
 * Enable queue
 *
 * @v vnet		Virtio network device
 * @v queue		Virtio network queue
 * @ret rc		Return status code
 */
static int virtio_net_enable ( struct virtio_net *vnet,
			       struct virtio_net_queue *queue ) {
	struct virtio_device *virtio = &vnet->virtio;
	struct virtio_desc *desc;
	unsigned int fill;
	unsigned int slot;
	unsigned int index;
	unsigned int write;
	size_t hlen;
	int rc;

	/* Calculate packet header length */
	hlen = ( virtio_is_legacy ( virtio ) ?
		 sizeof ( queue->hdr.legacy ) : sizeof ( queue->hdr.modern ) );

	/* Map packet header */
	if ( ( dma_map ( virtio->dma, &queue->map, &queue->hdr,
			 sizeof ( queue->hdr ), queue->dma ) ) != 0 ) {
		DBGC ( vnet, "VNET %s Q%d could not map header: %s\n",
		       virtio->name, queue->queue.index, strerror ( rc ) );
		goto err_map;
	}

	/* Enable queue */
	if ( ( rc = virtio_enable ( virtio, &queue->queue,
				    queue->count ) ) != 0 ) {
		DBGC ( vnet, "VNET %s Q%d could not initialise: %s\n",
		       virtio->name, queue->queue.index, strerror ( rc ) );
		goto err_enable;
	}

	/* Calculate mask */
	fill = queue->queue.count;
	if ( fill > queue->max )
		fill = queue->max;
	queue->fill = fill;
	queue->mask = ( fill - 1 );

	/* Initialise descriptors and slot ring */
	write = queue->write;
	for ( slot = 0 ; slot < fill ; slot++ ) {
		queue->slot[slot] = slot;
		index = ( slot * VIRTIO_NET_DESCS );
		desc = &queue->queue.desc[index];
		desc[0].addr = cpu_to_le64 ( dma ( &queue->map, &queue->hdr ));
		desc[0].len = cpu_to_le32 ( hlen );
		desc[0].flags = cpu_to_le16 ( VIRTIO_DESC_FL_NEXT | write );
		desc[0].next = cpu_to_le16 ( index + 1 );
		desc[1].flags = cpu_to_le16 ( write );
	}

	DBGC ( vnet, "VNET %s Q%d using %d/%d descriptors\n", virtio->name,
	       queue->queue.index, queue->fill, queue->queue.count );
	return 0;

	/* There may be no way to disable individual queues: the
	 * caller must reset the whole device to recover from a
	 * failure.
	 */
 err_enable:
	dma_unmap ( &queue->map, sizeof ( queue->hdr ) );
 err_map:
	return rc;
}

/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int virtio_net_open ( struct net_device *netdev ) {
	struct virtio_net *vnet = netdev->priv;
	struct virtio_device *virtio = &vnet->virtio;
	int rc;

	/* (Re)initialise device */
	if ( ( rc = virtio_init ( virtio, &virtio_net_features ) ) != 0 ) {
		DBGC ( vnet, "VNET %s could not initialise: %s\n",
		       virtio->name, strerror ( rc ) );
		goto err_init;
	}

	/* Enable receive queue */
	if ( ( rc = virtio_net_enable ( vnet, &vnet->rx ) ) != 0 ) {
		DBGC ( vnet, "VNET %s could not enable RX: %s\n",
		       virtio->name, strerror ( rc ) );
		goto err_rx;
	}

	/* Enable transmit queue */
	if ( ( rc = virtio_net_enable ( vnet, &vnet->tx ) ) != 0 ) {
		DBGC ( vnet, "VNET %s could not enable TX: %s\n",
		       virtio->name, strerror ( rc ) );
		goto err_tx;
	}

	return 0;

	dma_unmap ( &vnet->tx.map, sizeof ( vnet->tx.hdr ) );
 err_tx:
	dma_unmap ( &vnet->rx.map, sizeof ( vnet->rx.hdr ) );
 err_rx:
	/* There may be no way to disable individual queues: we must
	 * reset the whole device instead and then free the queues.
	 */
	virtio_reset ( virtio );
	virtio_free ( virtio, &vnet->rx.queue );
	virtio_free ( virtio, &vnet->tx.queue );
 err_init:
	return rc;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void virtio_net_close ( struct net_device *netdev ) {
	struct virtio_net *vnet = netdev->priv;
	struct virtio_device *virtio = &vnet->virtio;

	/* Reset device */
	virtio_reset ( virtio );

	/* Unmap headers (now that device is guaranteed idle) */
	dma_unmap ( &vnet->rx.map, sizeof ( vnet->rx.hdr ) );
	dma_unmap ( &vnet->tx.map, sizeof ( vnet->tx.hdr ) );

	/* Free queues */
	virtio_free ( virtio, &vnet->rx.queue );
	virtio_free ( virtio, &vnet->tx.queue );
}

/**
 * Transmit packet
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int virtio_net_transmit ( struct net_device *netdev,
				 struct io_buffer *iobuf ) {
	struct virtio_net *vnet = netdev->priv;
	struct virtio_device *virtio = &vnet->virtio;
	struct virtio_net_queue *queue = &vnet->tx;
	struct virtio_desc *desc;
	unsigned int index;
	unsigned int prod;
	unsigned int slot;
	size_t len;

	/* Get next transmit descriptor */
	prod = queue->queue.prod;
	if ( ( prod - queue->queue.cons ) >= queue->fill ) {
		DBGC ( vnet, "VNET %s out of transmit descriptors\n",
		       virtio->name );
		return -ENOBUFS;
	}
	slot = queue->slot[ prod & queue->mask ];
	index = ( slot * VIRTIO_NET_DESCS );
	desc = &queue->queue.desc[index];

	/* Populate transmit descriptor */
	len = iob_len ( iobuf );
	desc[1].addr = cpu_to_le64 ( iob_dma ( iobuf ) );
	desc[1].len = cpu_to_le32 ( len );
	DBGC2 ( vnet, "VNET %s TX [%02x-%02x] is [%lx,%lx)\n", virtio->name,
		index, ( index + 1 ), virt_to_phys ( iobuf->data ),
		( virt_to_phys ( iobuf->data ) + len ) );

	/* Push transmit descriptor */
	virtio_submit ( &queue->queue, index );
	virtio_notify ( &queue->queue );

	/* Record I/O buffer */
	assert ( vnet->tx_iobuf[slot] == NULL );
	vnet->tx_iobuf[slot] = iobuf;

	return 0;
}

/**
 * Poll for completed packets
 *
 * @v netdev		Network device
 */
static void virtio_net_poll_tx ( struct net_device *netdev ) {
	struct virtio_net *vnet = netdev->priv;
	struct virtio_device *virtio = &vnet->virtio;
	struct virtio_net_queue *queue = &vnet->tx;
	struct io_buffer *iobuf;
	unsigned int index;
	unsigned int cons;
	unsigned int slot;

	/* Poll for completed descriptors */
	while ( virtio_completed ( &queue->queue ) ) {

		/* Complete descriptor and recycle slot */
		cons = queue->queue.cons;
		index = virtio_complete ( &queue->queue, NULL );
		slot = ( index / VIRTIO_NET_DESCS );
		queue->slot[ cons & queue->mask ] = slot;

		/* Complete I/O buffer */
		iobuf = vnet->tx_iobuf[slot];
		assert ( iobuf != NULL );
		vnet->tx_iobuf[slot] = NULL;
		DBGC2 ( vnet, "VNET %s TX [%02x-%02x] complete\n",
			virtio->name, index, ( index + 1 ) );
		netdev_tx_complete ( netdev, iobuf );
	}
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void virtio_net_poll ( struct net_device *netdev ) {

	/* Poll for completed packets */
	virtio_net_poll_tx ( netdev );
}

/** Virtio network device operations */
static struct net_device_operations virtio_net_operations = {
	.open		= virtio_net_open,
	.close		= virtio_net_close,
	.transmit	= virtio_net_transmit,
	.poll		= virtio_net_poll,
};

/******************************************************************************
 *
 * PCI interface
 *
 ******************************************************************************
 */

/**
 * Probe PCI device
 *
 * @v pci		PCI device
 * @ret rc		Return status code
 */
static int virtio_net_probe ( struct pci_device *pci ) {
	struct net_device *netdev;
	struct virtio_net *vnet;
	struct virtio_device *virtio;
	int rc;

	/* Allocate and initialise net device */
	netdev = alloc_etherdev ( sizeof ( *vnet ) );
	if ( ! netdev ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	netdev_init ( netdev, &virtio_net_operations );
	vnet = netdev->priv;
	pci_set_drvdata ( pci, netdev );
	netdev->dev = &pci->dev;
	netdev->dma = &pci->dma;
	memset ( vnet, 0, sizeof ( *vnet ) );
	virtio = &vnet->virtio;
	virtio_net_queue_init ( &vnet->rx, vnet->rx_slot, VIRTIO_NET_RX_INDEX,
				VIRTIO_NET_RX_COUNT, VIRTIO_NET_RX_MAX,
				DMA_RX, VIRTIO_DESC_FL_WRITE );
	virtio_net_queue_init ( &vnet->tx, vnet->tx_slot, VIRTIO_NET_TX_INDEX,
				VIRTIO_NET_TX_COUNT, VIRTIO_NET_TX_MAX,
				DMA_TX, 0 );

	/* Map PCI device */
	if ( ( rc = virtio_pci_map ( virtio, pci ) ) != 0 ) {
		DBGC ( vnet, "VNET %s could not map: %s\n",
		       virtio->name, strerror ( rc ) );
		goto err_pci_map;
	}

	/* Reset the NIC */
	if ( ( rc = virtio_reset ( virtio ) ) != 0 )
		goto err_reset;

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	//
	netdev_link_up ( netdev );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
	virtio_reset ( virtio );
 err_reset:
	virtio_unmap ( virtio );
 err_pci_map:
	netdev_nullify ( netdev );
	netdev_put ( netdev );
 err_alloc:
	return rc;
}

/**
 * Remove PCI device
 *
 * @v pci		PCI device
 */
static void virtio_net_remove ( struct pci_device *pci ) {
	struct net_device *netdev = pci_get_drvdata ( pci );
	struct virtio_net *vnet = netdev->priv;
	struct virtio_device *virtio = &vnet->virtio;

	/* Unregister network device */
	unregister_netdev ( netdev );

	/* Reset device */
	virtio_reset ( virtio );

	/* Free network device */
	virtio_unmap ( virtio );
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** Virtio network PCI device IDs */
static struct pci_device_id virtio_net_ids[] = {
	PCI_ROM ( 0x1af4, 0x1000, "virtio-net", "Virtio (legacy)", 0 ),
	PCI_ROM ( 0x1af4, 0x1041, "virtio-net", "Virtio (modern)", 0 ),
};

/** Virtio network PCI driver */
struct pci_driver virtio_net_driver __pci_driver = {
	.ids = virtio_net_ids,
	.id_count = ( sizeof ( virtio_net_ids ) /
		      sizeof ( virtio_net_ids[0] ) ),
	.probe = virtio_net_probe,
	.remove = virtio_net_remove,
};
