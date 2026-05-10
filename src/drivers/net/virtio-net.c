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
	.word = { 0, VIRTIO_FEAT1_VERSION },
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
	unsigned int fill;
	unsigned int i;
	int rc;

	/* Enable queue */
	if ( ( rc = virtio_enable ( virtio, &queue->queue,
				    queue->count ) ) != 0 ) {
		DBGC ( vnet, "VNET %s Q%d could not initialise: %s\n",
		       virtio->name, queue->queue.index, strerror ( rc ) );
		return rc;
	}

	/* Calculate mask */
	fill = queue->queue.count;
	if ( fill > queue->max )
		fill = queue->max;
	queue->fill = fill;
	queue->mask = ( fill - 1 );

	/* Initialise buffer ID ring */
	for ( i = 0 ; i < fill ; i++ )
		queue->ids[i] = ( i * 2 );

	DBGC ( vnet, "VNET %s Q%d using %d/%d descriptors\n", virtio->name,
	       queue->queue.index, queue->fill, queue->queue.count );
	return 0;
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

 err_tx:
 err_rx:
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

	DBGC ( vnet, "VNET %s does not yet support transmit\n", virtio->name );
	( void ) iobuf;
	return -ENOTSUP;
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void virtio_net_poll ( struct net_device *netdev ) {
	struct virtio_net *vnet = netdev->priv;

	/* Not yet implemented */
	( void ) vnet;
}

/**
 * Enable or disable interrupts
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void virtio_net_irq ( struct net_device *netdev, int enable ) {
	struct virtio_net *vnet = netdev->priv;
	struct virtio_device *virtio = &vnet->virtio;

	DBGC ( vnet, "VNET %s does not yet support interrupts\n",
	       virtio->name );
	( void ) enable;
}

/** Virtio network device operations */
static struct net_device_operations virtio_net_operations = {
	.open		= virtio_net_open,
	.close		= virtio_net_close,
	.transmit	= virtio_net_transmit,
	.poll		= virtio_net_poll,
	.irq		= virtio_net_irq,
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
	virtio_net_queue_init ( &vnet->rx, VIRTIO_NET_RX_INDEX,
				VIRTIO_NET_RX_COUNT, VIRTIO_NET_RX_MAX,
				vnet->rx_ids );
	virtio_net_queue_init ( &vnet->tx, VIRTIO_NET_TX_INDEX,
				VIRTIO_NET_TX_COUNT, VIRTIO_NET_TX_MAX,
				vnet->tx_ids );

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
