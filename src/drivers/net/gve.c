/*
 * Copyright (C) 2024 Michael Brown <mbrown@fensystems.co.uk>.
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

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <byteswap.h>
#include <ipxe/netdevice.h>
#include <ipxe/ethernet.h>
#include <ipxe/if_ether.h>
#include <ipxe/iobuf.h>
#include <ipxe/pci.h>
#include "gve.h"

/** @file
 *
 * Google Virtual Ethernet network driver
 *
 */

/******************************************************************************
 *
 * Device reset
 *
 ******************************************************************************
 */

/**
 * Wait for reset operation to be acknowledged
 *
 * @v gve		GVE device
 * @v expected		Expected reset state
 * @ret rc		Return status code
 */
static int gve_reset_wait ( struct gve_nic *gve, uint32_t expected ) {
	uint32_t stat;
	unsigned int i;

	/* Wait for reset to complete */
	for ( i = 0 ; i < GVE_RESET_MAX_WAIT_MS ; i++ ) {

		/* Check if device is ready */
		stat = readl ( gve->regs + GVE_STAT );
		if ( ( stat & GVE_STAT_RESET ) == expected )
			return 0;

		/* Delay */
		mdelay ( 1 );
	}

	DBGC ( gve, "GVE %p timed out waiting for reset status %#08x "
	       "(got %#08x)\n", gve, expected, stat );
	return -ETIMEDOUT;
}

/**
 * Reset hardware
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_reset ( struct gve_nic *gve ) {
	int rc;

	/* Clear admin queue page frame number (for older hardware) */
	writel ( 0, ( gve->cfg + GVE_CFG_AQ_PFN ) );

	/* Trigger reset */
	writel ( GVE_CTRL_RESET, ( gve->regs + GVE_CTRL ) );

	/* Wait for reset to take effect */
	if ( ( rc = gve_reset_wait ( gve, GVE_STAT_RESET ) ) != 0 )
		return rc;

	/* Clear reset */
	writel ( 0, ( gve->regs + GVE_CTRL ) );

	/* Wait for reset to clear */
	if ( ( rc = gve_reset_wait ( gve, 0 ) ) != 0 )
		return rc;

	return 0;
}

/******************************************************************************
 *
 * Admin queue
 *
 ******************************************************************************
 */

/**
 * Set queue base address
 *
 * @v gve		GVE device
 * @v offset		Register offset
 * @v address		Base address
 */
static inline void gve_set_base ( struct gve_nic *gve, unsigned int offset,
				  void *base ) {
	physaddr_t phys = virt_to_bus ( base );

	/* Program base address registers */
	writel ( ( phys & 0xffffffffUL ),
		 ( gve->regs + offset + GVE_BASE_LO ) );
	if ( sizeof ( phys ) > sizeof ( uint32_t ) ) {
		writel ( ( ( ( uint64_t ) phys ) >> 32 ),
			 ( gve->regs + offset + GVE_BASE_HI ) );
	} else {
		writel ( 0, ( gve->regs + offset + GVE_BASE_HI ) );
	}
}

/**
 * Create admin queues
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_create_admin ( struct gve_nic *gve ) {
	size_t aq_len = ( GVE_AQ_COUNT * sizeof ( gve->aq.req[0] ) );
	size_t acq_len = ( GVE_ACQ_COUNT * sizeof ( gve->acq.rsp[0] ) );
	int rc;

	/* Allocate admin completion queue */
	gve->acq.rsp = malloc_phys ( acq_len, acq_len );
	if ( ! gve->acq.rsp ) {
		rc = -ENOMEM;
		goto err_alloc_acq;
	}
	memset ( gve->acq.rsp, 0, acq_len );

	/* Allocate admin queue */
	gve->aq.req = malloc_phys ( aq_len, aq_len );
	if ( ! gve->aq.req ) {
		rc = -ENOMEM;
		goto err_alloc_aq;
	}
	memset ( gve->aq.req, 0, aq_len );

	/* Program queue addresses and capabilities */
	gve_set_base ( gve, GVE_ACQ_BASE, gve->acq.rsp );
	gve_set_caps ( gve, GVE_ACQ_CAPS, GVE_ACQ_COUNT,
		       sizeof ( gve->acq.rsp[0] ) );
	gve_set_base ( gve, GVE_AQ_BASE, gve->aq.req );
	gve_set_caps ( gve, GVE_AQ_CAPS, GVE_AQ_COUNT,
		       sizeof ( gve->aq.req[0] ) );

	DBGC ( gve, "GVE %p AQ [%08lx,%08lx) ACQ [%08lx,%08lx)\n",
	       gve, virt_to_phys ( gve->aq.req ),
	       ( virt_to_phys ( gve->aq.req ) + aq_len ),
	       virt_to_phys ( gve->acq.rsp ),
	       ( virt_to_phys ( gve->acq.rsp ) + acq_len ) );
	return 0;

	gve_clear_caps ( gve, GVE_AQ_CAPS );
	gve_clear_caps ( gve, GVE_ACQ_CAPS );
	free_phys ( gve->aq.req, aq_len );
 err_alloc_aq:
	free_phys ( gve->acq.rsp, acq_len );
 err_alloc_acq:
	return rc;
}

/**
 * Destroy admin queues
 *
 * @v gve		GVE device
 */
static void gve_destroy_admin ( struct gve_nic *gve ) {
	size_t aq_len = ( GVE_AQ_COUNT * sizeof ( gve->aq.req[0] ) );
	size_t acq_len = ( GVE_ACQ_COUNT * sizeof ( gve->acq.rsp[0] ) );

	/* Clear queue capabilities */
	gve_clear_caps ( gve, GVE_AQ_CAPS );
	gve_clear_caps ( gve, GVE_ACQ_CAPS );
	wmb();

	/* Free queues */
	free_phys ( gve->aq.req, aq_len );
	free_phys ( gve->acq.rsp, acq_len );
	DBGC ( gve, "GVE %p AQ and ACQ destroyed\n", gve );
}

/**
 * Get next available admin queue request
 *
 * @v gve		GVE device
 * @ret req		Admin queue request
 */
static union gve_aq_req * gve_admin_req ( struct gve_nic *gve ) {
	union gve_aq_req *req;
	unsigned int index;

	/* Get next request */
	index = ( gve->aq.prod % GVE_AQ_COUNT );
	req = &gve->aq.req[index];

	/* Initialise request */
	memset ( ( ( ( void * ) req ) + sizeof ( req->header ) ), 0,
		 ( sizeof ( *req ) - sizeof ( req->header ) ) );
	req->header.id = gve->aq.prod;

	/* Increment producer counter */
	gve->aq.prod++;

	return req;
}

/**
 * Issue admin queue request
 *
 * @v gve		GVE device
 * @v req		Admin queue request
 * @v rsp		Admin queue response to fill in
 * @ret rc		Return status code
 */
static int gve_admin ( struct gve_nic *gve, union gve_aq_req *req,
		       union gve_acq_rsp **rsp ) {
	unsigned int index;
	unsigned int i;
	int rc;

	/* Locate response */
	index = ( gve->acq.cons % GVE_ACQ_COUNT );
	*rsp = &gve->acq.rsp[index];

	/* Mark request as ready */
	req->header.flags ^= GVE_AQ_PHASE;
	wmb();
	DBGC2 ( gve, "GVE %p admin request %#x:\n",
		gve, le16_to_cpu ( req->header.id ) );
	DBGC2_HDA ( gve, virt_to_phys ( req ), req, sizeof ( *req ) );

	/* Ring doorbell */
	writel ( gve->aq.prod, ( gve->regs + GVE_AQ_DB ) );

	/* Wait for response */
	for ( i = 0 ; i < GVE_ADMIN_MAX_WAIT_MS ; i++ ) {

		/* Check for response */
		if ( ( (*rsp)->header.flags ^ gve->acq.phase ) & GVE_ACQ_PHASE){
			mdelay ( 1 );
			continue;
		}
		DBGC2 ( gve, "GVE %p admin response %#x:\n",
			gve, le16_to_cpu ( (*rsp)->header.id ) );
		DBGC2_HDA ( gve, virt_to_phys ( *rsp ), *rsp, sizeof ( **rsp ));

		/* Increment consumer counter */
		gve->acq.cons++;
		if ( ( gve->acq.cons % GVE_ACQ_COUNT ) == 0 )
			gve->acq.phase ^= GVE_ACQ_PHASE;

		/* Check command identifier */
		if ( (*rsp)->header.id != req->header.id ) {
			DBGC ( gve, "GVE %p admin response %#x mismatch:\n",
			       gve, le16_to_cpu ( (*rsp)->header.id ) );
			rc = -EILSEQ;
			goto err;
		}

		/* Check status */
		if ( (*rsp)->header.status != 0 ) {
			DBGC ( gve, "GVE %p admin response %#x status %d:\n",
			       gve, le16_to_cpu ( (*rsp)->header.id ),
			       (*rsp)->header.status );
			rc = -EIO;
			goto err;
		}

		/* Success */
		return 0;
	}

	rc = -ETIMEDOUT;
	DBGC ( gve, "GVE %p timed out waiting for admin request %#x:\n",
	       gve, le16_to_cpu ( req->header.id ) );
 err:
	DBGC_HDA ( gve, virt_to_phys ( req ), req, sizeof ( *req ) );
	DBGC_HDA ( gve, virt_to_phys ( *rsp ), *rsp, sizeof ( **rsp ) );
	return rc;
}

/**
 * Get device descriptor
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int gve_describe ( struct net_device *netdev ) {

	//
	( void ) netdev;
	return -ENOTSUP;
}

/******************************************************************************
 *
 * Network device interface
 *
 ******************************************************************************
 */

/**
 * Refill receive queue
 *
 * @v netdev		Network device
 */
static void gve_refill_rx ( struct net_device *netdev ) {

	//
	( void ) netdev;
}

/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int gve_open ( struct net_device *netdev ) {

	//
	( void ) netdev;
	return -ENOTSUP;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void gve_close ( struct net_device *netdev ) {

	//
	( void ) netdev;
}

/**
 * Transmit packet
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int gve_transmit ( struct net_device *netdev, struct io_buffer *iobuf ) {

	//
	( void ) netdev;
	( void ) iobuf;
	return -ENOTSUP;
}

/**
 * Poll for completed transmissions
 *
 * @v netdev		Network device
 */
static void gve_poll_tx ( struct net_device *netdev ) {

	//
	( void ) netdev;
}

/**
 * Poll for received packets
 *
 * @v netdev		Network device
 */
static void gve_poll_rx ( struct net_device *netdev ) {

	//
	( void ) netdev;
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void gve_poll ( struct net_device *netdev ) {

	/* Poll for transmit completions */
	gve_poll_tx ( netdev );

	/* Poll for receive completions */
	gve_poll_rx ( netdev );

	/* Refill receive ring */
	gve_refill_rx ( netdev );
}

/** GVE network device operations */
static struct net_device_operations gve_operations = {
	.open		= gve_open,
	.close		= gve_close,
	.transmit	= gve_transmit,
	.poll		= gve_poll,
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
static int gve_probe ( struct pci_device *pci ) {
	struct net_device *netdev;
	struct gve_nic *gve;
	unsigned long cfg_start;
	int rc;

	/* Allocate and initialise net device */
	netdev = alloc_etherdev ( sizeof ( *gve ) );
	if ( ! netdev ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	netdev_init ( netdev, &gve_operations );
	gve = netdev->priv;
	pci_set_drvdata ( pci, netdev );
	netdev->dev = &pci->dev;
	memset ( gve, 0, sizeof ( *gve ) );

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Map configuration registers */
	cfg_start = pci_bar_start ( pci, GVE_CFG_BAR );
	gve->cfg = pci_ioremap ( pci, cfG_start, GVE_CFG_SIZE );
	if ( ! gve->cfg ) {
		rc = -ENODEV;
		goto err_cfg;
	}

	/* Reset the NIC */
	if ( ( rc = gve_reset ( gve ) ) != 0 )
		goto err_reset;

	/* Create admin queues */
	if ( ( rc = gve_create_admin ( gve ) ) != 0 )
		goto err_create_admin;

	/* Fetch MAC address */
	if ( ( rc = gve_describe ( netdev ) ) != 0 )
		goto err_describe;

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Mark as link up, since we have no way to test link state on
	 * this hardware.
	 */
	netdev_link_up ( netdev );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
 err_describe:
 err_set_host_attributes:
	gve_destroy_admin ( gve );
 err_create_admin:
	gve_reset ( gve );
 err_reset:
	iounmap ( gve->cfg );
 err_cfg:
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
static void gve_remove ( struct pci_device *pci ) {
	struct net_device *netdev = pci_get_drvdata ( pci );
	struct gve_nic *gve = netdev->priv;

	/* Unregister network device */
	unregister_netdev ( netdev );

	/* Destroy admin queues */
	gve_destroy_admin ( gve );

	/* Reset card */
	gve_reset ( gve );

	/* Free network device */
	iounmap ( gve->cfg );
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** GVE PCI device IDs */
static struct pci_device_id gve_nics[] = {
	PCI_ROM ( 0x1ae0, 0x0042, "gve", "gVNIC", 0 ),
};

/** GVE PCI driver */
struct pci_driver gve_driver __pci_driver = {
	.ids = gve_nics,
	.id_count = ( sizeof ( gve_nics ) / sizeof ( gve_nics[0] ) ),
	.probe = gve_probe,
	.remove = gve_remove,
};
