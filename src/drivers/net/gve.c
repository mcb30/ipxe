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
#include <unistd.h>
#include <errno.h>
#include <byteswap.h>
#include <ipxe/netdevice.h>
#include <ipxe/ethernet.h>
#include <ipxe/if_ether.h>
#include <ipxe/iobuf.h>
#include <ipxe/malloc.h>
#include <ipxe/pci.h>
#include "gve.h"

/** @file
 *
 * Google Virtual Ethernet network driver
 *
 */

/* Disambiguate the various error causes */
#define EINFO_EIO_AQ_UNSET						\
	__einfo_uniqify ( EINFO_EIO, 0x00, "Uncompleted" )
#define EIO_AQ_UNSET							\
	__einfo_error ( EINFO_EIO_AQ_UNSET )
#define EINFO_EIO_AQ_ABORTED						\
	__einfo_uniqify ( EINFO_EIO, 0x10, "Aborted" )
#define EIO_AQ_ABORTED							\
	__einfo_error ( EINFO_EIO_AQ_ABORTED )
#define EINFO_EIO_AQ_EXISTS						\
	__einfo_uniqify ( EINFO_EIO, 0x11, "Already exists" )
#define EIO_AQ_EXISTS							\
	__einfo_error ( EINFO_EIO_AQ_EXISTS )
#define EINFO_EIO_AQ_CANCELLED						\
	__einfo_uniqify ( EINFO_EIO, 0x12, "Cancelled" )
#define EIO_AQ_CANCELLED						\
	__einfo_error ( EINFO_EIO_AQ_CANCELLED )
#define EINFO_EIO_AQ_DATALOSS						\
	__einfo_uniqify ( EINFO_EIO, 0x13, "Data loss" )
#define EIO_AQ_DATALOSS							\
	__einfo_error ( EINFO_EIO_AQ_DATALOSS )
#define EINFO_EIO_AQ_DEADLINE						\
	__einfo_uniqify ( EINFO_EIO, 0x14, "Deadline exceeded" )
#define EIO_AQ_DEADLINE							\
	__einfo_error ( EINFO_EIO_AQ_DEADLINE )
#define EINFO_EIO_AQ_PRECONDITION					\
	__einfo_uniqify ( EINFO_EIO, 0x15, "Failed precondition" )
#define EIO_AQ_PRECONDITION						\
	__einfo_error ( EINFO_EIO_AQ_PRECONDITION )
#define EINFO_EIO_AQ_INTERNAL						\
	__einfo_uniqify ( EINFO_EIO, 0x16, "Internal error" )
#define EIO_AQ_INTERNAL							\
	__einfo_error ( EINFO_EIO_AQ_INTERNAL )
#define EINFO_EIO_AQ_INVAL						\
	__einfo_uniqify ( EINFO_EIO, 0x17, "Invalid argument" )
#define EIO_AQ_INVAL							\
	__einfo_error ( EINFO_EIO_AQ_INVAL )
#define EINFO_EIO_AQ_NOT_FOUND						\
	__einfo_uniqify ( EINFO_EIO, 0x18, "Not found" )
#define EIO_AQ_NOT_FOUND						\
	__einfo_error ( EINFO_EIO_AQ_NOT_FOUND )
#define EINFO_EIO_AQ_RANGE						\
	__einfo_uniqify ( EINFO_EIO, 0x19, "Out of range" )
#define EIO_AQ_RANGE							\
	__einfo_error ( EINFO_EIO_AQ_RANGE )
#define EINFO_EIO_AQ_PERM						\
	__einfo_uniqify ( EINFO_EIO, 0x1a, "Permission denied" )
#define EIO_AQ_PERM							\
	__einfo_error ( EINFO_EIO_AQ_PERM )
#define EINFO_EIO_AQ_UNAUTH						\
	__einfo_uniqify ( EINFO_EIO, 0x1b, "Unauthenticated" )
#define EIO_AQ_UNAUTH							\
	__einfo_error ( EINFO_EIO_AQ_UNAUTH )
#define EINFO_EIO_AQ_RESOURCE						\
	__einfo_uniqify ( EINFO_EIO, 0x1c, "Resource exhausted" )
#define EIO_AQ_RESOURCE							\
	__einfo_error ( EINFO_EIO_AQ_RESOURCE )
#define EINFO_EIO_AQ_UNAVAIL						\
	__einfo_uniqify ( EINFO_EIO, 0x1d, "Unavailable" )
#define EIO_AQ_UNAVAIL							\
	__einfo_error ( EINFO_EIO_AQ_UNAVAIL )
#define EINFO_EIO_AQ_NOTSUP						\
	__einfo_uniqify ( EINFO_EIO, 0x1e, "Unimplemented" )
#define EIO_AQ_NOTSUP	       						\
	__einfo_error ( EINFO_EIO_AQ_NOTSUP )
#define EINFO_EIO_AQ_UNKNOWN						\
	__einfo_uniqify ( EINFO_EIO, 0x1f, "Unknown error" )
#define EIO_AQ_UNKNOWN							\
	__einfo_error ( EINFO_EIO_AQ_UNKNOWN )
#define EIO_AQ( status )						\
	EUNIQ ( EINFO_EIO, ( (status) & 0x1f ),				\
		EIO_AQ_UNSET, EIO_AQ_ABORTED, EIO_AQ_EXISTS,		\
		EIO_AQ_CANCELLED, EIO_AQ_DATALOSS, EIO_AQ_DEADLINE,	\
		EIO_AQ_PRECONDITION, EIO_AQ_INTERNAL, EIO_AQ_INVAL,	\
		EIO_AQ_NOT_FOUND, EIO_AQ_RANGE, EIO_AQ_PERM,		\
		EIO_AQ_UNAUTH, EIO_AQ_RESOURCE, EIO_AQ_UNAVAIL,		\
		EIO_AQ_NOTSUP, EIO_AQ_UNKNOWN )

/******************************************************************************
 *
 * Device reset
 *
 ******************************************************************************
 */

/**
 * Reset hardware
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_reset ( struct gve_nic *gve ) {
	uint32_t devstat;
	uint32_t pfn;
	unsigned int i;

	/* Clear admin queue page frame number (for older hardware) */
	writel ( 0, ( gve->cfg + GVE_CFG_AQ_PFN ) );

	/* Trigger reset (for newer hardware) */
	writel ( bswap_32 ( GVE_CFG_DRVSTAT_RESET ),
		 ( gve->cfg + GVE_CFG_DRVSTAT ) );

	/* Wait for device to reset */
	for ( i = 0 ; i < GVE_RESET_MAX_WAIT_MS ; i++ ) {

		/* Check for reset completion */
		devstat = readl ( gve->cfg + GVE_CFG_DEVSTAT );
		pfn = readl ( gve->cfg + GVE_CFG_AQ_PFN );
		if ( ( ( devstat & bswap_32 ( GVE_CFG_DEVSTAT_RESET ) ) ||
		       ( gve->revision == 0 ) ) &&
		     ( pfn == 0 ) ) {
			return 0;
		}

		/* Delay */
		mdelay ( 1 );
	}

	DBGC ( gve, "GVE %p reset timed out (PFN %#08x devstat %#08x)\n",
	       gve, bswap_32 ( pfn ), bswap_32 ( devstat ) );
	return -ETIMEDOUT;
}

/******************************************************************************
 *
 * Admin queue
 *
 ******************************************************************************
 */

/**
 * Create admin queues
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_create_admin ( struct gve_nic *gve ) {
	physaddr_t base;
	int rc;

	/* Allocate admin queue */
	gve->aq.cmd = malloc_phys ( GVE_AQ_LEN, GVE_AQ_ALIGN );
	if ( ! gve->aq.cmd ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	memset ( gve->aq.cmd, 0, GVE_AQ_LEN );

	/* Program queue addresses and capabilities */
	base = virt_to_bus ( gve->aq.cmd );
	writel ( bswap_32 ( base / GVE_AQ_ALIGN ),
		 ( gve->cfg + GVE_CFG_AQ_PFN ) );
	writel ( bswap_32 ( base & 0xffffffffUL ),
		 ( gve->cfg + GVE_CFG_AQ_BASE_LO ) );
	if ( sizeof ( base ) > sizeof ( uint32_t ) ) {
		writel ( bswap_32 ( ( ( uint64_t ) base ) >> 32 ),
			 ( gve->cfg + GVE_CFG_AQ_BASE_HI ) );
	} else {
		writel ( 0, ( gve->cfg + GVE_CFG_AQ_BASE_HI ) );
	}
	writel ( bswap_16 ( GVE_AQ_LEN ), ( gve->cfg + GVE_CFG_AQ_LEN ) );
	writel ( bswap_32 ( GVE_CFG_DRVSTAT_RUN ),
		 ( gve->cfg + GVE_CFG_DRVSTAT ) );

	DBGC ( gve, "GVE %p AQ at [%08lx,%08lx)\n",
	       gve, virt_to_phys ( gve->aq.cmd ),
	       ( virt_to_phys ( gve->aq.cmd ) + GVE_AQ_LEN ) );
	return 0;

	free_phys ( gve->aq.cmd, GVE_AQ_LEN );
 err_alloc:
	return rc;
}

/**
 * Destroy admin queues
 *
 * @v gve		GVE device
 */
static void gve_destroy_admin ( struct gve_nic *gve ) {
	int rc;

	/* Reset device */
	if ( ( rc = gve_reset ( gve ) ) != 0 ) {
		DBGC ( gve, "GVE %p could not free AQ: %s\n",
		       gve, strerror ( rc ) );
		/* Leak memory: there is nothing else we can do */
		return;
	}

	/* Free admin queue */
	free_phys ( gve->aq.cmd, GVE_AQ_LEN );
}

/**
 * Get next available admin queue command slot
 *
 * @v gve		GVE device
 * @ret cmd		Admin queue command
 */
static union gve_aq_command * gve_admin_command ( struct gve_nic *gve ) {
	union gve_aq_command *cmd;
	unsigned int index;

	/* Get next command slot */
	index = ( gve->aq.prod % GVE_AQ_COUNT );
	cmd = &gve->aq.cmd[index];

	/* Initialise request */
	memset ( cmd, 0, sizeof ( *cmd ) );

	return cmd;
}

/**
 * Wait for admin queue command to complete
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_admin_wait ( struct gve_nic *gve ) {
	uint32_t evt;
	unsigned int i;

	/* Wait for any outstanding commands to complete */
	for ( i = 0 ; i < GVE_AQ_MAX_WAIT_MS ; i++ ) {

		/* Check event counter */
		evt = bswap_32 ( readl ( gve->cfg + GVE_CFG_AQ_EVT ) );
		if ( evt == gve->aq.prod )
			return 0;

		/* Delay */
		mdelay ( 1 );
	}

	DBGC ( gve, "GVE %p AQ %#x timed out (completed %#x)\n",
	       gve, gve->aq.prod, evt );
	return -ETIMEDOUT;
}

/**
 * Issue admin queue command
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_admin ( struct gve_nic *gve ) {
	union gve_aq_command *cmd;
	unsigned int index;
	uint32_t opcode;
	uint32_t status;
	int rc;

	/* Ensure admin queue is idle */
	if ( ( rc = gve_admin_wait ( gve ) ) != 0 )
		return rc;

	/* Get current command slot */
	index = ( gve->aq.prod % GVE_AQ_COUNT );
	cmd = &gve->aq.cmd[index];
	opcode = be32_to_cpu ( cmd->hdr.opcode );
	DBGC2 ( gve, "GVE %p AQ %#x command %#04x:\n",
		gve, gve->aq.prod, opcode );
	DBGC2_HDA ( gve, 0, cmd, sizeof ( *cmd ) );

	/* Increment producer counter */
	gve->aq.prod++;

	/* Ring doorbell */
	writel ( bswap_32 ( gve->aq.prod ), ( gve->cfg + GVE_CFG_AQ_DB ) );

	/* Wait for command to complete */
	if ( ( rc = gve_admin_wait ( gve ) ) != 0 )
		return rc;

	/* Check command status */
	status = be32_to_cpu ( cmd->hdr.status );
	if ( status != GVE_AQ_STATUS_OK ) {
		rc = -EIO_AQ ( status );
		DBGC ( gve, "GVE %p AQ %#x failed %#04x: %#08x\n",
		       gve, ( gve->aq.prod - 1 ), opcode, status );
		DBGC_HDA ( gve, 0, cmd, sizeof ( *cmd ) );
		DBGC ( gve, "GVE %p AQ error: %s\n", gve, strerror ( rc ) );
		return rc;
	}

	DBGC2 ( gve, "GVE %p AQ %#x status:\n", gve, ( gve->aq.prod - 1 ) );
	DBGC2_HDA ( gve, 0, cmd, sizeof ( *cmd ) );
	return 0;
}

/**
 * Get device descriptor
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int gve_describe ( struct net_device *netdev ) {
	struct gve_nic *gve = netdev->priv;
	union gve_aq_command *cmd;
	int rc;

	/* Construct request */
	cmd = gve_admin_command ( gve );
	cmd->hdr.opcode = cpu_to_be32 ( GVE_AQ_DESCRIBE );
	cmd->desc.addr = cpu_to_be64 ( virt_to_bus ( &gve->desc ) );
	cmd->desc.ver = cpu_to_be32 ( GVE_AQ_DESCRIBE_VER );
	cmd->desc.len = cpu_to_be32 ( sizeof ( gve->desc ) );

	/* Issue command */
	if ( ( rc = gve_admin ( gve ) ) != 0 )
		return rc;
	DBGC2 ( gve, "GVE %p device descriptor:\n", gve );
	DBGC2_HDA ( gve, 0, &gve->desc, sizeof ( gve->desc ) );

	/* Extract parameters */
	build_assert ( sizeof ( gve->desc.mac ) == ETH_ALEN );
	memcpy ( netdev->hw_addr, &gve->desc.mac, sizeof ( gve->desc.mac ) );
	netdev->mtu = be16_to_cpu ( gve->desc.mtu );
	netdev->max_pkt_len = ( netdev->mtu + ETH_HLEN );

	DBGC ( gve, "GVE %p MAC %s (\"%s\") MTU %zd\n",
	       gve, eth_ntoa ( netdev->hw_addr ),
	       inet_ntoa ( gve->desc.mac.in ), netdev->mtu );
	return 0;
}

/**
 * Register page list
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_register ( struct gve_nic *gve ) {
	union gve_aq_command *cmd;
	int rc;

	/* Construct request */
	cmd = gve_admin_command ( gve );
	cmd->hdr.opcode = cpu_to_be32 ( GVE_AQ_REGISTER );
	cmd->reg.id = cpu_to_be32 ( GVE_AQ_REGISTER_ID );
	cmd->reg.count = cpu_to_be32 ( 1 );
	//
	extern char _textdata[];
	physaddr_t start = virt_to_bus ( _textdata );
	cmd->reg.addr = cpu_to_be64 ( start );
	cmd->reg.size = cpu_to_be64 ( 0x200000 );

	/* Issue command */
	if ( ( rc = gve_admin ( gve ) ) != 0 )
		return rc;

	return 0;
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

	/* Check PCI revision */
	pci_read_config_byte ( pci, PCI_REVISION, &gve->revision );
	DBGC ( gve, "GVE %p is revision %#02x\n", gve, gve->revision );

	/* Map configuration registers */
	cfg_start = pci_bar_start ( pci, GVE_CFG_BAR );
	gve->cfg = pci_ioremap ( pci, cfg_start, GVE_CFG_SIZE );
	if ( ! gve->cfg ) {
		rc = -ENODEV;
		goto err_cfg;
	}

	/* Reset the NIC */
	if ( ( rc = gve_reset ( gve ) ) != 0 )
		goto err_reset;

	/* Create admin queue */
	if ( ( rc = gve_create_admin ( gve ) ) != 0 )
		goto err_create_admin;

	/* Fetch MAC address */
	if ( ( rc = gve_describe ( netdev ) ) != 0 )
		goto err_describe;

	//
	gve_register ( gve );

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
	gve_destroy_admin ( gve );
 err_create_admin:
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

	/* Destroy admin queue and reset device */
	gve_destroy_admin ( gve );

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
