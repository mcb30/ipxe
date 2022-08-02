/*
 * Copyright (C) 2022 Michael Brown <mbrown@fensystems.co.uk>.
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
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <byteswap.h>
#include <ipxe/netdevice.h>
#include <ipxe/ethernet.h>
#include <ipxe/if_ether.h>
#include <ipxe/vlan.h>
#include <ipxe/iobuf.h>
#include <ipxe/pci.h>
#include <ipxe/version.h>
#include "ice.h"

/** @file
 *
 * Intel 100 Gigabit Ethernet network card driver
 *
 */

/******************************************************************************
 *
 * Admin queue
 *
 ******************************************************************************
 */

/**
 * Get admin queue command parameters
 *
 * @v cmd		Command descriptor
 * @ret params		Command parameters
 */
static inline __attribute__ (( always_inline )) union ice_admin_params *
ice_admin_command_parameters ( struct intelxl_admin_descriptor *cmd ) {
	union intelxl_admin_params *params;

	params = &cmd->params;
	return container_of ( &params->buffer, union ice_admin_params, buffer );
}

/**
 * Get next admin command queue data buffer
 *
 * @v intelxl		Intel device
 * @ret buf		Data buffer
 */
static inline __attribute__ (( always_inline )) union ice_admin_buffer *
ice_admin_command_buffer ( struct intelxl_nic *intelxl ) {
	union intelxl_admin_buffer *buf;

	buf = intelxl_admin_command_buffer ( intelxl );
	return container_of ( &buf->pad, union ice_admin_buffer, pad );
}

/**
 * Get firmware version
 *
 * @v intelxl		Intel device
 * @ret rc		Return status code
 */
static int ice_admin_version ( struct intelxl_nic *intelxl ) {
	struct intelxl_admin_descriptor *cmd;
	union ice_admin_params *params;
	struct ice_admin_version_params *version;
	unsigned int api;
	int rc;

	/* Populate descriptor */
	cmd = intelxl_admin_command_descriptor ( intelxl );
	cmd->opcode = cpu_to_le16 ( INTELXL_ADMIN_VERSION );
	params = ice_admin_command_parameters ( cmd );
	version = &params->version;

	/* Issue command */
	if ( ( rc = intelxl_admin_command ( intelxl ) ) != 0 )
		return rc;
	api = version->api.major;
	DBGC ( intelxl, "ICE %p firmware v%d/%d.%d.%d API v%d/%d.%d.%d\n",
	       intelxl, version->firmware.branch, version->firmware.major,
	       version->firmware.minor, version->firmware.patch,
	       version->api.branch, version->api.major, version->api.minor,
	       version->api.patch );

	/* Check for API compatibility */
	if ( api > INTELXL_ADMIN_API_MAJOR ) {
		DBGC ( intelxl, "ICE %p unsupported API v%d\n", intelxl, api );
		return -ENOTSUP;
	}

	return 0;
}

/**
 * Get switch configuration
 *
 * @v intelxl		Intel device
 * @ret rc		Return status code
 */
static int ice_admin_switch ( struct intelxl_nic *intelxl ) {
	struct intelxl_admin_descriptor *cmd;
	struct intelxl_admin_switch_params *sw;
	union ice_admin_buffer *buf;
	uint32_t next = 0;
	uint16_t seid;
	uint16_t type;
	int rc;

	/* Get each configuration in turn */
	do {
		/* Populate descriptor */
		cmd = intelxl_admin_command_descriptor ( intelxl );
		cmd->opcode = cpu_to_le16 ( INTELXL_ADMIN_SWITCH );
		cmd->flags = cpu_to_le16 ( INTELXL_ADMIN_FL_BUF );
		cmd->len = cpu_to_le16 ( intelxl->api->sw_buf_len );
		sw = &cmd->params.sw;
		sw->next = next;
		buf = ice_admin_command_buffer ( intelxl );

		/* Issue command */
		if ( ( rc = intelxl_admin_command ( intelxl ) ) != 0 )
			return rc;
		seid = le16_to_cpu ( buf->sw.cfg[0].seid );

		/* Dump raw configuration */
		DBGC2 ( intelxl, "ICE %p SEID %#04x:\n", intelxl, seid );
		DBGC2_HDA ( intelxl, 0, &buf->sw.cfg[0],
			    sizeof ( buf->sw.cfg[0] ) );

		/* Parse response */
		type = ( seid & ICE_ADMIN_SWITCH_TYPE_MASK );
		if ( type == ICE_ADMIN_SWITCH_TYPE_VSI ) {
			intelxl->vsi = ( seid & ~ICE_ADMIN_SWITCH_TYPE_MASK );
			DBGC ( intelxl, "INTELXL %p VSI %#04x uplink %#04x "
			       "func %#04x\n", intelxl, intelxl->vsi,
			       le16_to_cpu ( buf->sw.cfg[0].uplink ),
			       le16_to_cpu ( buf->sw.cfg[0].func ) );
		}

	} while ( ( next = sw->next ) );

	/* Check that we found a VSI */
	if ( ! intelxl->vsi ) {
		DBGC ( intelxl, "ICE %p has no VSI\n", intelxl );
		return -ENOENT;
	}

	return 0;
}

/**
 * Check if scheduler node is a parent (i.e. non-leaf) node
 *
 * @v branch		Scheduler topology branch
 * @v node		Scheduler topology node
 * @ret child		Any child node, or NULL if not found
 */
static struct ice_admin_schedule_node *
ice_admin_schedule_is_parent ( struct ice_admin_schedule_branch *branch,
			       struct ice_admin_schedule_node *node ) {
	unsigned int count = le16_to_cpu ( branch->count );
	struct ice_admin_schedule_node *child;
	unsigned int i;

	/* Find a child element, if any */
	for ( i = 0 ; i < count ; i++ ) {
		child = &branch->node[i];
		if ( child->parent == node->teid )
			return child;
	}

	return NULL;
}

/**
 * Query default scheduling tree topology
 *
 * @v intelxl		Intel device
 * @ret rc		Return status code
 */
static int ice_admin_schedule ( struct intelxl_nic *intelxl ) {
	struct intelxl_admin_descriptor *cmd;
	union ice_admin_params *params;
	struct ice_admin_schedule_params *sched;
	union ice_admin_buffer *buf;
	struct ice_admin_schedule_branch *branch;
	struct ice_admin_schedule_node *node;
	int i;
	int rc;

	/* Populate descriptor */
	cmd = intelxl_admin_command_descriptor ( intelxl );
	cmd->opcode = cpu_to_le16 ( ICE_ADMIN_SCHEDULE );
	cmd->flags = cpu_to_le16 ( INTELXL_ADMIN_FL_BUF );
	cmd->len = cpu_to_le16 ( sizeof ( buf->sched ) );
	params = ice_admin_command_parameters ( cmd );
	sched = &params->sched;
	buf = ice_admin_command_buffer ( intelxl );

	/* Issue command */
	if ( ( rc = intelxl_admin_command ( intelxl ) ) != 0 )
		return rc;

	/* Sanity checks */
	if ( ! sched->branches ) {
		DBGC ( intelxl, "ICE %p topology has no branches\n", intelxl );
		return -EINVAL;
	}
	branch = buf->sched.branch;

	/* Identify leaf node */
	for ( i = ( le16_to_cpu ( branch->count ) - 1 ) ; i >= 0 ; i-- ) {
		node = &branch->node[i];
		if ( ! ice_admin_schedule_is_parent ( branch, node ) ) {
			intelxl->teid = le32_to_cpu ( node->teid );
			DBGC2 ( intelxl, "ICE %p TEID %#08x type %d\n",
				intelxl, intelxl->teid, node->config.type );
			break;
		}
	}
	if ( ! intelxl->teid ) {
		DBGC ( intelxl, "ICE %p found no leaf TEID\n", intelxl );
		return -EINVAL;
	}

	return 0;
}






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
static int ice_probe ( struct pci_device *pci ) {
	struct net_device *netdev;
	struct intelxl_nic *intelxl;
	uint32_t pffunc_rid;
	uint32_t pfgen_portnum;
	int rc;

	/* Allocate and initialise net device */
	netdev = alloc_etherdev ( sizeof ( *intelxl ) );
	if ( ! netdev ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	netdev_init ( netdev, &ice_operations );
	intelxl = netdev->priv;
	pci_set_drvdata ( pci, netdev );
	netdev->dev = &pci->dev;
	memset ( intelxl, 0, sizeof ( *intelxl ) );
	intelxl->intr = ICE_GLINT_DYN_CTL;
	intelxl_init_admin ( &intelxl->command, INTELXL_ADMIN_CMD,
			     &intelxl_admin_offsets );
	intelxl_init_admin ( &intelxl->event, INTELXL_ADMIN_EVT,
			     &intelxl_admin_offsets );
	ice_init_ring ( &intelxl->tx, INTELXL_TX_NUM_DESC,
			sizeof ( intelxl->tx.desc.tx[0] ) );
	ice_init_ring ( &intelxl->rx, INTELXL_RX_NUM_DESC,
			sizeof ( intelxl->rx.desc.rx[0] ) );

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Map registers */
	intelxl->regs = pci_ioremap ( pci, pci->membase, ICE_BAR_SIZE );
	if ( ! intelxl->regs ) {
		rc = -ENODEV;
		goto err_ioremap;
	}

	/* Configure DMA */
	intelxl->dma = &pci->dma;
	dma_set_mask_64bit ( intelxl->dma );
	netdev->dma = intelxl->dma;

	/* Locate PCI Express capability */
	intelxl->exp = pci_find_capability ( pci, PCI_CAP_ID_EXP );
	if ( ! intelxl->exp ) {
		DBGC ( intelxl, "ICE %p missing PCIe capability\n",
		       intelxl );
		rc = -ENXIO;
		goto err_exp;
	}

	/* Reset the function via PCIe FLR */
	pci_reset ( pci, intelxl->exp );

	/* Enable MSI-X dummy interrupt */
	if ( ( rc = intelxl_msix_enable ( intelxl, pci,
					  INTELXL_MSIX_VECTOR ) ) != 0 )
		goto err_msix;

	/* Open admin queues */
	if ( ( rc = intelxl_open_admin ( intelxl ) ) != 0 )
		goto err_open_admin;

	/* Get firmware version */
	if ( ( rc = ice_admin_version ( intelxl ) ) != 0 )
		goto err_admin_version;

	/* Clear PXE mode */
	if ( ( rc = intelxl_admin_clear_pxe ( intelxl ) ) != 0 )
		goto err_admin_clear_pxe;

	/* Get switch configuration */
	if ( ( rc = ice_admin_switch ( intelxl ) ) != 0 )
		goto err_admin_switch;

	/* Query scheduler topology */
	if ( ( rc = ice_admin_schedule ( intelxl ) ) != 0 )
		goto err_admin_schedule;

	/* Get MAC address */
	if ( ( rc = intelxl_admin_mac_read ( netdev,
					     ice_admin_mac_read_lan ) ) != 0 )
		goto err_admin_mac_read;

	/* Get maximum frame size */
	if ( ( rc = intelxl_admin_link ( netdev,
					 intelxl_admin_link_mfs ) ) != 0 )
		goto err_admin_link_mfs;

	/* Get function and port number */
	pffunc_rid = readl ( intelxl->regs + ICE_PFFUNC_RID );
	intelxl->pf = ICE_PFFUNC_RID_FUNC_NUM ( pffunc_rid );
	pfgen_portnum = readl ( intelxl->regs + ICE_PFGEN_PORTNUM );
	intelxl->port = ICE_PFGEN_PORTNUM_PORT_NUM ( pfgen_portnum );
	DBGC ( intelxl, "ICE %p PF %d using port %d\n",
	       intelxl, intelxl->pf, intelxl->port );

	/* Configure queue register addresses */
	intelxl->tx.tail = ICE_QTX_COMM_DBELL;
	intelxl->rx.reg = ICE_QRX_CTRL;
	intelxl->rx.tail = ICE_QRX_TAIL;

	/* Configure interrupt causes */
	writel ( ( ICE_QINT_TQCTL_ITR_INDX_NONE | ICE_QINT_TQCTL_CAUSE_ENA ),
		 intelxl->regs + ICE_QINT_TQCTL );
	writel ( ( ICE_QINT_RQCTL_ITR_INDX_NONE | ICE_QINT_RQCTL_CAUSE_ENA ),
		 intelxl->regs + ICE_QINT_RQCTL );

	/* Set a default value for the queue context flex extension,
	 * since this register erroneously retains its value across at
	 * least a PCIe FLR.
	 */
	writel ( ( ICE_QRX_FLXP_CNTXT_RXDID_IDX_LEGACY_32 |
		   ICE_QRX_FLXP_CNTXT_RXDID_PRIO_MAX ),
		 intelxl->regs + ICE_QRX_FLXP_CNTXT );

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	intelxl_admin_link ( netdev, intelxl_admin_link_status );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
 err_admin_link_mfs:
 err_admin_mac_read:
 err_admin_schedule:
 err_admin_switch:
 err_admin_clear_pxe:
 err_admin_version:
	intelxl_close_admin ( intelxl );
 err_open_admin:
	intelxl_msix_disable ( intelxl, pci, INTELXL_MSIX_VECTOR );
 err_msix:
	pci_reset ( pci, intelxl->exp );
 err_exp:
	iounmap ( intelxl->regs );
 err_ioremap:
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
static void ice_remove ( struct pci_device *pci ) {
	struct net_device *netdev = pci_get_drvdata ( pci );
	struct intelxl_nic *intelxl = netdev->priv;

	/* Unregister network device */
	unregister_netdev ( netdev );

	/* Close admin queues */
	intelxl_close_admin ( intelxl );

	/* Disable MSI-X dummy interrupt */
	intelxl_msix_disable ( intelxl, pci, INTELXL_MSIX_VECTOR );

	/* Reset the NIC */
	pci_reset ( pci, intelxl->exp );

	/* Free network device */
	iounmap ( intelxl->regs );
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** PCI device IDs */
static struct pci_device_id ice_nics[] = {
	PCI_ROM ( 0x8086, 0x124c, "e823l-bp", "E823-L backplane", 0 ),
	PCI_ROM ( 0x8086, 0x124d, "e823l-sfp", "E823-L SFP", 0 ),
	PCI_ROM ( 0x8086, 0x124e, "e823l-10gt", "E823-L 10GBASE-T", 0 ),
	PCI_ROM ( 0x8086, 0x124f, "e823l-1g", "E823-L 1GbE", 0 ),
	PCI_ROM ( 0x8086, 0x151d, "e823l-qsfp", "E823-L QSFP", 0 ),
	PCI_ROM ( 0x8086, 0x1591, "e810c-bp", "E810-C backplane", 0 ),
	PCI_ROM ( 0x8086, 0x1592, "e810c-qsfp", "E810-C QSFP", 0 ),
	PCI_ROM ( 0x8086, 0x1593, "e810c-sfp", "E810-C SFP", 0 ),
	PCI_ROM ( 0x8086, 0x1599, "e810-xxv-bp", "E810-XXV backplane", 0 ),
	PCI_ROM ( 0x8086, 0x159a, "e810-xxv-qsfp", "E810-XXV QSFP", 0 ),
	PCI_ROM ( 0x8086, 0x159b, "e810-xxv-sfp", "E810-XXV SFP", 0 ),
	PCI_ROM ( 0x8086, 0x188a, "e823c-bp", "E823-C backplane", 0 ),
	PCI_ROM ( 0x8086, 0x188b, "e823c-qsfp", "E823-C QSFP", 0 ),
	PCI_ROM ( 0x8086, 0x188c, "e823c-sfp", "E823-C SFP", 0 ),
	PCI_ROM ( 0x8086, 0x188d, "e823c-10gt", "E823-C 10GBASE-T", 0 ),
	PCI_ROM ( 0x8086, 0x188e, "e823c-1g", "E823-C 1GbE", 0 ),
	PCI_ROM ( 0x8086, 0x1890, "e822c-bp", "E822-C backplane", 0 ),
	PCI_ROM ( 0x8086, 0x1891, "e822c-qsfp", "E822-C QSFP", 0 ),
	PCI_ROM ( 0x8086, 0x1892, "e822c-sfp", "E822-C SFP", 0 ),
	PCI_ROM ( 0x8086, 0x1893, "e822c-10gt", "E822-C 10GBASE-T", 0 ),
	PCI_ROM ( 0x8086, 0x1894, "e822c-1g", "E822-C 1GbE", 0 ),
	PCI_ROM ( 0x8086, 0x1897, "e822l-bp", "E822-L backplane", 0 ),
	PCI_ROM ( 0x8086, 0x1898, "e822l-sfp", "E822-L SFP", 0 ),
	PCI_ROM ( 0x8086, 0x1899, "e822l-10gt", "E822-L 10GBASE-T", 0 ),
	PCI_ROM ( 0x8086, 0x189a, "e822l-1g", "E822-L 1GbE", 0 ),
};

/** PCI driver */
struct pci_driver ice_driver __pci_driver = {
	.ids = ice_nics,
	.id_count = ( sizeof ( ice_nics ) / sizeof ( ice_nics[0] ) ),
	.probe = ice_probe,
	.remove = ice_remove,
};



//
// checks:
//
// - removed all _v2
// - DBG() using ICE not INTELXL
// - intelxl_admin_switch using uint32_t next in struct and func
