/*
 * Copyright (C) 2013 Michael Brown <mbrown@fensystems.co.uk>.
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
#include <ipxe/pci.h>
#include "intelx.h"

/** @file
 *
 * Intel 10 Gigabit Ethernet network card driver
 *
 */

/******************************************************************************
 *
 * MAC address
 *
 ******************************************************************************
 */

/**
 * Try to fetch initial MAC address
 *
 * @v intel		Intel device
 * @v ral0		RAL0 register address
 * @v hw_addr		Hardware address to fill in
 * @ret rc		Return status code
 */
static int intelx_try_fetch_mac ( struct intel_nic *intel, unsigned int ral0,
				  uint8_t *hw_addr ) {
	union intel_receive_address mac;

	/* Read current address from RAL0/RAH0 */
	mac.reg.low = cpu_to_le32 ( readl ( intel->regs + ral0 ) );
	mac.reg.high = cpu_to_le32 ( readl ( intel->regs + ral0 +
					     ( INTELX_RAH0 - INTELX_RAL0 ) ) );

	/* Use current address if valid */
	if ( is_valid_ether_addr ( mac.raw ) ) {
		DBGC ( intel, "INTEL %p has autoloaded MAC address %s at "
		       "%#05x\n", intel, eth_ntoa ( mac.raw ), ral0 );
		memcpy ( hw_addr, mac.raw, ETH_ALEN );
		return 0;
	}

	return -ENOENT;
}

/**
 * Fetch initial MAC address
 *
 * @v intel		Intel device
 * @v hw_addr		Hardware address to fill in
 * @ret rc		Return status code
 */
static int intelx_fetch_mac ( struct intel_nic *intel, uint8_t *hw_addr ) {
	int rc;

	/* Try to fetch address from INTELX_RAL0 */
	if ( ( rc = intelx_try_fetch_mac ( intel, INTELX_RAL0,
					   hw_addr ) ) == 0 ) {
		return 0;
	}

	/* Try to fetch address from INTELX_RAL0_ALT */
	if ( ( rc = intelx_try_fetch_mac ( intel, INTELX_RAL0_ALT,
					   hw_addr ) ) == 0 ) {
		return 0;
	}

	DBGC ( intel, "INTEL %p has no MAC address to use\n", intel );
	return -ENOENT;
}

/******************************************************************************
 *
 * Device reset
 *
 ******************************************************************************
 */

/**
 * Reset hardware
 *
 * @v intel		Intel device
 * @ret rc		Return status code
 */
static int intelx_reset ( struct intel_nic *intel ) {
	uint32_t ctrl;

	/* Perform a global software reset */
	ctrl = readl ( intel->regs + INTELX_CTRL );
	writel ( ( ctrl | INTELX_CTRL_RST | INTELX_CTRL_LRST ),
		 intel->regs + INTELX_CTRL );
	mdelay ( INTELX_RESET_DELAY_MS );

	DBGC ( intel, "INTEL %p reset (ctrl %08x)\n", intel, ctrl );
	return 0;
}

/******************************************************************************
 *
 * I2C interface
 *
 ******************************************************************************
 */

/**
 * Read I2C line status
 *
 * @v basher		Bit-bashing interface
 * @v bit_id		Bit number
 * @ret zero		Input is a logic 0
 * @ret non-zero	Input is a logic 1
 */
static int intelx_i2c_read_bit ( struct bit_basher *basher,
				 unsigned int bit_id ) {
	struct intel_nic *intel =
		container_of ( basher, struct intel_nic, i2cbit.basher );
	uint32_t i2cctl;

	/* Sanity check */
	assert ( bit_id == I2C_BIT_SDA );

	/* Read bit */
	i2cctl = readl ( intel->regs + INTELX_I2CCTL );
	return ( i2cctl & INTELX_I2CCTL_DATA_IN );
}

/**
 * Write I2C line status
 *
 * @v basher		Bit-bashing interface
 * @v bit_id		Bit number
 * @v data		Value to write
 */
static void intelx_i2c_write_bit ( struct bit_basher *basher,
				   unsigned int bit_id, unsigned long data ) {
	struct intel_nic *intel =
		container_of ( basher, struct intel_nic, i2cbit.basher );
	static const uint8_t masks[] = {
		[I2C_BIT_SCL] = INTELX_I2CCTL_CLK_OUT,
		[I2C_BIT_SDA] = INTELX_I2CCTL_DATA_OUT,
	};
	uint32_t i2cctl;
	uint32_t mask;

	/* Sanity check */
	assert ( bit_id < ( sizeof ( masks ) / sizeof ( masks[0] ) ) );

	/* Write bit */
	mask = masks[bit_id];
	i2cctl = readl ( intel->regs + INTELX_I2CCTL );
	i2cctl &= ~mask;
	if ( data )
		i2cctl |= mask;
	writel ( i2cctl, ( intel->regs + INTELX_I2CCTL ) );
}

/** I2C bit-bashing interface operations */
static struct bit_basher_operations intelx_i2c_basher_ops = {
	.read = intelx_i2c_read_bit,
	.write = intelx_i2c_write_bit,
};

/**
 * Initialise I2C bus
 *
 * @v intel		Intel device
 * @ret rc		Return status code
 */
static int intelx_i2c_init ( struct intel_nic *intel ) {
	int rc;

	/* Initialise bus */
	if ( ( rc = init_i2c_bit_basher ( &intel->i2cbit,
					  &intelx_i2c_basher_ops ) ) != 0 ) {
		DBGC ( intel, "INTEL %p could not initialise I2C bus: %s\n",
		       intel, strerror ( rc ) );
		return rc;
	}

	/* Initialise PHY device */
	init_i2c_eeprom ( &intel->phy, INTELX_PHY_ADDRESS );

	//
	struct i2c_interface *i2c = &intel->i2cbit.i2c;
	uint8_t foo[0x40];
	memset ( foo, 0xaa, sizeof ( foo ) );
	i2c->read ( i2c, &intel->phy, 0, foo, sizeof ( foo ) );
	DBGC_HDA ( intel, 0, foo, sizeof ( foo ) );

	return 0;
}

/******************************************************************************
 *
 * Link state
 *
 ******************************************************************************
 */

/**
 * Check link state
 *
 * @v netdev		Network device
 */
static void intelx_check_link ( struct net_device *netdev ) {
	struct intel_nic *intel = netdev->priv;
	uint32_t autoc;
	uint32_t links;

	/* Read autonegotiation control register */
	autoc = readl ( intel->regs + INTELX_AUTOC );
	DBGC ( intel, "INTEL %p autonegotiation control is %08x\n",
	       intel, autoc );

	/* Read link status */
	links = readl ( intel->regs + INTELX_LINKS );
	DBGC ( intel, "INTEL %p link status is %08x\n", intel, links );

	/* Update network device */
	if ( links & INTELX_LINKS_UP ) {
		netdev_link_up ( netdev );
	} else {
		netdev_link_down ( netdev );
	}
}

/******************************************************************************
 *
 * Network device interface
 *
 ******************************************************************************
 */

/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int intelx_open ( struct net_device *netdev ) {
	struct intel_nic *intel = netdev->priv;
	union intel_receive_address mac;
	uint32_t ral0;
	uint32_t rah0;
	uint32_t dmatxctl;
	uint32_t fctrl;
	uint32_t srrctl;
	uint32_t hlreg0;
	uint32_t maxfrs;
	uint32_t rdrxctl;
	uint32_t rxctrl;
	uint32_t dca_rxctrl;
	int rc;

	/* Create transmit descriptor ring */
	if ( ( rc = intel_create_ring ( intel, &intel->tx ) ) != 0 )
		goto err_create_tx;

	/* Create receive descriptor ring */
	if ( ( rc = intel_create_ring ( intel, &intel->rx ) ) != 0 )
		goto err_create_rx;

	/* Program MAC address */
	memset ( &mac, 0, sizeof ( mac ) );
	memcpy ( mac.raw, netdev->ll_addr, sizeof ( mac.raw ) );
	ral0 = le32_to_cpu ( mac.reg.low );
	rah0 = ( le32_to_cpu ( mac.reg.high ) | INTELX_RAH0_AV );
	writel ( ral0, intel->regs + INTELX_RAL0 );
	writel ( rah0, intel->regs + INTELX_RAH0 );
	writel ( ral0, intel->regs + INTELX_RAL0_ALT );
	writel ( rah0, intel->regs + INTELX_RAH0_ALT );

	/* Allocate interrupt vectors */
	writel ( ( INTELX_IVAR_RX0_DEFAULT | INTELX_IVAR_RX0_VALID |
		   INTELX_IVAR_TX0_DEFAULT | INTELX_IVAR_TX0_VALID ),
		 intel->regs + INTELX_IVAR );

	/* Enable transmitter  */
	dmatxctl = readl ( intel->regs + INTELX_DMATXCTL );
	dmatxctl |= INTELX_DMATXCTL_TE;
	writel ( dmatxctl, intel->regs + INTELX_DMATXCTL );

	/* Configure receive filter */
	fctrl = readl ( intel->regs + INTELX_FCTRL );
	fctrl |= ( INTELX_FCTRL_BAM | INTELX_FCTRL_UPE | INTELX_FCTRL_MPE );
	writel ( fctrl, intel->regs + INTELX_FCTRL );

	/* Configure receive buffer sizes */
	srrctl = readl ( intel->regs + INTELX_SRRCTL );
	srrctl &= ~INTELX_SRRCTL_BSIZE_MASK;
	srrctl |= INTELX_SRRCTL_BSIZE_DEFAULT;
	writel ( srrctl, intel->regs + INTELX_SRRCTL );

	/* Configure jumbo frames.  Required to allow the extra 4-byte
	 * headroom for VLANs, since we don't use the hardware's
	 * native VLAN offload.
	 */
	hlreg0 = readl ( intel->regs + INTELX_HLREG0 );
	hlreg0 |= INTELX_HLREG0_JUMBOEN;
	writel ( hlreg0, intel->regs + INTELX_HLREG0 );

	/* Configure frame size */
	maxfrs = readl ( intel->regs + INTELX_MAXFRS );
	maxfrs &= ~INTELX_MAXFRS_MFS_MASK;
	maxfrs |= INTELX_MAXFRS_MFS_DEFAULT;
	writel ( maxfrs, intel->regs + INTELX_MAXFRS );

	/* Configure receive DMA */
	rdrxctl = readl ( intel->regs + INTELX_RDRXCTL );
	rdrxctl |= INTELX_RDRXCTL_SECRC;
	writel ( rdrxctl, intel->regs + INTELX_RDRXCTL );

	/* Clear "must-be-zero" bit for direct cache access (DCA).  We
	 * leave DCA disabled anyway, but if we do not clear this bit
	 * then the received packets contain garbage data.
	 */
	dca_rxctrl = readl ( intel->regs + INTELX_DCA_RXCTRL );
	dca_rxctrl &= ~INTELX_DCA_RXCTRL_MUST_BE_ZERO;
	writel ( dca_rxctrl, intel->regs + INTELX_DCA_RXCTRL );

	/* Enable receiver */
	rxctrl = readl ( intel->regs + INTELX_RXCTRL );
	rxctrl |= INTELX_RXCTRL_RXEN;
	writel ( rxctrl, intel->regs + INTELX_RXCTRL );

	/* Fill receive ring */
	intel_refill_rx ( intel );

	//
	DBGC ( intel, "*** hacking AUTOC.LMS\n" );
	//writel ( 0xc09c5004, intel->regs + INTELX_AUTOC );
	writel ( ( 0xc09c1004 | ( 1 << 13 ) ), intel->regs + INTELX_AUTOC );

	/* Update link state */
	intelx_check_link ( netdev );

	return 0;

	intel_destroy_ring ( intel, &intel->rx );
 err_create_rx:
	intel_destroy_ring ( intel, &intel->tx );
 err_create_tx:
	return rc;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void intelx_close ( struct net_device *netdev ) {
	struct intel_nic *intel = netdev->priv;
	uint32_t rxctrl;
	uint32_t dmatxctl;

	/* Disable receiver */
	rxctrl = readl ( intel->regs + INTELX_RXCTRL );
	rxctrl &= ~INTELX_RXCTRL_RXEN;
	writel ( rxctrl, intel->regs + INTELX_RXCTRL );

	/* Disable transmitter  */
	dmatxctl = readl ( intel->regs + INTELX_DMATXCTL );
	dmatxctl &= ~INTELX_DMATXCTL_TE;
	writel ( dmatxctl, intel->regs + INTELX_DMATXCTL );

	/* Destroy receive descriptor ring */
	intel_destroy_ring ( intel, &intel->rx );

	/* Discard any unused receive buffers */
	intel_empty_rx ( intel );

	/* Destroy transmit descriptor ring */
	intel_destroy_ring ( intel, &intel->tx );

	/* Reset the NIC, to flush the transmit and receive FIFOs */
	intelx_reset ( intel );
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void intelx_poll ( struct net_device *netdev ) {
	struct intel_nic *intel = netdev->priv;
	uint32_t eicr;

	/* Check for and acknowledge interrupts */
	eicr = readl ( intel->regs + INTELX_EICR );
	if ( ! eicr )
		return;

	/* Poll for TX completions, if applicable */
	if ( eicr & INTELX_EIRQ_TX0 )
		intel_poll_tx ( netdev );

	/* Poll for RX completions, if applicable */
	if ( eicr & ( INTELX_EIRQ_RX0 | INTELX_EIRQ_RXO ) )
		intel_poll_rx ( netdev );

	/* Report receive overruns */
	if ( eicr & INTELX_EIRQ_RXO )
		netdev_rx_err ( netdev, NULL, -ENOBUFS );

	/* Check link state, if applicable */
	if ( eicr & INTELX_EIRQ_LSC )
		intelx_check_link ( netdev );

	/* Refill RX ring */
	intel_refill_rx ( intel );
}

/**
 * Enable or disable interrupts
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void intelx_irq ( struct net_device *netdev, int enable ) {
	struct intel_nic *intel = netdev->priv;
	uint32_t mask;

	mask = ( INTELX_EIRQ_LSC | INTELX_EIRQ_RXO | INTELX_EIRQ_TX0 |
		 INTELX_EIRQ_RX0 );
	if ( enable ) {
		writel ( mask, intel->regs + INTELX_EIMS );
	} else {
		writel ( mask, intel->regs + INTELX_EIMC );
	}
}

/** Network device operations */
static struct net_device_operations intelx_operations = {
	.open		= intelx_open,
	.close		= intelx_close,
	.transmit	= intel_transmit,
	.poll		= intelx_poll,
	.irq		= intelx_irq,
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
static int intelx_probe ( struct pci_device *pci ) {
	struct net_device *netdev;
	struct intel_nic *intel;
	int rc;

	/* Allocate and initialise net device */
	netdev = alloc_etherdev ( sizeof ( *intel ) );
	if ( ! netdev ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	netdev_init ( netdev, &intelx_operations );
	intel = netdev->priv;
	pci_set_drvdata ( pci, netdev );
	netdev->dev = &pci->dev;
	memset ( intel, 0, sizeof ( *intel ) );
	intel->port = PCI_FUNC ( pci->busdevfn );
	intel_init_ring ( &intel->tx, INTEL_NUM_TX_DESC, INTELX_TD,
			  intel_describe_tx );
	intel_init_ring ( &intel->rx, INTEL_NUM_RX_DESC, INTELX_RD,
			  intel_describe_rx );

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Map registers */
	intel->regs = pci_ioremap ( pci, pci->membase, INTEL_BAR_SIZE );
	if ( ! intel->regs ) {
		rc = -ENODEV;
		goto err_ioremap;
	}

	/* Configure DMA */
	intel->dma = &pci->dma;
	dma_set_mask_64bit ( intel->dma );
	netdev->dma = intel->dma;

	//
	uint32_t autoc;
	autoc = readl ( intel->regs + INTELX_AUTOC );
	DBGC ( intel, "*** autoc = %#08x\n", autoc );

	/* Reset the NIC */
	if ( ( rc = intelx_reset ( intel ) ) != 0 )
		goto err_reset;

	/* Fetch MAC address */
	if ( ( rc = intelx_fetch_mac ( intel, netdev->hw_addr ) ) != 0 )
		goto err_fetch_mac;

	/* Initialise I2C */
	if ( ( rc = intelx_i2c_init ( intel ) ) != 0 )
		goto err_i2c;

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	intelx_check_link ( netdev );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
 err_i2c:
 err_fetch_mac:
	intelx_reset ( intel );
 err_reset:
	iounmap ( intel->regs );
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
static void intelx_remove ( struct pci_device *pci ) {
	struct net_device *netdev = pci_get_drvdata ( pci );
	struct intel_nic *intel = netdev->priv;

	/* Unregister network device */
	unregister_netdev ( netdev );

	/* Reset the NIC */
	intelx_reset ( intel );

	/* Free network device */
	iounmap ( intel->regs );
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** PCI device IDs */
static struct pci_device_id intelx_nics[] = {
	PCI_ROM ( 0x8086, 0x10f7, "82599-kx4", "82599 (KX/KX4)", 0 ),
	PCI_ROM ( 0x8086, 0x10f8, "82599-combo-backplane", "82599 (combined backplane; KR/KX4/KX)", 0 ),
	PCI_ROM ( 0x8086, 0x10f9, "82599-cx4", "82599 (CX4)", 0 ),
	PCI_ROM ( 0x8086, 0x10fb, "82599-sfp", "82599 (SFI/SFP+)", 0 ),
	PCI_ROM ( 0x8086, 0x10fc, "82599-xaui", "82599 (XAUI/BX4)", 0 ),
	PCI_ROM ( 0x8086, 0x151c, "82599-tn", "82599 (TN)", 0 ),
	PCI_ROM ( 0x8086, 0x1528, "x540t", "X540-AT2/X540-BT2", 0 ),
	PCI_ROM ( 0x8086, 0x154d, "82599-sfp-sf2", "82599 (SFI/SFP+)", 0 ),
	PCI_ROM ( 0x8086, 0x1557, "82599en-sfp", "82599 (Single Port SFI Only)", 0 ),
	PCI_ROM ( 0x8086, 0x1560, "x540t1", "X540-AT2/X540-BT2 (with single port NVM)", 0 ),
	PCI_ROM ( 0x8086, 0x1563, "x550t2", "X550-T2", 0 ),
	PCI_ROM ( 0x8086, 0x15ab, "x552", "X552", 0 ),
	PCI_ROM ( 0x8086, 0x15c8, "x553t", "X553/X557-AT", 0 ),
	PCI_ROM ( 0x8086, 0x15ce, "x553-sfp", "X553 (SFP+)", 0 ),
	PCI_ROM ( 0x8086, 0x15e4, "x553a", "X553", 0 ),
	PCI_ROM ( 0x8086, 0x15e5, "x553", "X553", 0 ),
};

/** PCI driver */
struct pci_driver intelx_driver __pci_driver = {
	.ids = intelx_nics,
	.id_count = ( sizeof ( intelx_nics ) / sizeof ( intelx_nics[0] ) ),
	.probe = intelx_probe,
	.remove = intelx_remove,
};
