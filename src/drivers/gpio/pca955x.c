/*
 * Copyright (C) 2025 Michael Brown <mbrown@fensystems.co.uk>.
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

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ipxe/devtree.h>
#include <ipxe/fdt.h>
#include <ipxe/i2c.h>
#include <ipxe/gpio.h>
#include "pca955x.h"

/** @file
 *
 * Philips Semiconductors PCA9555/PCA9554 and derivative I2C GPIO controllers
 *
 */

/**
 * Automatically determine register shift
 *
 * @v i2cdev		I2C device
 * @ret shift		Register shift, or negative error
 */
static int pca955x_autosize ( struct i2c_device *i2cdev ) {
	unsigned int address;
	int shift;
	int rc;

	/* The polarity inversion register affects only reads from the
	 * input register: it does not affect the output status in any
	 * way.  We can therefore perform a read-modify-restore test
	 * on the polarity inversion register without causing any side
	 * effects.
	 *
	 * The polarity inversion register is at offset 2<<N, where N
	 * is the register shift.  If the device's real register shift
	 * is smaller than N, then this offset will correspond to
	 * either the read-only input register (if the device
	 * truncates register offsets) or to an unimplemented
	 * register.
	 *
	 * We can therefore set N to a maximal realistic value and use
	 * a read-modify-restore test on the polarity inversion
	 * register, decreasing N until the test succeeds.  Any
	 * iterations on which N is larger than the real register
	 * shift will fail the test harmlessly, by attempting to write
	 * to either the read-only input register or to an
	 * unimplemented register.
	 */
	for ( shift = PCA955X_MAX_SHIFT ; shift >= 0 ; shift-- ) {

		/* Test if polarity register is writable at this shift */
		address = PCA955X_POLARITY ( shift );
		if ( ( rc = i2c_writable ( i2cdev, address ) ) == 0 )
			return shift;
	}

	return -ENODEV;
}

/**
 * Probe devicetree device
 *
 * @v dt		Devicetree device
 * @v offset		Starting node offset
 * @ret rc		Return status code
 */
static int pca955x_probe ( struct dt_device *dt, unsigned int offset ) {
	struct xhci_device *xhci;
	uint32_t gctl;
	int rc;

	/* Allocate and initialise structure */
	xhci = zalloc ( sizeof ( *xhci ) );
	if ( ! xhci ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	xhci->dev = &dt->dev;
	xhci->dma = &dt->dma;

	/* Map registers */
	xhci->regs = dt_ioremap ( dt, offset, 0, 0 );
	if ( ! xhci->regs ) {
		rc = -ENODEV;
		goto err_ioremap;
	}

	/* Reset via global core control register */
	gctl = readl ( xhci->regs + PCA955X_GCTL );
	writel ( ( gctl | PCA955X_GCTL_RESET ), ( xhci->regs + PCA955X_GCTL ) );
	mdelay ( 100 );
	writel ( gctl, ( xhci->regs + PCA955X_GCTL ) );

	/* Configure as a host controller */
	gctl &= ~PCA955X_GCTL_PRTDIR_MASK;
	gctl |= PCA955X_GCTL_PRTDIR_HOST;
	writel ( gctl, ( xhci->regs + PCA955X_GCTL ) );

	/* Initialise xHCI device */
	xhci_init ( xhci );

	//
	struct gpios *hubswitch = gpios_find ( BUS_TYPE_DT, 0x39 );
	DBG ( "*** hubswitch via %s\n", hubswitch->dev->name );
	gpio_out ( &hubswitch->gpio[0x04], 1 );
	gpio_config ( &hubswitch->gpio[0x04], GPIO_CFG_OUTPUT );
	//
	struct gpios *vbus = gpios_find ( BUS_TYPE_DT, 0x55 );
	DBG ( "*** vbus via %s\n", vbus->dev->name );
	gpio_out ( &vbus->gpio[0x16], 1 );
	gpio_config ( &vbus->gpio[0x16], GPIO_CFG_OUTPUT );

	/* Register xHCI device */
	if ( ( rc = xhci_register ( xhci ) ) != 0 ) {
		DBGC ( xhci, "XHCI %s could not register: %s\n",
		       xhci->name, strerror ( rc ) );
		goto err_register;
	}

	dt_set_drvdata ( dt, xhci );
	return 0;

	xhci_unregister ( xhci );
 err_register:
	iounmap ( xhci->regs );
 err_ioremap:
	free ( xhci );
 err_alloc:
	return rc;
}

/**
 * Remove devicetree device
 *
 * @v dt		Devicetree device
 */
static void pca955x_remove ( struct dt_device *dt ) {
	struct xhci_device *xhci = dt_get_drvdata ( dt );

	/* Unregister xHCI device */
	xhci_unregister ( xhci );

	/* Unmap registers */
	iounmap ( xhci->regs );

	/* Free device */
	free ( xhci );
}

/** PCA955x compatible model identifiers */
static const char * pca955x_ids[] = {
	"nxp,pca9557",
};

/** PCA955x devicetree driver */
struct dt_driver pca955x_driver __dt_driver = {
	.name = "pca955x",
	.ids = pca955x_ids,
	.id_count = ( sizeof ( pca955x_ids ) / sizeof ( pca955x_ids[0] ) ),
	.probe = pca955x_probe,
	.remove = pca955x_remove,
};
