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
#include <unistd.h>
#include <errno.h>
#include <ipxe/devtree.h>
#include <ipxe/fdt.h>
#include <ipxe/i2c.h>
#include "dwi2c.h"

/** @file
 *
 * Synopsys DesignWare I2C driver
 *
 */

/**
 * Wait for data transfer to complete
 *
 * @v dwi2c		DesignWare I2C controller
 * @v bit		Status bit for which to wait
 * @ret rc		Return status code
 */
static int dwi2c_wait ( struct dwi2c *dwi2c, uint32_t bit ) {
	uint32_t sta;
	unsigned int i;

	/* Wait for specified bit to be set */
	for ( i = 0 ; i < DWI2C_MAX_WAIT_US ; i++ ) {

		/* Check for both FIFOs being empty */
		sta = readl ( dwi2c->regs + DWI2C_STA );
		if ( sta & bit )
			return 0;

		/* Delay */
		udelay ( 1 );
	}

	DBGC ( dwi2c, "DWI2C %s timed out (STA %#08x)\n", dwi2c->name, sta );
	return -ETIMEDOUT;
}

/**
 * Write data byte to I2C bus
 *
 * @v dwi2c		DesignWare I2C controller
 * @v byte		Data byte
 * @ret rc		Return status code
 */
static int dwi2c_write_byte ( struct dwi2c *dwi2c, uint8_t byte ) {
	int rc;

	/* Initiate write */
	writel ( byte, ( dwi2c->regs + DWI2C_DAT ) );

	/* Wait for write to complete */
	if ( ( rc = dwi2c_wait ( dwi2c, DWI2C_STA_TFE ) ) != 0 )
		return rc;

	return 0;
}

/**
 * Read data byte from I2C bus
 *
 * @v dwi2c		DesignWare I2C controller
 * @ret byte		Data byte, or negative error
 */
static int dwi2c_read_byte ( struct dwi2c *dwi2c ) {
	uint32_t dat;
	uint8_t byte;
	int rc;

	/* Initiate read */
	writel ( DWI2C_DAT_READ, ( dwi2c->regs + DWI2C_DAT ) );

	/* Wait for read to complete */
	if ( ( rc = dwi2c_wait ( dwi2c, DWI2C_STA_RFNE ) ) != 0 )
		return rc;

	/* Read byte */
	dat = readl ( dwi2c->regs + DWI2C_DAT );
	byte = ( dat & DWI2C_DAT_MASK );

	return byte;
}

/**
 * Start I2C transfer
 *
 * @v dwi2c		DesignWare I2C controller
 * @v i2cdev		I2C device
 * @v offset		Starting offset within the device
 * @ret rc		Return status code
 */
static int dwi2c_start ( struct dwi2c *dwi2c, struct i2c_device *i2cdev,
			 unsigned int offset ) {
	unsigned int len = i2cdev->word_addr_len;
	unsigned int address;
	uint32_t tar;
	uint8_t byte;
	int rc;

	/* Construct target address */
	address = i2c_address ( i2cdev, offset );
	tar = ( address & DWI2C_TAR_MASK );
	if ( tar != address )
		tar |= DWI2C_TAR_10BIT;

	/* Write target address */
	writel ( tar, ( dwi2c->regs + DWI2C_TAR ) );

	/* Enable I2C transfers */
	writel ( DWI2C_ENA_I2C, ( dwi2c->regs + DWI2C_ENA ) );

	/* Write starting offset */
	while ( len-- ) {
		byte = ( offset >> ( 8 * len ) );
		if ( ( rc = dwi2c_write_byte ( dwi2c, byte ) ) != 0 )
			return rc;
	}

	return 0;
}

/**
 * Stop I2C transfer
 *
 * @v dwi2c		DesignWare I2C controller
 */
static inline void dwi2c_stop ( struct dwi2c *dwi2c ) {

	/* Disable I2C transfers */
	writel ( 0, ( dwi2c->regs + DWI2C_ENA ) );
}

/**
 * Read data from I2C device
 *
 * @v i2c		I2C controller
 * @v i2cdev		I2C device
 * @v offset		Starting offset within the device
 * @v data		Data buffer
 * @v len		Length of data buffer
 * @ret rc		Return status code
 */
static int dwi2c_read ( struct i2c_bus *i2c, struct i2c_device *i2cdev,
			unsigned int offset, void *data, size_t len ) {
	struct dwi2c *dwi2c = i2c->priv;
	uint8_t *bytes = data;
	int byte;
	int rc;

	/* Start transfer */
	if ( ( rc = dwi2c_start ( dwi2c, i2cdev, offset ) ) != 0 )
		return rc;

	/* Read data */
	while ( len-- ) {
		byte = dwi2c_read_byte ( dwi2c );
		if ( byte < 0 ) {
			rc = byte;
			return rc;
		}
		*(bytes++) = byte;
	}

	/* Stop transfer */
	dwi2c_stop ( dwi2c );

	return 0;
}

/**
 * Write data to I2C device
 *
 * @v i2c		I2C controller
 * @v i2cdev		I2C device
 * @v offset		Starting offset within the device
 * @v data		Data buffer
 * @v len		Length of data buffer
 * @ret rc		Return status code
 */
static int dwi2c_write ( struct i2c_bus *i2c, struct i2c_device *i2cdev,
			 unsigned int offset, const void *data, size_t len ) {
	struct dwi2c *dwi2c = i2c->priv;
	const uint8_t *bytes = data;
	int rc;

	/* Start transfer */
	if ( ( rc = dwi2c_start ( dwi2c, i2cdev, offset ) ) != 0 )
		return rc;

	/* Write data */
	while ( len-- ) {
		if ( ( rc = dwi2c_write_byte ( dwi2c, *(bytes++) ) ) != 0 )
			return rc;
	}

	/* Stop transfer */
	dwi2c_stop ( dwi2c );

	return 0;
}

/** I2C operations */
static struct i2c_operations dwi2c_operations = {
	.read = dwi2c_read,
	.write = dwi2c_write,
};

/**
 * Probe devicetree device
 *
 * @v dt		Devicetree device
 * @v offset		Starting node offset
 * @ret rc		Return status code
 */
static int dwi2c_probe ( struct dt_device *dt, unsigned int offset ) {
	struct i2c_bus *i2c;
	struct dwi2c *dwi2c;
	int rc;

	/* Allocate and initialise device */
	i2c = alloc_i2c ( sizeof ( *dwi2c ) );
	if ( ! i2c ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	dt_set_drvdata ( dt, i2c );
	i2c->dev = &dt->dev;
	i2c_init_bus ( i2c, &dwi2c_operations );
	dwi2c = i2c->priv;
	dwi2c->name = dt->name;

	/* Map registers */
	dwi2c->regs = dt_ioremap ( dt, offset, 0, 0 );
	if ( ! dwi2c->regs ) {
		rc = -ENODEV;
		goto err_ioremap;
	}
	DBGC ( dwi2c, "DWI2C %s is type %#08x version %#08x\n",
	       dwi2c->name, readl ( dwi2c->regs + DWI2C_TYP ),
	       readl ( dwi2c->regs + DWI2C_VER ) );

	/* Configure to operate in controller mode */
	dwi2c_stop ( dwi2c );
	writel ( ( DWI2C_CTL_BUS_CLEAR | DWI2C_CTL_TARGET_DIS |
		   DWI2C_CTL_RESTART_ENA | DWI2C_CTL_SPEED_STD |
		   DWI2C_CTL_CTRLR_ENA ), ( dwi2c->regs + DWI2C_CTL ) );

	/* Register I2C controller */
	if ( ( rc = i2c_register ( i2c ) ) != 0 ) {
		DBGC ( dwi2c, "DWI2C %s could not register: %s\n",
		       dwi2c->name, strerror ( rc ) );
		goto err_register;
	}

	/* Probe child devices */
	if ( ( rc = dt_probe_children ( dt, offset ) ) != 0 )
		goto err_children;

	return 0;

	dt_remove_children ( dt );
 err_children:
	i2c_unregister ( i2c );
 err_register:
	iounmap ( dwi2c->regs );
 err_ioremap:
	i2c_nullify ( i2c );
	i2c_put ( i2c );
 err_alloc:
	return rc;
}

/**
 * Remove devicetree device
 *
 * @v dt		Devicetree device
 */
static void dwi2c_remove ( struct dt_device *dt ) {
	struct i2c_bus *i2c = dt_get_drvdata ( dt );
	struct dwi2c *dwi2c = i2c->priv;

	/* Remove child ports */
	dt_remove_children ( dt );

	/* Unregister I2C controller */
	i2c_unregister ( i2c );

	/* Disable device */
	dwi2c_stop ( dwi2c );

	/* Unmap registers */
	iounmap ( dwi2c->regs );

	/* Free I2C device */
	i2c_nullify ( i2c );
	i2c_put ( i2c );
}

/** DesignWare I2C compatible model identifiers */
static const char * dwi2c_ids[] = {
	"snps,designware-i2c",
};

/** DesignWare I2C devicetree driver */
struct dt_driver dwi2c_driver __dt_driver = {
	.name = "dwi2c",
	.ids = dwi2c_ids,
	.id_count = ( sizeof ( dwi2c_ids ) / sizeof ( dwi2c_ids[0] ) ),
	.probe = dwi2c_probe,
	.remove = dwi2c_remove,
};
