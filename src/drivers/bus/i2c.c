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
#include <errno.h>
#include <ipxe/i2c.h>

/** @file
 *
 * I2C buses
 *
 */

/** List of I2C buses */
static LIST_HEAD ( i2c_buses );

/**
 * Allocate I2C bus
 *
 * @v priv_len		Size of driver-private data
 * @ret i2c		I2C bus, or NULL
 */
struct i2c_bus * alloc_i2c ( size_t priv_len ) {
	struct i2c_bus *i2c;

	/* Allocate and initialise structure */
	i2c = zalloc ( sizeof ( *i2c ) + priv_len );
	if ( ! i2c )
		return NULL;
	i2c->priv = ( ( ( void * ) i2c ) + sizeof ( *i2c ) );

	return i2c;
}

/**
 * Register I2C bus
 *
 * @v i2c		I2C bus
 * @ret rc		Return status code
 */
int i2c_register ( struct i2c_bus *i2c ) {

	/* Add to list of I2C buses */
	i2c_get ( i2c );
	list_add_tail ( &i2c->list, &i2c_buses );
	DBGC ( i2c, "I2C %s registered\n", i2c->dev->name );

	return 0;
}

/**
 * Unregister I2C bus
 *
 * @v i2c		I2C bus
 */
void i2c_unregister ( struct i2c_bus *i2c ) {

	/* Remove from list of I2C buses */
	DBGC ( i2c, "I2C %s unregistered", i2c->dev->name );
	list_del ( &i2c->list );
	i2c_put ( i2c );
}

/**
 * Read data from null I2C device
 *
 * @v i2c		I2C bus
 * @v i2cdev		I2C device
 * @v offset		Starting offset within the device
 * @v data		Data buffer
 * @v len		Length of data buffer
 * @ret rc		Return status code
 */
static int null_i2c_read ( struct i2c_bus *i2c __unused,
			   struct i2c_device *i2cdev __unused,
			   unsigned int offset __unused,
			   void *data __unused, size_t len __unused ) {

	return -ENODEV;
}

/**
 * Write data to null I2C device
 *
 * @v i2c		I2C bus
 * @v i2cdev		I2C device
 * @v offset		Starting offset within the device
 * @v data		Data buffer
 * @v len		Length of data buffer
 * @ret rc		Return status code
 */
static int null_i2c_write ( struct i2c_bus *i2c __unused,
			    struct i2c_device *i2cdev __unused,
			    unsigned int offset __unused,
			    const void *data __unused, size_t len __unused ) {

	return -ENODEV;
}

/** Null I2C operations */
struct i2c_operations null_i2c_operations = {
	.read = null_i2c_read,
	.write = null_i2c_write,
};
