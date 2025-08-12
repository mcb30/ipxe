#ifndef _IPXE_I2C_H
#define _IPXE_I2C_H

/** @file
 *
 * I2C interface
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

#include <stdint.h>
#include <ipxe/bitbash.h>

/** An I2C device
 *
 * An I2C device represents a specific slave device on an I2C bus.
 */
struct i2c_device {
	/** I2C bus */
	struct i2c_bus *i2c;
	/** Address of this device
	 *
	 * The actual address sent on the bus will look like
	 *
	 *    <start> <device address> <word address overflow> <r/w>
	 *
	 * The "word address overflow" is any excess bits from the
	 * word address, i.e. any portion that does not fit within the
	 * defined word address length.
	 */
	unsigned int dev_addr;
	/** Word adddress length, in bytes
	 *
	 * This is the number of bytes that comprise the word address,
	 * defined to be the portion that starts after the read/write
	 * bit and ends before the first data byte.
	 *
	 * For some devices, this length will be zero (i.e. the word
	 * address is contained entirely within the "word address
	 * overflow").
	 */
	unsigned int word_addr_len;
};

/** I2C bus operations */
struct i2c_operations {
	/**
	 * Read data from I2C device
	 *
	 * @v i2c		I2C bus
	 * @v i2cdev		I2C device
	 * @v offset		Starting offset within the device
	 * @v data		Data buffer
	 * @v len		Length of data buffer
	 * @ret rc		Return status code
	 */
	int ( * read ) ( struct i2c_bus *i2c, struct i2c_device *i2cdev,
			 unsigned int offset, void *data, size_t len );
	/**
	 * Write data to I2C device
	 *
	 * @v i2c		I2C bus
	 * @v i2cdev		I2C device
	 * @v offset		Starting offset within the device
	 * @v data		Data buffer
	 * @v len		Length of data buffer
	 * @ret rc		Return status code
	 */
	int ( * write ) ( struct i2c_bus *i2c, struct i2c_device *i2cdev,
			  unsigned int offset, const void *data, size_t len );
};

/** An I2C bus */
struct i2c_bus {
	/** Bus operations */
	struct i2c_operations *op;
};

/** A bit-bashing I2C bus
 *
 * This provides a standardised way to construct I2C buses via a
 * bit-bashing interface.
 */
struct i2c_bit_basher {
	/** I2C bus */
	struct i2c_bus i2c;
	/** Bit-bashing interface */
	struct bit_basher basher;
};

/** Ten-bit address marker
 *
 * This value is ORed with the I2C device address to indicate a
 * ten-bit address format on the bus.
 */
#define I2C_TENBIT_ADDRESS 0x7800

/** An I2C write command */
#define I2C_WRITE 0

/** An I2C read command */
#define I2C_READ 1

/** Bit indices used for I2C bit-bashing interface */
enum {
	/** Serial clock */
	I2C_BIT_SCL = 0,
	/** Serial data */
	I2C_BIT_SDA,
};

/** Delay required for bit-bashing operation */
#define I2C_UDELAY 5

/** Maximum number of cycles to use when attempting a bus reset */
#define I2C_RESET_MAX_CYCLES 32

/**
 * Read data from I2C device
 *
 * @v i2cdev		I2C device
 * @v offset		Starting offset within the device
 * @v data		Data buffer
 * @v len		Length of data buffer
 * @ret rc		Return status code
 */
static inline __attribute__ (( always_inline )) int
i2c_read ( struct i2c_device *i2cdev, unsigned int offset, void *data,
	   size_t len ) {
	struct i2c_bus *i2c = i2cdev->i2c;

	return i2c->op->read ( i2c, i2cdev, offset, data, len );
}

/**
 * Write data to I2C device
 *
 * @v i2cdev		I2C device
 * @v offset		Starting offset within the device
 * @v data		Data buffer
 * @v len		Length of data buffer
 * @ret rc		Return status code
 */
static inline __attribute__ (( always_inline )) int
i2c_write ( struct i2c_device *i2cdev, unsigned int offset, const void *data,
	    size_t len ) {
	struct i2c_bus *i2c = i2cdev->i2c;

	return i2c->op->write ( i2c, i2cdev, offset, data, len );
}

/**
 * Check presence of I2C device
 *
 * @v i2cdev		I2C device
 * @ret rc		Return status code
 *
 * Checks for the presence of the device on the I2C bus by attempting
 * a zero-length write.
 */
static inline __attribute__ (( always_inline )) int
i2c_check_presence ( struct i2c_device *i2cdev ) {

	return i2c_write ( i2cdev, 0, NULL, 0 );
}

/**
 * Calculate device address (including any word address overflow)
 *
 * @v i2cdev		I2C device
 * @v offset		Starting offset within the device
 * @ret address		Device address (including any word address overflow)
 */
static inline unsigned int i2c_address ( struct i2c_device *i2cdev,
					 unsigned int offset ) {

	return ( i2cdev->dev_addr |
		 ( offset >> ( 8 * i2cdev->word_addr_len ) ) );
}

/**
 * Initialise I2C bus
 *
 * @v i2c		I2C bus
 * @v op		Bus operations
 */
static inline __attribute__ (( always_inline )) void
i2c_init_bus ( struct i2c_bus *i2c, struct i2c_operations *op ) {

	i2c->op = op;
}

/**
 * Initialise generic I2C device
 *
 * @v i2cdev		I2C device
 * @v i2c		I2C bus
 * @v dev_addr		Device address
 * @v word_addr_len	Word address length
 */
static inline __attribute__ (( always_inline )) void
i2c_init_device ( struct i2c_device *i2cdev, struct i2c_bus *i2c,
		  unsigned int dev_addr, unsigned int word_addr_len ) {

	i2cdev->i2c = i2c;
	i2cdev->dev_addr = dev_addr;
	i2cdev->word_addr_len = word_addr_len;
}

/**
 * Initialise generic single-byte addressed I2C device
 *
 * @v i2cdev		I2C device
 * @v i2c		I2C bus
 * @v dev_addr		Device address
 * @v word_addr_len	Word address length
 */
static inline __attribute__ (( always_inline )) void
i2c_init_single ( struct i2c_device *i2cdev, struct i2c_bus *i2c,
		  unsigned int dev_addr ) {

	i2c_init_device ( i2cdev, i2c, dev_addr, 1 );
}

/**
 * Initialise solo I2C device (e.g. Atmel AT24C11)
 *
 * @v i2cdev		I2C device
 * @v i2c		I2C bus
 */
static inline __attribute__ (( always_inline )) void
i2c_init_solo ( struct i2c_device *i2cdev, struct i2c_bus *i2c ) {

	/* This chip has no device address; it must be the only chip
	 * on the bus.  The word address is contained entirely within
	 * the device address field.
	 */
	i2c_init_device ( i2cdev, i2c, 0, 0 );
}

extern int i2c_bit_init ( struct i2c_bit_basher *i2cbit,
			  struct bit_basher_operations *bash_op );

#endif /* _IPXE_I2C_H */
