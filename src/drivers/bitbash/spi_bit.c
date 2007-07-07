/*
 * Copyright (C) 2006 Michael Brown <mbrown@fensystems.co.uk>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <byteswap.h>
#include <errno.h>
#include <assert.h>
#include <timer.h>
#include <gpxe/bitbash.h>
#include <gpxe/spi_bit.h>

/** @file
 *
 * SPI bit-bashing interface
 *
 */

/** Delay between SCLK changes and around SS changes */
static void spi_bit_delay ( void ) {
	udelay ( SPI_BIT_UDELAY );
}

/** Chip select line will be asserted */
#define SELECT_SLAVE 0

/** Chip select line will be deasserted */
#define DESELECT_SLAVE SPI_MODE_SSPOL

/**
 * Select/deselect slave
 *
 * @v spibit		SPI bit-bashing interface
 * @v slave		Slave number
 * @v state		Slave select state
 *
 * @c state must be @c SELECT_SLAVE or @c DESELECT_SLAVE.
 */
static void spi_bit_set_slave_select ( struct spi_bit_basher *spibit,
				       unsigned int slave,
				       unsigned int state ) {
	struct bit_basher *basher = &spibit->basher;

	state ^= ( spibit->bus.mode & SPI_MODE_SSPOL );
	DBG ( "Setting slave %d select %s\n", slave,
	      ( state ? "high" : "low" ) );

	spi_bit_delay();
	write_bit ( basher, SPI_BIT_SS ( slave ), state );
	spi_bit_delay();
}

/**
 * Transfer bits over SPI bit-bashing bus
 *
 * @v bus		SPI bus
 * @v data_out		TX data buffer (or NULL)
 * @v data_in		RX data buffer (or NULL)
 * @v len		Length of transfer (in @b bits)
 * @v endianness	Endianness of this data transfer
 *
 * This issues @c len clock cycles on the SPI bus, shifting out data
 * from the @c data_out buffer to the MOSI line and shifting in data
 * from the MISO line to the @c data_in buffer.  If @c data_out is
 * NULL, then the data sent will be all zeroes.  If @c data_in is
 * NULL, then the incoming data will be discarded.
 */
static void spi_bit_transfer ( struct spi_bit_basher *spibit,
			       const void *data_out, void *data_in,
			       unsigned int len, int endianness ) {
	struct spi_bus *bus = &spibit->bus;
	struct bit_basher *basher = &spibit->basher;
	unsigned int sclk = ( ( bus->mode & SPI_MODE_CPOL ) ? 1 : 0 );
	unsigned int cpha = ( ( bus->mode & SPI_MODE_CPHA ) ? 1 : 0 );
	unsigned int bit_offset;
	unsigned int byte_offset;
	unsigned int byte_mask;
	unsigned int bit;
	unsigned int step;

	DBG ( "Transferring %d bits in mode %x\n", len, bus->mode );

<<<<<<< HEAD:src/drivers/bitbash/spi_bit.c
	for ( step = 0 ; step < ( len * 2 ) ; step++ ) {
		/* Calculate byte offset and byte mask */
		bit_offset = ( ( endianness == SPI_BIT_BIG_ENDIAN ) ?
			       ( len - ( step / 2 ) - 1 ) : ( step / 2 ) );
		byte_offset = ( bit_offset / 8 );
		byte_mask = ( 1 << ( bit_offset % 8 ) );
=======
>>>>>>> debugging natsemi.c:src/drivers/bitbash/spi_bit.c
<<<<<<< HEAD:src/drivers/bitbash/spi_bit.c
=======
>>>>>>> debugging natsemi.c:src/drivers/bitbash/spi_bit.c
		/* Shift data in or out */
		if ( sclk == cpha ) {
			const uint8_t *byte;

			/* Shift data out */
			if ( data_out ) {
				byte = ( data_out + byte_offset );
				bit = ( *byte & byte_mask );
			} else {
				bit = 0;
			}
			write_bit ( basher, SPI_BIT_MOSI, bit );
		} else {
			uint8_t *byte;

			/* Shift data in */
			bit = read_bit ( basher, SPI_BIT_MISO );
			if ( data_in ) {
				byte = ( data_in + byte_offset );
				*byte &= ~byte_mask;
				*byte |= ( bit & byte_mask );
			}
		}

		/* Toggle clock line */
		spi_bit_delay();
		sclk = ~sclk;
		write_bit ( basher, SPI_BIT_SCLK, sclk );
	}
}

/**
 * Read/write data via SPI bit-bashing bus
 *
 * @v bus		SPI bus
 * @v device		SPI device
 * @v command		Command
 * @v address		Address to read/write (<0 for no address)
 * @v data_out		TX data buffer (or NULL)
 * @v data_in		RX data buffer (or NULL)
 * @v len		Length of transfer
 * @ret rc		Return status code
 */
static int spi_bit_rw ( struct spi_bus *bus, struct spi_device *device,
			unsigned int command, int address,
			const void *data_out, void *data_in, size_t len ) {
	struct spi_bit_basher *spibit
		= container_of ( bus, struct spi_bit_basher, bus );
	uint32_t tmp;

	/* Assert chip select on specified slave */
	spi_bit_set_slave_select ( spibit, device->slave, SELECT_SLAVE );

	/* Transmit command */
	assert ( device->command_len <= ( 8 * sizeof ( tmp ) ) );
	tmp = cpu_to_le32 ( command );
	spi_bit_transfer ( spibit, &tmp, NULL, device->command_len,
			   SPI_BIT_BIG_ENDIAN );

	/* Transmit address, if present */
	if ( address >= 0 ) {
		assert ( device->address_len <= ( 8 * sizeof ( tmp ) ) );
		tmp = cpu_to_le32 ( address );
		spi_bit_transfer ( spibit, &tmp, NULL, device->address_len,
				   SPI_BIT_BIG_ENDIAN );
	}

	/* Transmit/receive data */
	spi_bit_transfer ( spibit, data_out, data_in, ( len * 8 ),
			   spibit->endianness );

	/* Deassert chip select on specified slave */
	spi_bit_set_slave_select ( spibit, device->slave, DESELECT_SLAVE );

	return 0;
}

/**
 * Initialise SPI bit-bashing interface
 *
 * @v spibit		SPI bit-bashing interface
 */
void init_spi_bit_basher ( struct spi_bit_basher *spibit ) {
	assert ( &spibit->basher.op->read != NULL );
	assert ( &spibit->basher.op->write != NULL );
	spibit->bus.rw = spi_bit_rw;
}
