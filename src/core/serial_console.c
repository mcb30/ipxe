/*
 * Copyright (C) 2014 Michael Brown <mbrown@fensystems.co.uk>.
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

FILE_LICENCE ( GPL2_OR_LATER );

/** @file
 *
 * Serial console
 *
 */

#include <stddef.h>
#include <ipxe/init.h>
#include <ipxe/uart.h>
#include <ipxe/console.h>
#include <config/console.h>
#include <config/serial.h>

/* Set default console usage if applicable */
#if ! ( defined ( CONSOLE_SERIAL ) && CONSOLE_EXPLICIT ( CONSOLE_SERIAL ) )
#undef CONSOLE_SERIAL
#define CONSOLE_SERIAL ( CONSOLE_USAGE_ALL & ~CONSOLE_USAGE_LOG )
#endif

/* UART base address */
#ifdef COMCONSOLE
#define CONSOLE_BASE ( ( void * ) COMCONSOLE )
#else
#define CONSOLE_BASE NULL
#endif

/* UART baud rate */
#ifdef COMPRESERVE
#define CONSOLE_BAUD 0
#else
#define CONSOLE_BAUD COMSPEED
#endif

/* UART line control register value */
#ifdef COMPRESERVE
#define CONSOLE_LCR 0
#else
#define CONSOLE_LCR UART_LCR_WPS ( COMDATA, COMPARITY, COMSTOP )
#endif

struct console_driver serial_console __console_driver;

/** Serial console UART */
static struct uart console_uart = {
	.base = CONSOLE_BASE,
};

/**
 * Print a character to serial console
 *
 * @v character		Character to be printed
 */
static void serial_putchar ( int character ) {

	uart_transmit ( &console_uart, character );
}

/**
 * Get character from serial console
 *
 * @ret character	Character read from console
 */
static int serial_getchar ( void ) {
	uint8_t data;

	/* Wait for data to be ready */
	while ( ! uart_data_ready ( &console_uart ) ) {}

	/* Receive data */
	data = uart_receive ( &console_uart );

	/* Strip any high bit and convert DEL to backspace */
	data &= 0x7f;
	if ( data == 0x7f )
		data = 0x08;

	return data;
}

/**
 * Check for character ready to read from serial console
 *
 * @ret True		Character available to read
 * @ret False		No character available to read
 */
static int serial_iskey ( void ) {

	return uart_data_ready ( &console_uart );
}

/**
 * Configure serial console
 *
 * @v config		Console configuration, or NULL to leave unchanged
 * @ret rc		Return status code
 */
static int serial_configure ( struct console_configuration *config ) {
	int rc;

	/* Do nothing unless we have a serial configuration to apply */
	if ( ( ! config ) || ( ! config->port ) )
		return 0;

	/* Flush and disable console */
	if ( ! serial_console.disabled )
		uart_flush ( &console_uart );
	serial_console.disabled = CONSOLE_DISABLED;

	/* Select UART */
	if ( ( rc = uart_select ( &console_uart, config->port ) ) != 0 ) {
		DBG ( "Could not select UART port %d: %s\n",
		      config->port, strerror ( rc ) );
		return rc;
	}

	/* Initialise UART */
	if ( ( rc = uart_init ( &console_uart, config->speed,
				config->lcr ) ) != 0 ) {
		DBG ( "Could not initialise serial console: %s\n",
		      strerror ( rc ) );
		return rc;
	}

	/* Enable console */
	serial_console.disabled = 0;

	return 0;
}

/** Serial console */
struct console_driver serial_console __console_driver = {
	.putchar = serial_putchar,
	.getchar = serial_getchar,
	.iskey = serial_iskey,
	.configure = serial_configure,
	.disabled = CONSOLE_DISABLED,
	.usage = CONSOLE_SERIAL,
};

/** Initialise serial console */
static void serial_init ( void ) {
	int rc;

	/* Do nothing if we have no default port */
	if ( ! CONSOLE_BASE )
		return;

	/* Initialise UART */
	if ( ( rc = uart_init ( &console_uart, CONSOLE_BAUD,
				CONSOLE_LCR ) ) != 0 ) {
		DBG ( "Could not initialise serial console: %s\n",
		      strerror ( rc ) );
		return;
	}

	/* Enable console */
	serial_console.disabled = 0;
}

/**
 * Shut down serial console
 *
 * @v flags		Shutdown flags
 */
static void serial_shutdown ( int flags __unused ) {

	/* Flush output */
	if ( ! serial_console.disabled )
		uart_flush ( &console_uart );

	/* Do not mark console as disabled; it's still usable */
}

/** Serial console initialisation function */
struct init_fn serial_console_init_fn __init_fn ( INIT_CONSOLE ) = {
	.initialise = serial_init,
};

/** Serial console startup function */
struct startup_fn serial_startup_fn __startup_fn ( STARTUP_EARLY ) = {
	.shutdown = serial_shutdown,
};
