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
 * 16550-compatible UART
 *
 */

#include <errno.h>
#include <ipxe/uart.h>
#include <config/serial.h>

/** UART port bases */
static uint16_t uart_base[] = { COM1, COM2, COM3, COM4 };

/**
 * Select UART port
 *
 * @v uart		UART
 * @v port		Port number
 * @ret rc		Return status code
 */
int uart_select ( struct uart *uart, unsigned int port ) {

	if ( port < ( sizeof ( uart_base ) / sizeof ( uart_base[0] ) ) ) {
		uart->base = ( ( void * ) ( intptr_t ) uart_base[port] );
		return 0;
	} else {
		return -ENODEV;
	}
}
