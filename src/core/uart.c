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

#include <unistd.h>
#include <errno.h>
#include <ipxe/uart.h>

/** Timeout for transmit holding register to become empty */
#define UART_THRE_TIMEOUT_MS 100

/** Timeout for transmitter to become empty */
#define UART_TEMT_TIMEOUT_MS 1000

/**
 * Transmit data
 *
 * @v uart		UART
 * @v data		Data
 */
void uart_transmit ( struct uart *uart, uint8_t data ) {
	unsigned int i;
	uint8_t lsr;

	/* Wait for transmitter holding register to become empty */
	for ( i = 0 ; i < UART_THRE_TIMEOUT_MS ; i++ ) {
		lsr = uart_read ( uart, UART_LSR );
		if ( lsr & UART_LSR_THRE )
			break;
		mdelay ( 1 );
	}

	/* Transmit data (even if we timed out) */
	uart_write ( uart, UART_THR, data );
}

/**
 * Flush data
 *
 * @v uart		UART
 */
void uart_flush ( struct uart *uart ) {
	unsigned int i;
	uint8_t lsr;

	/* Wait for transmitter and receiver to become empty */
	for ( i = 0 ; i < UART_TEMT_TIMEOUT_MS ; i++ ) {
		uart_read ( uart, UART_RBR );
		lsr = uart_read ( uart, UART_LSR );
		if ( ( lsr & UART_LSR_TEMT ) && ! ( lsr & UART_LSR_DR ) )
			break;
	}
}

/**
 * Initialise UART
 *
 * @v uart		UART
 * @v baud		Baud rate, or zero to leave unchanged
 * @v lcr		Line control register value, or zero to leave unchanged
 * @ret rc		Return status code
 */
int uart_init ( struct uart *uart, unsigned int baud, uint8_t lcr ) {
	uint16_t divisor;
	uint8_t dlm;
	uint8_t dll;

	/* Check for existence of UART */
	if ( ! uart->base )
		return -ENODEV;
	uart_write ( uart, UART_SCR, 0x18 );
	if ( uart_read ( uart, UART_SCR ) != 0x18 )
		return -ENODEV;
	uart_write ( uart, UART_SCR, 0xae );
	if ( uart_read ( uart, UART_SCR ) != 0xae )
		return -ENODEV;

	/* Configure divisor and line control register, if applicable */
	if ( ! lcr )
		lcr = uart_read ( uart, UART_LCR );
	if ( baud ) {
		uart_write ( uart, UART_LCR, ( lcr | UART_LCR_DLAB ) );
		divisor = ( UART_MAX_BAUD / baud );
		dlm = ( ( divisor >> 8 ) & 0xff );
		dll = ( ( divisor >> 0 ) & 0xff );
		uart_write ( uart, UART_DLM, dlm );
		uart_write ( uart, UART_DLL, dll );
	}
	uart_write ( uart, UART_LCR, ( lcr & ~UART_LCR_DLAB ) );

	/* Disable interrupts */
	uart_write ( uart, UART_IER, 0 );

	/* Enable FIFOs */
	uart_write ( uart, UART_FCR, UART_FCR_FE );

	/* Assert DTR and RTS */
	uart_write ( uart, UART_MCR, ( UART_MCR_DTR | UART_MCR_RTS ) );

	/* Flush any stale data */
	uart_flush ( uart );

	return 0;
}
