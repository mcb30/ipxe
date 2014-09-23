/*
 * Copyright (C) 2008 Stefan Hajnoczi <stefanha@gmail.com>.
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

#include <stddef.h>
#include <assert.h>
#include <ipxe/uart.h>
#include <ipxe/gdbstub.h>
#include <ipxe/gdbserial.h>
#include <config/serial.h>

/* UART base address */
#ifdef COMCONSOLE
#define GDBSERIAL_BASE ( ( void * ) COMCONSOLE )
#else
#define GDBSERIAL_BASE NULL
#endif

/* UART baud rate */
#ifdef COMPRESERVE
#define GDBSERIAL_BAUD 0
#else
#define GDBSERIAL_BAUD COMSPEED
#endif

/* UART line control register value */
#ifdef COMPRESERVE
#define GDBSERIAL_LCR 0
#else
#define GDBSERIAL_LCR UART_LCR_WPS ( COMDATA, COMPARITY, COMSTOP )
#endif

/** GDB serial UART */
static struct uart gdbserial_uart = {
	.base = GDBSERIAL_BASE,
};

static size_t gdbserial_recv ( char *buf, size_t len ) {

	assert ( len > 0 );
	while ( ! uart_data_ready ( &gdbserial_uart ) ) {}
	buf[0] = uart_receive ( &gdbserial_uart );
	return 1;
}

static void gdbserial_send ( const char *buf, size_t len ) {

	while ( len-- > 0 ) {
		uart_transmit ( &gdbserial_uart, *buf++ );
	}
}

static int gdbserial_init ( int argc __unused, char **argv __unused ) {

	return uart_init ( &gdbserial_uart, GDBSERIAL_BAUD, GDBSERIAL_LCR );
}

struct gdb_transport serial_gdb_transport __gdb_transport = {
	.name = "serial",
	.init = gdbserial_init,
	.recv = gdbserial_recv,
	.send = gdbserial_send,
};
