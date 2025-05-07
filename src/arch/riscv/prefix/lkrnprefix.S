/*
 * Copyright (C) 2025 Michael Brown <mbrown@fensystems.co.uk>.
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
 *
 * You can also choose to distribute this program under the terms of
 * the Unmodified Binary Distribution Licence (as given in the file
 * COPYING.UBDL), provided that you have satisfied its requirements.
 */

	FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL )

/** @file
 *
 * Linux kernel prefix
 *
 */

	.section ".note.GNU-stack", "", @progbits
	.text

/* ELF machine type */
#define EM_RISCV 243

	/*
	 * Display progress message via debug console
	 */
	.macro	progress message
#ifndef NDEBUG
	.section ".rodata.progress_\@", "a", @progbits
progress_\@:
	.asciz	"\message"
	.size	progress_\@, . - progress_\@
	.previous
	la	t1, progress_\@
	call	print_message
#endif
	.endm

	/*
	 * Linux kernel header
	 */
	.section ".prefix", "ax", @progbits
	.org 0
	/* Executable code / MZ header */
	j	_lkrn_start
	.org 8
	/* Image load offset
	 *
	 * Must be set to 0x200000 for 64-bit, 0x400000 for 32-bit
	 */
	.dword	( 0x400000 / ( __riscv_xlen / 32 ) )
	/** Image size */
	.dword	_memsz
	/** Flags */
	.dword	0
	/** Version */
	.word	( ( 0 << 16 ) | ( 2 << 0 ) )
	/** Reserved */
	.word	0
	.dword	0
	/** Magic numbers */
	.ascii	"RISCV\0\0\0"
	.ascii	"RSC\x05"
	/** Reserved / PE offset */
	.word	0
	.org	64

	/*
	 * Linux kernel entry point
	 */
	.globl	_lkrn_start
_lkrn_start:
	/* Preserve arguments */
	mv	s0, a0
	mv	s1, a1
	progress "\nLKRN->iPXE"

	/* Enable paging to map the iPXE image at its current address
	 *
	 * Linux expects to be placed at the image load offset from
	 * the start of RAM.  Assume that we can therefore use the
	 * (not yet initialised) memory immediately after our own
	 * loaded image for the temporary page table.
	 */
	la	a0, _end
	li	t0, 4095
	add	a0, a0, t0
	xori	t0, t0, -1
	and	a0, a0, t0
	call	enable_paging

	/* Install iPXE
	 *
	 * As above, assume that we can use the (not yet initialised)
	 * memory immediately after our own loaded image for the
	 * temporary stack.
	 */
	la	sp, _estack
	mv	a0, s0
	mv	a1, s1
	// virt_offset
	li	a2, 0
	call	install

	/* Call main program */
	call	main

	/* We have no return path, since the Linux kernel does not
	 * define that a valid return address exists.
	 *
	 * Attempt a system reset, since there is nothing else we can
	 * viably do at this point.
	 */
	j	reset_system
	.size	_lkrn_start, . - _lkrn_start

	/* File split information for the compressor */
	.section ".zinfo", "a", @progbits
	.ascii	"COPY"
	.word	0
	.word	_filesz
	.word	1
	.ascii	"BASE"
	.word	0
	.dword	_base
	.ascii	"ZREL"
	.word	_reloc_offset
	.word	_reloc_filesz
	.word	EM_RISCV
