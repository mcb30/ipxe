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
 * RISC-V prefix library
 *
 */

	.section ".note.GNU-stack", "", @progbits
	.text

/*****************************************************************************
 *
 * Apply relocation records
 *
 *****************************************************************************
 *
 * Apply relocation records from .rel.dyn to fix up iPXE to run at its
 * current address.
 *
 * This function must run before .bss is zeroed (since the relocation
 * records are overlaid with .bss).  It does not require a valid stack
 * pointer.
 *
 * Parameters: none (address is implicit in the program counter)
 *
 * Returns: none
 *
 */

/* Relative relocation type */
#define R_RISCV_RELATIVE 3

	/* Layout of a relocation record */
	.struct 0
rela_offset:	.space ( __riscv_xlen / 8 )
rela_type:	.space ( __riscv_xlen / 8 )
rela_addend:	.space ( __riscv_xlen / 8 )
rela_len:
	.previous

	.section ".prefix.apply_relocs", "ax", @progbits
	.globl	apply_relocs
apply_relocs:

	/* Get relocation records */
	la	t0, _reloc
	la	t1, _ereloc

	/* Determine current location */
	la	t2, reloc_base
	LOADN	t2, (t2)
	sub	t2, t0, t2

1:	/* Read relocation record */
	LOADN	t3, rela_offset(t0)
	LOADN	t4, rela_type(t0)
	LOADN	t5, rela_addend(t0)

	/* Check relocation type */
	addi	t4, t4, -R_RISCV_RELATIVE
	bnez	t4, 2f

	/* Apply relocation */
	add	t3, t3, t2
	add	t5, t5, t2
	STOREN	t5, (t3)
2:
	/* Loop until done */
	addi	t0, t0, rela_len
	blt	t0, t1, 1b

	/* Return to caller */
	ret
	.size	apply_relocs, . - apply_relocs

	/* Link-time address of _reloc */
	.section ".rodata", "a", @progbits
reloc_base:
	.dword	_reloc_base
	.size	reloc_base, . - reloc_base

/*****************************************************************************
 *
 * Set up paging
 *
 *****************************************************************************
 *
 * This function must be called with flat physical addressing.  It
 * does not require a valid stack pointer.
 *
 * Parameters:
 *
 *   a0 - Page table to fill in (4kB, must be aligned to a 4kB boundary)
 *
 * Returns:
 *
 *   a0 - Paging level enabled (0=no paging)
 *
 */

#define PTE_V		0x00000001	/**< Page table entry is valid */
#define PTE_R		0x00000002	/**< Page is readable */
#define PTE_W		0x00000004	/**< Page is writable */
#define PTE_X		0x00000008	/**< Page is executable */
#define PTE_A		0x00000040	/**< Page has been accessed */
#define PTE_D		0x00000080	/**< Page is dirty */

	.globl	setup_paging
	.equ	setup_paging, _C2 ( setup_paging_, __riscv_xlen )

/*****************************************************************************
 *
 * Set up 64-bit paging
 *
 *****************************************************************************
 *
 * Construct a 64-bit page table to identity-map the whole of the
 * mappable physical address space, and to map iPXE itself at its
 * link-time address (which must be 2MB-aligned and be within the
 * upper half of the kernel address space).
 *
 * This function must be called with flat physical addressing.  It
 * does not require a valid stack pointer.
 *
 * Parameters:
 *
 *   a0 - Page table to fill in (4kB, must be aligned to a 4kB boundary)
 *
 * Returns:
 *
 *   a0 - Paging is enabled (0=no paging)
 *
 * A 4kB 64-bit page table contains 512 8-byte PTEs.  We choose to use
 * these as:
 *
 *    - PTE[0-255] : Identity map for the physical address space.
 *
 *      This conveniently requires exactly 256 PTEs, regardless of the
 *      paging level.  Higher paging levels are able to identity-map a
 *      larger physical address space:
 *
 *      Sv57 : 256 x 256TB "petapages" (55-bit physical address space)
 *      Sv48 : 256 x 512GB "terapages" (46-bit physical address space)
 *      Sv39 : 256 x   1GB "gigapages" (37-bit physical address space)
 *
 *      Note that Sv48 and Sv39 cannot identity-map the whole of the
 *      available physical address space, since the virtual address
 *      space is not large enough (and is halved by the constraint
 *      that virtual addresses with bit 47/38 set must also have all
 *      higher bits set, and so cannot identity-map to a 55-bit
 *      physical address).
 *
 *    - PTE[344-510 (max)] : Virtual address map for iPXE
 *
 *      These are 2MB "megapages" used to map the link-time virtual
 *      address range used by iPXE itself.  We choose a link-time
 *      starting address of 0xffffffffeb000000, which breaks down as:
 *
 *         VPN[4] = 511 (in Sv57)
 *         VPN[3] = 511 (in Sv57 and Sv48)
 *         VPN[2] = 511 (in Sv57, Sv48, and Sv39)
 *         VPN[1] = 344
 *         VPN[0] = 0
 *
 *      In most builds, only a single 2MB "megapage" will be needed.
 *
 *    - PTE[511] : Recursive next level page table pointer
 *
 *      This is a non-leaf PTE that points back to the page table
 *      itself.  It acts as the next level page table pointer for:
 *
 *         VPN[4] = 511 (in Sv57)
 *         VPN[3] = 511 (in Sv57 and Sv48)
 *         VPN[2] = 511 (in Sv57, Sv48, and Sv39)
 *
 *      This recursive usage creates some duplicate mappings within
 *      unused portions of the virtual address space, but allows us to
 *      use only a single 4kB page table.
 */

#define PG64_SV57	10	/**< Five-level paging (Sv57) */
#define PG64_SV48	9	/**< Four-level paging (Sv48) */
#define PG64_SV39	8	/**< Three-level paging (Sv39) */

	.section ".prefix.setup_paging_64", "ax", @progbits
setup_paging_64:
	/* Register usage:
	 *
	 * t0 - currently attempted paging level
	 *
	 *
	 */

	/* t0 := SvXX = attempted paging level (starting with Sv57) */
	li	t0, PG64_SV57
1:
	/* t1 := PTE stride for identity map */
	slli	t1, t0, 3	/* t1 := ( 9 * SvXX - 44 ) */
	add	t1, t0, t1
	addi	t1, t1, -44
	li	t2, 1		/* t1 := ( 1 << ( 9 * SvXX - 44 ) ) */
	sll	t1, t2, t1	/*     = PTE stride at level SvXX   */

	/* Construct PTE[0-255] for identity map */
	li	t2, ( PTE_D | PTE_A | PTE_X | PTE_W | PTE_R | PTE_V )
	mv	t3, a0
	li	t4, 256
2:	STOREN	t2, (t3)
	add	t2, t2, t1
	addi	t3, t3, 8
	addi	t4, t4, -1
	bgtz	t4, 2b

	/* Zero remaining PTEs */
	li	t4, 256
3:	STOREN	zero, (t3)
	addi	t3, t3, 8
	addi	t4, t4, -1
	bgtz	t4, 3b

	/* Construct PTE[511] as next level page table pointer */
	srli	t1, a0, 2
	ori	t1, t1, ( PTE_D | PTE_A | PTE_V )
	STOREN	t1, -8(t3)

	/* Construct PTE[344+] to cover iPXE itself */
	addi	t3, t3, -( ( 512 - 344 ) * 8 )
	la	t1, _prefix
	srli	t1, t1, 2
	ori	t1, t1, ( PTE_D | PTE_A | PTE_X | PTE_W | PTE_R | PTE_V )
	la	t2, _ebss
	srli	t2, t2, 2
	li	t4, 1
	slli	t4, t4, 19
4:	STOREN	t1, (t3)
	addi	t3, t3, 8
	add	t1, t1, t4
	ble	t1, t2, 4b

	/* Construct SATP */
	slli	t1, t0, 60
	srli	t2, a0, 12
	or	t1, t1, t2
	csrrw	zero, satp, t1
	sfence.vma

	/* Read back SATP to see if this paging level was supported */
	csrrw	t1, satp, t1
	srli	t1, t1, 60
	beq	t0, t1, 99f

	/* Try next paging level */
	addi	t0, t0, -1
	li	t2, PG64_SV39
	bge	t0, t2, 1b

99:	/* Return enabled paging level (if any) */
	mv	a0, t1
	ret
	.size	setup_paging_64, . - setup_paging_64
