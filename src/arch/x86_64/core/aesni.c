/*
 * Copyright (C) 2026 Michael Brown <mbrown@fensystems.co.uk>.
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

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );
FILE_SECBOOT ( PERMITTED );

/** @file
 *
 * AES-NI hardware acceleration
 *
 */

#include <assert.h>
#include <ipxe/cpuid.h>
#include <ipxe/aes.h>

/**
 * Encrypt data
 *
 * @v cipher		Cipher algorithm
 * @v ctx		Context
 * @v src		Data to encrypt
 * @v dst		Buffer for encrypted data
 * @v len		Length of data
 */
static void aesni_encrypt ( struct cipher_algorithm *cipher __unused,
			    void *ctx, const void *src, void *dst,
			    size_t len ) {
	struct aes_context *aes = aes_context ( ctx );
	const union aes_matrix *key = aes->encrypt.key;
	const union aes_matrix *in = src;
	union aes_matrix *out = dst;
	union aes_matrix tmp;

	/* Sanity check */
	assert ( len == sizeof ( *in ) );
	assert ( len == sizeof ( *out ) );

	/* Encrypt */
	asm ( /* Initial round (AddRoundKey) */
	      "movdqu %3, %0\n\t"
	      "pxor (%1), %0\n\t"
	      /* Intermediate rounds (ShiftRows, SubBytes, MixColumns,
	       * AddRoundKey).
	       */
	      "cmpb $13, %4\n\t"
	      "jb 2f\n\t"
	      "je 1f\n\t"
	      /* 15 rounds (13 intermediate rounds) */
	      "aesenc 0x10(%1), %0\n\t"
	      "aesenc 0x20(%1), %0\n\t"
	      "lea 0x20(%1), %1\n\t"
	      "\n1:\n\t"
	      /* 13+ rounds (11+ intermediate rounds) */
	      "aesenc 0x10(%1), %0\n\t"
	      "aesenc 0x20(%1), %0\n\t"
	      "lea 0x20(%1), %1\n\t"
	      "\n2:\n\t"
	      /* 11+ rounds (9+ intermediate rounds) */
	      "aesenc 0x10(%1), %0\n\t"
	      "aesenc 0x20(%1), %0\n\t"
	      "aesenc 0x30(%1), %0\n\t"
	      "aesenc 0x40(%1), %0\n\t"
	      "aesenc 0x50(%1), %0\n\t"
	      "aesenc 0x60(%1), %0\n\t"
	      "aesenc 0x70(%1), %0\n\t"
	      "aesenc 0x80(%1), %0\n\t"
	      "aesenc 0x90(%1), %0\n\t"
	      /* Final round (ShiftRows, SubBytes, AddRoundKey) */
	      "aesenclast 0xa0(%1), %0\n\t"
	      "movdqu %0, %2\n\t"
	      : "=&x" ( tmp ), "+r" ( key ), "=m" ( *out )
	      : "m" ( *in ), "g" ( aes->rounds ), "m" ( aes->encrypt ) );
}

/**
 * Decrypt data
 *
 * @v cipher		Cipher algorithm
 * @v ctx		Context
 * @v src		Data to encrypt
 * @v dst		Buffer for encrypted data
 * @v len		Length of data
 */
static void aesni_decrypt ( struct cipher_algorithm *cipher __unused,
			    void *ctx, const void *src, void *dst,
			    size_t len ) {
	struct aes_context *aes = aes_context ( ctx );
	const union aes_matrix *key = aes->decrypt.key;
	const union aes_matrix *in = src;
	union aes_matrix *out = dst;
	union aes_matrix tmp;

	/* Sanity check */
	assert ( len == sizeof ( *in ) );
	assert ( len == sizeof ( *out ) );

	/* Decrypt */
	asm ( /* Initial round (AddRoundKey) */
	      "movdqu %3, %0\n\t"
	      "pxor (%1), %0\n\t"
	      /* Intermediate rounds (InvShiftRows, InvSubBytes,
	       * InvMixColumns, AddRoundKey).
	       */
	      "cmpb $13, %4\n\t"
	      "jb 2f\n\t"
	      "je 1f\n\t"
	      /* 15 rounds (13 intermediate rounds) */
	      "aesdec 0x10(%1), %0\n\t"
	      "aesdec 0x20(%1), %0\n\t"
	      "lea 0x20(%1), %1\n\t"
	      "\n1:\n\t"
	      /* 13+ rounds (11+ intermediate rounds) */
	      "aesdec 0x10(%1), %0\n\t"
	      "aesdec 0x20(%1), %0\n\t"
	      "lea 0x20(%1), %1\n\t"
	      "\n2:\n\t"
	      /* 11+ rounds (9+ intermediate rounds) */
	      "aesdec 0x10(%1), %0\n\t"
	      "aesdec 0x20(%1), %0\n\t"
	      "aesdec 0x30(%1), %0\n\t"
	      "aesdec 0x40(%1), %0\n\t"
	      "aesdec 0x50(%1), %0\n\t"
	      "aesdec 0x60(%1), %0\n\t"
	      "aesdec 0x70(%1), %0\n\t"
	      "aesdec 0x80(%1), %0\n\t"
	      "aesdec 0x90(%1), %0\n\t"
	      /* Final round (InvShiftRows, InvSubBytes, AddRoundKey) */
	      "aesdeclast 0xa0(%1), %0\n\t"
	      "movdqu %0, %2\n\t"
	      : "=&x" ( tmp ), "+r" ( key ), "=m" ( *out )
	      : "m" ( *in ), "g" ( aes->rounds ), "m" ( aes->decrypt ) );
}

/**
 * Enable hardware AES acceleration (if supported)
 *
 */
void aesni_detect ( void ) {
	struct x86_features features;

	/* Detect AES-NI support */
	x86_features ( &features );
	if ( features.intel.ecx & CPUID_FEATURES_INTEL_ECX_AESNI ) {
		DBGC ( &aes_algorithm, "AES using AES-NI acceleration\n" );
		aes_algorithm.encrypt = aesni_encrypt;
		aes_algorithm.decrypt = aesni_decrypt;
	}
}
