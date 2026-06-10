#ifndef _BITS_BYTESWAP_H
#define _BITS_BYTESWAP_H

/** @file
 *
 * Byte-order swapping functions
 *
 */

#include <stdint.h>

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );
FILE_SECBOOT ( PERMITTED );

static inline __attribute__ (( always_inline, const )) uint16_t
__bswap_variable_16 ( uint16_t x ) {
	uint32_t swapped;

	__asm__ ( "lrvr %0, %1" : "=r" ( swapped ) : "r" ( x ) );
	return ( swapped >> 16 );
}

static inline __attribute__ (( always_inline )) void
__bswap_16s ( uint16_t *x ) {
	uint32_t swapped;

	__asm__ ( "lrvr %0, %1" : "=r" ( swapped ) : "r" ( x ) );
	*x = ( swapped >> 16 );
}

static inline __attribute__ (( always_inline, const )) uint32_t
__bswap_variable_32 ( uint32_t x ) {
	uint32_t swapped;

	__asm__ ( "lrvr %0, %1" : "=r" ( swapped ) : "r" ( x ) );
	return swapped;
}

static inline __attribute__ (( always_inline )) void
__bswap_32s ( uint32_t *x ) {
	uint32_t swapped;

	__asm__ ( "lrvr %0, %1" : "=r" ( swapped ) : "r" ( x ) );
	*x = swapped;
}

static inline __attribute__ (( always_inline, const )) uint64_t
__bswap_variable_64 ( uint64_t x ) {
	uint64_t swapped;

	__asm__ ( "lrvgr %0, %1" : "=r" ( swapped ) : "r" ( x ) );
	return swapped;
}

static inline __attribute__ (( always_inline )) void
__bswap_64s ( uint64_t *x ) {
	uint64_t swapped;

	__asm__ ( "lrvgr %0, %1" : "=r" ( swapped ) : "r" ( x ) );
	*x = swapped;
}

#endif /* _BITS_BYTESWAP_H */
