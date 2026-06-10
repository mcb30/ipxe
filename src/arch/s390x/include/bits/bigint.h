#ifndef _BITS_BIGINT_H
#define _BITS_BIGINT_H

/** @file
 *
 * Big integer support
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );
FILE_SECBOOT ( PERMITTED );

#include <stdint.h>
#include <string.h>
#include <strings.h>

/** Element of a big integer */
typedef unsigned long bigint_element_t;

/**
 * Initialise big integer
 *
 * @v value0		Element 0 of big integer to initialise
 * @v size		Number of elements
 * @v data		Raw data
 * @v len		Length of raw data
 */
static inline __attribute__ (( always_inline )) void
bigint_init_raw ( unsigned long *value0, unsigned int size,
		  const void *data, size_t len ) {
	size_t pad_len = ( sizeof ( bigint_t ( size ) ) - len );
	uint8_t *value_byte = ( ( void * ) value0 );
	const uint8_t *data_byte = ( data + len );

	/* Copy raw data in reverse order, padding with zeros */
	while ( len-- )
		*(value_byte++) = *(--data_byte);
	while ( pad_len-- )
		*(value_byte++) = 0;
}

/**
 * Add big integers
 *
 * @v addend0		Element 0 of big integer to add
 * @v value0		Element 0 of big integer to be added to
 * @v size		Number of elements
 * @ret carry		Carry out
 */
static inline __attribute__ (( always_inline )) int
bigint_add_raw ( const unsigned long *addend0, unsigned long *value0,
		 unsigned int size ) {
	bigint_t ( size ) __attribute__ (( may_alias )) *addend =
		( ( void * ) addend0 );
	bigint_t ( size ) __attribute__ (( may_alias )) *value =
		( ( void * ) value0 );
	unsigned long discard_offset;
	unsigned long discard_temp;
	unsigned int index_carry;

	__asm__ ( "xgr %0, %0\n\t" /* zero offset and clear carry-in */
		  "\n1:\n\t"
		  "lg %1, %O3(%0, %R3)\n\t"
		  "alcg %1, %O4(%0, %R4)\n\t"
		  "sg %1, %O3(%0, %R3)\n\t"
		  "la %0, 8(%0)\n\t"
		  "brct %2, 1b\n\t"
		  "alcr %2, %2\n\t" /* carry-out */
		  : "=&a" ( discard_offset ),
		    "=&r" ( discard_temp ),
		    "=&r" ( index_carry ),
		    "+S" ( *value )
		  : "S" ( *addend ),
		    "2" ( size ) );

	return index_carry;
}

/**
 * Subtract big integers
 *
 * @v subtrahend0	Element 0 of big integer to subtract
 * @v value0		Element 0 of big integer to be subtracted from
 * @v size		Number of elements
 * @ret borrow		Borrow out
 */
static inline __attribute__ (( always_inline )) int
bigint_subtract_raw ( const unsigned long *subtrahend0, unsigned long *value0,
		      unsigned int size ) {

	//
	( void ) subtrahend0;
	( void ) value0;
	( void ) size;
	return 0;
}

/**
 * Shift big integer left
 *
 * @v value0		Element 0 of big integer
 * @v size		Number of elements
 * @ret out		Bit shifted out
 */
static inline __attribute__ (( always_inline )) int
bigint_shl_raw ( unsigned long *value0, unsigned int size ) {

	//
	( void ) value0;
	( void ) size;
	return 0;
}

/**
 * Shift big integer right
 *
 * @v value0		Element 0 of big integer
 * @v size		Number of elements
 * @ret out		Bit shifted out
 */
static inline __attribute__ (( always_inline )) int
bigint_shr_raw ( unsigned long *value0, unsigned int size ) {

	//
	( void ) value0;
	( void ) size;
	return 0;
}

/**
 * Test if big integer is equal to zero
 *
 * @v value0		Element 0 of big integer
 * @v size		Number of elements
 * @ret is_zero		Big integer is equal to zero
 */
static inline __attribute__ (( always_inline, pure )) int
bigint_is_zero_raw ( const unsigned long *value0, unsigned int size ) {

	//
	( void ) value0;
	( void ) size;
	return 0;
}

/**
 * Compare big integers
 *
 * @v value0		Element 0 of big integer
 * @v reference0	Element 0 of reference big integer
 * @v size		Number of elements
 * @ret geq		Big integer is greater than or equal to the reference
 */
static inline __attribute__ (( always_inline, pure )) int
bigint_is_geq_raw ( const unsigned long *value0,
		    const unsigned long *reference0, unsigned int size ) {

	//
	( void ) value0;
	( void ) reference0;
	( void ) size;
	return 0;
}

/**
 * Find highest bit set in big integer
 *
 * @v value0		Element 0 of big integer
 * @v size		Number of elements
 * @ret max_bit		Highest bit set + 1 (or 0 if no bits set)
 */
static inline __attribute__ (( always_inline )) int
bigint_max_set_bit_raw ( const unsigned long *value0, unsigned int size ) {

	//
	( void ) value0;
	( void ) size;
	return 0;
}

/**
 * Grow big integer
 *
 * @v source0		Element 0 of source big integer
 * @v source_size	Number of elements in source big integer
 * @v dest0		Element 0 of destination big integer
 * @v dest_size		Number of elements in destination big integer
 */
static inline __attribute__ (( always_inline )) void
bigint_grow_raw ( const unsigned long *source0, unsigned int source_size,
		  unsigned long *dest0, unsigned int dest_size ) {

	//
	( void ) source0;
	( void ) dest0;
	( void ) source_size;
	( void ) dest_size;
}

/**
 * Shrink big integer
 *
 * @v source0		Element 0 of source big integer
 * @v source_size	Number of elements in source big integer
 * @v dest0		Element 0 of destination big integer
 * @v dest_size		Number of elements in destination big integer
 */
static inline __attribute__ (( always_inline )) void
bigint_shrink_raw ( const unsigned long *source0,
		    unsigned int source_size __unused,
		    unsigned long *dest0, unsigned int dest_size ) {

	//
	( void ) source0;
	( void ) dest0;
	( void ) source_size;
	( void ) dest_size;
}

/**
 * Finalise big integer
 *
 * @v value0		Element 0 of big integer to finalise
 * @v size		Number of elements
 * @v out		Output buffer
 * @v len		Length of output buffer
 */
static inline __attribute__ (( always_inline )) void
bigint_done_raw ( const unsigned long *value0, unsigned int size __unused,
		  void *out, size_t len ) {
	const uint8_t *value_byte = ( ( const void * ) value0 );
	uint8_t *out_byte = ( out + len );

	/* Copy raw data in reverse order */
	while ( len-- )
		*(--out_byte) = *(value_byte++);
}

/**
 * Multiply big integer elements
 *
 * @v multiplicand	Multiplicand element
 * @v multiplier	Multiplier element
 * @v result		Result element
 * @v carry		Carry element
 */
static inline __attribute__ (( always_inline )) void
bigint_multiply_one ( const unsigned long multiplicand,
		      const unsigned long multiplier,
		      unsigned long *result, unsigned long *carry ) {

	//
	( void ) multiplicand;
	( void ) multiplier;
	( void ) result;
	( void ) carry;
}

#endif /* _BITS_BIGINT_H */
