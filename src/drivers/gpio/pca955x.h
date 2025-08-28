#ifndef _PCA955X_H
#define _PCA955X_H

/** @file
 *
 * Philips Semiconductors PCA9555/PCA9554 and derivative I2C GPIO controllers
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

/** Input port register(s) */
#define PCA955X_INPUT(shift) ( 0 << (shift) )

/** Output port register(s) */
#define PCA955X_OUTPUT(shift) ( 1 << (shift) )

/** Polarity inversion register(s) */
#define PCA955X_POLARITY(shift) ( 2 << (shift) )

/** Configuration register(s) */
#define PCA955X_CONFIG(shift) ( 3 << (shift) )

/** Maximum register shift
 *
 * We assume that up to 32-bit parts may exist.
 */
#define PCA955X_MAX_SHIFT 2

/** A PCA955x-compatible GPIO controller */
struct pca955x {
	/** Device name */
	const char *name;
	/** Register shift */
	unsigned int shift;
};

#endif /* _PCA955X_H */

