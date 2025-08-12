#ifndef _DWI2C_H
#define _DWI2C_H

/** @file
 *
 * Synopsys DesignWare I2C driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

/** I2C Control Register */
#define DWI2C_CTL 0x00
#define DWI2C_CTL_BUS_CLEAR	0x00000800	/**< Bus clear enabled */
#define DWI2C_CTL_TARGET_DIS	0x00000040	/**< Target mode disabled */
#define DWI2C_CTL_RESTART_ENA	0x00000020	/**< Restart enabled */
#define DWI2C_CTL_SPEED( x ) 	( (x) << 1 )	/**< Bus speed */
#define DWI2C_CTL_SPEED_STD \
	DWI2C_CTL_SPEED ( 0x1 )			/**< Standard speed */
#define DWI2C_CTL_CTRLR_ENA	0x00000001	/**< Controller mode enabled */

/** I2C Target Address Register */
#define DWI2C_TAR 0x04
#define DWI2C_TAR_10BIT		0x00001000	/**< 10-bit target address */
#define DWI2C_TAR_MASK		0x000001ff	/**< Target address mask */

/** I2C RX/TX Data Buffer and Command Register */
#define DWI2C_DAT 0x10
#define DWI2C_DAT_READ		0x00000100	/**< Read command */
#define DWI2C_DAT_MASK		0x000000ff	/**< Data byte mask */

/** I2C Enable Register */
#define DWI2C_ENA 0x6c
#define DWI2C_ENA_I2C		0x00000001	/**< I2C enabled */

/** I2C Status Register */
#define DWI2C_STA 0x70
#define DWI2C_STA_RFNE		0x00000008	/**< RX FIFO not empty */
#define DWI2C_STA_TFE		0x00000004	/**< TX FIFO empty */

/** I2C Component Version Register */
#define DWI2C_VER 0xf8

/** I2C Component Type Register */
#define DWI2C_TYP 0xfc

/** Maximum time to wait for an I2C byte read or write to complete, in us */
#define DWI2C_MAX_WAIT_US 1000

/** A DesignWare I2C controller */
struct dwi2c {
	/** Device name */
	const char *name;
	/** Registers */
	void *regs;
};

#endif /* _DWI2C_H */
