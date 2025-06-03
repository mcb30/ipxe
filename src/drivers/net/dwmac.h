#ifndef _DWMAC_H
#define _DWMAC_H

/** @file
 *
 * Synopsys DesignWare MAC network driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

#include <stdint.h>
#include <ipxe/if_ether.h>

/** I/O region index */
#define DWMAC_REG_IDX 0

/** I/O region length */
#define DWMAC_REG_LEN 0x2000

/** MAC register block */
#define DWMAC_MAC 0x0000

/** MAC configuration register */
#define DWMAC_CFG ( DWMAC_MAC + 0x00 )
#define DWMAC_CFG_TXEN		0x00000008	/**< TX enabled */
#define DWMAC_CFG_RXEN		0x00000004	/**< RX enabled */

/** MAC filter register */
#define DWMAC_FILTER ( DWMAC_MAC + 0x04 )
#define DWMAC_FILTER_PR		0x00000001	/**< Promiscuous mode */

/** MAC address high register */
#define DWMAC_ADDRH ( DWMAC_MAC + 0x40 )

/** MAC address low register */
#define DWMAC_ADDRL ( DWMAC_MAC + 0x44 )

/** A DesignWare MAC address */
union dwmac_mac {
	struct {
		uint32_t addrl;
		uint32_t addrh;
	} __attribute__ (( packed )) reg;
	uint8_t raw[ETH_ALEN];
};

/** DMA register block */
#define DWMAC_DMA 0x1000

/** Bus mode register */
#define DWMAC_BUS ( DWMAC_DMA + 0x00 )
#define DWMAC_BUS_PBL4		0x01000000	/**< 4x PBL mode */
#define DWMAC_BUS_USP		0x00800000	/**< Use separate PBL */
#define DWMAC_BUS_RPBL(x)	( (x) << 17 )	/**< RX DMA PBL */
#define DWMAC_BUS_FB		0x00010000	/**< Fixed burst */
#define DWMAC_BUS_PBL(x)	( (x) << 8 )	/**< (TX) DMA PBL */
#define DWMAC_BUS_SWR		0x00000001	/**< Software reset */

/** Time to wait for software reset to complete */
#define DWMAC_RESET_MAX_WAIT_MS 500

/** Transmit poll demand register */
#define DWMAC_TXPOLL ( DWMAC_DMA + 0x04 )

/** Receive poll demand register */
#define DWMAC_RXPOLL ( DWMAC_DMA + 0x08 )

/** Receive descriptor list address register */
#define DWMAC_RXBASE ( DWMAC_DMA + 0x0c )

/** Transmit descriptor list address register */
#define DWMAC_TXBASE ( DWMAC_DMA + 0x10 )

/** Operation mode register */
#define DWMAC_OP ( DWMAC_DMA + 0x18 )
#define DWMAC_OP_TXEN		0x00002000	/**< TX enabled */
#define DWMAC_OP_RXEN		0x00000002	/**< RX enabled */

/** A frame descriptor */
struct dwmac_descriptor {
	/** Completion status */
	uint32_t stat;
	/** Buffer size */
	uint16_t size;
	/** Reserved */
	uint8_t reserved_a;
	/** Ring control */
	uint8_t ctrl;
	/** Buffer address */
	uint32_t addr;
	/** Reserved (second buffer address) */
	uint32_t reserved_b;
} __attribute__ (( packed ));

/* Completion status */
#define DWMAC_STAT_OWN		0x80000000	/**< Owned by hardware */
#define DWMAC_STAT_ERR		0x00008000	/**< Error summary */
#define DWMAC_STAT_RX_FIRST	0x00000200	/**< First segment (RX) */
#define DWMAC_STAT_RX_LAST	0x00000100	/**< Last segment (RX) */
#define DWMAC_STAT_RX_LEN(x) \
	( ( (x) >> 16 ) & 0x3ff )		/**< Frame length (RX) */

/* Ring control */
#define DWMAC_CTRL_TX_LAST	0x40		/**< Last segment (TX) */
#define DWMAC_CTRL_TX_FIRST	0x20		/**< First segment (TX) */
#define DWMAC_CTRL_END		0x02		/**< End of ring */

/** A DesignWare descriptor ring */
struct dwmac_ring {
	/** Descriptors */
	struct dwmac_descriptor *desc;
	/** Descriptor ring DMA mapping */
	struct dma_mapping map;
	/** Producer index */
	unsigned int prod;
	/** Consumer index */
	unsigned int cons;

	/** Queue base address register (within DMA block) */
	uint8_t qbase;
	/** Number of descriptors */
	uint8_t count;
	/** Default control flags */
	uint8_t ctrl;
	/** Length of descriptors */
	size_t len;
};

/** Number of transmit descriptors */
#define DWMAC_NUM_TX_DESC 8

/** Number of receive descriptors */
#define DWMAC_NUM_RX_DESC 8

/** Length of receive buffers
 *
 * Must be a multiple of 16.
 */
#define DWMAC_RX_LEN 1536

/**
 * Initialise descriptor ring
 *
 * @v ring		Descriptor ring
 * @v count		Number of descriptors
 * @v qbase		Queue base address register
 * @v ctrl		Default descriptor control flags
 */
static inline __attribute__ (( always_inline )) void
dwmac_init_ring ( struct dwmac_ring *ring, unsigned int count,
		  unsigned int qbase, unsigned int ctrl ) {

	ring->qbase = ( qbase - DWMAC_DMA );
	ring->count = count;
	ring->ctrl = ctrl;
	ring->len = ( count * sizeof ( ring->desc[0] ) );
}

/** A DesignWare MAC network card */
struct dwmac {
	/** Registers */
	void *regs;
	/** DMA device */
	struct dma_device *dma;
	/** Device name (for debugging) */
	const char *name;

	/** Transmit ring */
	struct dwmac_ring tx;
	/** Receive ring */
	struct dwmac_ring rx;
	/** Receive I/O buffers */
	struct io_buffer *rx_iobuf[DWMAC_NUM_RX_DESC];
};

#endif /* _DWMAC_H */
