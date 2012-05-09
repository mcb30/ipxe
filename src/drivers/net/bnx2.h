#ifndef _BNX2_H
#define _BNX2_H

/** @file
 *
 * Broadcom NetXtreme II series 10/100/1000 network card driver
 *
 */

#include <ipxe/mii.h>

FILE_LICENCE ( GPL2_OR_LATER );

/** bnx2 BAR size */
#define BNX2_BAR_SIZE									MB_GET_CID_ADDR ( 17 )

#define BNX2_FW_ACK_TIMEOUT_MS							1000
#define BNX2_MISC_ENABLE_DEFAULT						0x05ffffff

#define RV2P_PROC1										0
#define RV2P_PROC2										1

#define BNX2_RX_FILL									4

#define L1_CACHE_BYTES									128
#define L1_CACHE_ALIGN( X ) ( ( ( X ) + L1_CACHE_BYTES - 1 ) & ~ ( L1_CACHE_BYTES - 1 ) )

#define BNX2_RX_OFFSET			( 16 + 2 )
#define RX_BUF_USE_SIZE			( ETH_MAX_MTU + ETH_HLEN + BNX2_RX_OFFSET + 8 )
#define RX_BUF_SIZE				( L1_CACHE_ALIGN ( RX_BUF_USE_SIZE + 8 ) )

#define BNX2_PCICFG_REG_WINDOW_ADDRESS					0x00000078

#define BNX2_PCICFG_REG_WINDOW							0x00000080

#define BNX2_MISC_COMMAND								0x00000800
#define BNX2_MISC_COMMAND_CORE_RESET					( 1L << 4 )

#define BNX2_PCICFG_MISC_CONFIG							0x00000068
#define BNX2_PCICFG_MISC_CONFIG_TARGET_MB_WORD_SWAP		( 1L << 3 )
#define BNX2_PCICFG_MISC_CONFIG_REG_WINDOW_ENA			( 1L << 7 )
#define BNX2_PCICFG_MISC_CONFIG_CORE_RST_REQ			( 1L << 8 )
#define BNX2_PCICFG_MISC_CONFIG_CORE_RST_BSY			( 1L << 9 )

#define BNX2_PCI_SWAP_DIAG0								0x00000418
#define BNX2_PCI_SWAP_DIAG0_VALUE						0x01020304

#define BNX2_MISC_ID									0x00000808
#define BNX2_MISC_ID_CHIP_NUM							0xffff0000
#define BNX2_MISC_ID_CHIP_ID							0xfffffff0

#define BNX2_DEV_INFO_SIGNATURE							0x00000020
#define BNX2_DEV_INFO_SIGNATURE_MAGIC					0x44564900
#define BNX2_DEV_INFO_SIGNATURE_MAGIC_MASK				0xffffff00

#define BNX2_COM_CPU_MODE								0x00105000
#define BNX2_COM_CPU_STATE								0x00105004
#define BNX2_COM_CPU_PROGRAM_COUNTER					0x0010501c
#define BNX2_COM_CPU_INSTRUCTION						0x00105020
#define BNX2_COM_SCRATCH								0x00120000

#define BNX2_CP_CPU_MODE								0x00185000
#define BNX2_CP_CPU_STATE								0x00185004
#define BNX2_CP_CPU_PROGRAM_COUNTER						0x0018501c
#define BNX2_CP_CPU_INSTRUCTION							0x00185020
#define BNX2_CP_SCRATCH									0x001a0000

#define BNX2_RXP_CPU_MODE								0x000c5000
#define BNX2_RXP_CPU_STATE								0x000c5004
#define BNX2_RXP_CPU_PROGRAM_COUNTER					0x000c501c
#define BNX2_RXP_CPU_INSTRUCTION						0x000c5020
#define BNX2_RXP_SCRATCH								0x000e0000

#define BNX2_TPAT_CPU_MODE								0x00085000
#define BNX2_TPAT_CPU_STATE								0x00085004
#define BNX2_TPAT_CPU_PROGRAM_COUNTER					0x0008501c
#define BNX2_TPAT_CPU_INSTRUCTION						0x00085020
#define BNX2_TPAT_SCRATCH								0x000a0000

#define BNX2_TXP_CPU_MODE								0x00045000
#define BNX2_TXP_CPU_STATE								0x00045004
#define BNX2_TXP_CPU_PROGRAM_COUNTER					0x0004501c
#define BNX2_TXP_CPU_INSTRUCTION						0x00045020
#define BNX2_TXP_SCRATCH								0x00060000

#define BNX2_MCP_SCRATCH								0x00160000

#define BNX2_MIPS_VIEW_BASE								0x08000000
#define BNX2_CPU_MODE_SOFT_HALT							( 1L << 10 )

#define BCM_PAGE_BITS									12
#define BCM_PAGE_SIZE						( 1 << BCM_PAGE_BITS )

#define RV2P_P1_FIXUP_PAGE_SIZE_IDX						0
#define RV2P_BD_PAGE_SIZE_MASK							0xffff
#define RV2P_BD_PAGE_SIZE			( ( BCM_PAGE_SIZE / 16 ) - 1 )

#define BNX2_RV2P_INSTR_HIGH							0x00002830
#define BNX2_RV2P_INSTR_LOW								0x00002834

#define BNX2_RV2P_PROC1_ADDR_CMD						0x00002838
#define BNX2_RV2P_PROC1_ADDR_CMD_ADD					( 0x3ffL << 0 )
#define BNX2_RV2P_PROC1_ADDR_CMD_RDWR					( 1L << 31 )

#define BNX2_RV2P_PROC2_ADDR_CMD						0x0000283c
#define BNX2_RV2P_PROC2_ADDR_CMD_ADD					( 0x3ffL << 0 )
#define BNX2_RV2P_PROC2_ADDR_CMD_RDWR					( 1L << 31 )

#define BNX2_RV2P_COMMAND								0x00002800
#define BNX2_RV2P_COMMAND_PROC1_RESET					( 1L << 16 )
#define BNX2_RV2P_COMMAND_PROC2_RESET					( 1L << 17 )

#define BNX2_MISC_NEW_CORE_CTL							0x000008c8
#define BNX2_MISC_NEW_CORE_CTL_DMA_ENABLE				( 1L << 16 )

#define BNX2_PCICFG_DEVICE_STATUS						0x000000b6
#define BNX2_PCICFG_DEVICE_STATUS_NO_PEND					( 1 << 5 )

#define BNX2_MISC_ENABLE_CLR_BITS						0x00000814
#define BNX2_MISC_ENABLE_CLR_BITS_TX_DMA_ENABLE			( 1L << 4 )
#define BNX2_MISC_ENABLE_CLR_BITS_RX_DMA_ENABLE			( 1L << 17 )
#define BNX2_MISC_ENABLE_CLR_BITS_HOST_COALESCE_ENABLE	( 1L << 19 )
#define BNX2_MISC_ENABLE_CLR_BITS_DMA_ENGINE_ENABLE		( 1L << 26 )

#define BNX2_MISC_COMMAND								0x00000800
#define BNX2_MISC_COMMAND_SW_RESET						( 1L << 4 )

#define BNX2_DRV_RESET_SIGNATURE						0x00000000
#define BNX2_DRV_RESET_SIGNATURE_MAGIC					0x4841564b

#define BNX2_DRV_MSG_SEQ								0x0000ffff

#define BNX2_FW_MB										0x00000008
#define BNX2_FW_MSG_ACK									0x0000ffff
#define BNX2_FW_MSG_STATUS_MASK							0x00ff0000
#define BNX2_FW_MSG_STATUS_OK							0x00000000

#define BNX2_DRV_MB										0x00000004
#define BNX2_DRV_MSG_CODE								0xff000000
#define BNX2_DRV_MSG_CODE_RESET							0x01000000
#define BNX2_DRV_MSG_CODE_FW_TIMEOUT					0x05000000

#define BNX2_DRV_MSG_DATA								0x00ff0000
#define BNX2_DRV_MSG_DATA_WAIT0							0x00010000
#define BNX2_DRV_MSG_DATA_WAIT1							0x00020000
#define BNX2_DRV_MSG_DATA_WAIT2							0x00030000

#define BNX2_HC_COMMAND									0x00006800
#define BNX2_HC_COMMAND_ENABLE							( 1L << 0 )
#define BNX2_HC_COMMAND_COAL_NOW_WO_INT					( 1L << 17 )

#define BNX2_PORT_HW_CFG_MAC_UPPER						0x00000050
#define BNX2_PORT_HW_CFG_MAC_LOWER						0x00000054

#define BNX2_EMAC_MODE									0x00001400
#define BNX2_EMAC_MODE_HALF_DUPLEX						( 1L << 1 )
#define BNX2_EMAC_MODE_PORT_MII							( 1L << 2 )
#define BNX2_EMAC_MODE_PORT_GMII						( 2L << 2 )
#define BNX2_EMAC_MODE_PORT_MII_10						( 3L << 2 )
#define BNX2_EMAC_MODE_25G								( 1L << 5 )

#define BNX2_EMAC_MDIO_COMM								0x000014ac
#define BNX2_EMAC_MDIO_COMM_COMMAND_WRITE				( 1L << 26 )
#define BNX2_EMAC_MDIO_COMM_COMMAND_READ				( 2L << 26 )
#define BNX2_EMAC_MDIO_COMM_START_BUSY					( 1L << 29 )
#define BNX2_EMAC_MDIO_COMM_FAIL						( 1L << 28 )

#define BNX2_EMAC_BACKOFF_SEED							0x00001498
#define BNX2_EMAC_BACKOFF_SEED_EMAC_BACKOFF_SEED		( 0x3ffL << 0 )

#define BNX2_EMAC_RX_MODE								0x000014c8
#define BNX2_EMAC_RX_MODE_SORT_MODE						( 1L << 12 )

#define BNX2_PCICFG_INT_ACK_CMD							0x00000084
#define BNX2_PCICFG_INT_ACK_CMD_INDEX_VALID				( 1L << 16 )
#define BNX2_PCICFG_INT_ACK_CMD_MASK_INT				( 1L << 18 )

#define BNX2_HOST_VIEW_SHMEM_BASE						0x00167c00

#define BNX2_SHM_HDR_ADDR_0								BNX2_MCP_SCRATCH + 4

#define BNX2_SHM_HDR_SIGNATURE							BNX2_MCP_SCRATCH
#define BNX2_SHM_HDR_SIGNATURE_SIG_MASK					0xffff0000
#define BNX2_SHM_HDR_SIGNATURE_SIG						0x53530000

#define BNX2_DMA_CONFIG									0x00000c08
#define BNX2_DMA_CONFIG_DATA_BYTE_SWAP					( 1L << 0 )
#define BNX2_DMA_CONFIG_DATA_WORD_SWAP					( 1L << 1 )
#define BNX2_DMA_CONFIG_CNTL_BYTE_SWAP					( 1L << 4 )
#define BNX2_DMA_CONFIG_CNTL_WORD_SWAP					( 1L << 5 )
#define BNX2_DMA_CONFIG_CNTL_PING_PONG_DMA				( 1L << 10 )

#define BNX2_EMAC_ATTENTION_ENA							0x00001408
#define BNX2_EMAC_ATTENTION_ENA_LINK					( 1L << 11 )

#define BNX2_HC_ATTN_BITS_ENABLE						0x0000680c
#define BNX2_HC_STATUS_ADDR_L							0x00006810
#define BNX2_HC_STATUS_ADDR_H							0x00006814

#define BNX2_MISC_ENABLE_SET_BITS						0x00000810
#define BNX2_MISC_ENABLE_SET_BITS_RX_V2P_ENABLE			( 1L << 15 )
#define BNX2_MISC_ENABLE_SET_BITS_HOST_COALESCE_ENABLE	( 1L << 19 )
#define BNX2_MISC_ENABLE_SET_BITS_CONTEXT_ENABLE		( 1L << 21 )

#define CHIP_NUM(misc_id)	( ( misc_id ) & BNX2_MISC_ID_CHIP_NUM )
#define CHIP_NUM_5706									0x57060000
#define CHIP_NUM_5708									0x57080000
#define CHIP_NUM_5709									0x57090000

#define CHIP_ID(misc_id)	( ( misc_id ) & BNX2_MISC_ID_CHIP_ID )
#define CHIP_ID_5706_A0									0x57060000
#define CHIP_ID_5709_A0									0x57090000
#define CHIP_ID_5709_A1									0x57090010

#define BNX2_CTX_VIRT_ADDR								0x00001008

#define BNX2_CTX_PAGE_TBL								0x0000100c

#define BNX2_CTX_COMMAND								0x00001000
#define BNX2_CTX_COMMAND_ENABLED						( 1L << 0 )
#define BNX2_CTX_COMMAND_MEM_INIT						( 1L << 13 )

#define BNX2_CTX_HOST_PAGE_TBL_CTRL						0x000010c8
#define BNX2_CTX_HOST_PAGE_TBL_CTRL_WRITE_REQ			( 1L << 30 )

#define BNX2_CTX_HOST_PAGE_TBL_DATA0					0x000010cc
#define BNX2_CTX_HOST_PAGE_TBL_DATA0_VALID				( 1L << 0 )

#define BNX2_CTX_HOST_PAGE_TBL_DATA1					0x000010d0

#define CTX_SHIFT						7
#define CTX_SIZE						( 1 << CTX_SHIFT )
#define CTX_MASK						( CTX_SIZE - 1 )
#define GET_CTX_ID_ADDR(cid)			( ( cid ) << CTX_SHIFT )
#define GET_CTX_ID(cid_addr)			( ( cid_addr ) >> CTX_SHIFT )

#define PHY_CTX_SHIFT					6
#define PHY_CTX_SIZE					( 1 << PHY_CTX_SHIFT )
#define PHY_CTX_MASK					( PHY_CTX_SIZE - 1 )
#define GET_PHY_CTX_ID_ADDR(pcid)		( ( pcid ) << PHY_CTX_SHIFT )
#define GET_PHY_CTX_ID(pcid_addr)		( ( pcid_addr ) >> PHY_CTX_SHIFT )

#define BNX2_CTX_CTRL									0x0000101c
#define BNX2_CTX_CTRL_WRITE_REQ							( 1 << 30 )

#define BNX2_CTX_CTX_DATA								0x00001020

#define BNX2_CTX_DATA									0x00001014

#define BNX2_CTX_DATA_ADR								0x00001010

#define BNX2_RV2P_CONFIG								0x00002808

#define BNX2_L2CTX_CTX_TYPE								0x00000000
#define BNX2_L2CTX_CTX_TYPE_CTX_BD_CHN_TYPE_VALUE		( 1 << 28 )
#define BNX2_L2CTX_CTX_TYPE_SIZE_L2					( ( 0x20 / 20 ) <<16 )

#define BNX2_L2CTX_TBDR_BHADDR_HI						0x000000a0
#define BNX2_L2CTX_TBDR_BHADDR_LO						0x000000a4

#define BNX2_L2CTX_TYPE_XI								0x00000080
#define BNX2_L2CTX_CMD_TYPE_XI							0x00000240
#define BNX2_L2CTX_TBDR_BHADDR_HI_XI					0x00000258
#define BNX2_L2CTX_TBDR_BHADDR_LO_XI					0x0000025c

#define BNX2_L2CTX_CMD_TYPE								0x00000088
#define BNX2_L2CTX_CMD_TYPE_TYPE_L2						( 0 << 24 )

#define BNX2_L2CTX_TYPE									0x00000000
#define BNX2_L2CTX_TYPE_SIZE_L2				( ( 0xc0 / 0x20 ) << 16 )
#define BNX2_L2CTX_TYPE_TYPE_L2							( 1 << 28 )

#define BNX2_PCICFG_STATUS_BIT_SET_CMD					0x00000088
#define BNX2_PCICFG_STATUS_BIT_CLEAR_CMD				0x0000008c

#define BNX2_L2CTX_TX_HOST_BIDX							0x00000088
#define BNX2_L2CTX_TX_HOST_BSEQ							0x00000090

#define BNX2_L2CTX_NX_BDHADDR_HI						0x00000010
#define BNX2_L2CTX_NX_BDHADDR_LO						0x00000014

#define BNX2_L2CTX_HOST_BDIDX							0x00000004
#define BNX2_L2CTX_HOST_BSEQ							0x00000008

#define BNX2_RPM_SORT_USER0								0x00001820
#define BNX2_EMAC_MULTICAST_HASH0						0x000014d0
#define BNX2_RPM_SORT_USER0_BC_EN						( 1L << 16 )
#define BNX2_RPM_SORT_USER0_MC_EN						( 1L << 17 )
#define BNX2_RPM_SORT_USER0_ENA							( 1L << 31 )
#define NUM_MC_HASH_REGISTERS							8

#define BNX2_EMAC_MAC_MATCH0							0x00001410
#define BNX2_EMAC_MAC_MATCH1							0x00001414

#define TX_DESC_CNT			( BCM_PAGE_SIZE / sizeof ( struct bnx2_tx_bd ) )
#define RX_DESC_CNT			( BCM_PAGE_SIZE / sizeof ( struct bnx2_rx_bd ) )
#define MAX_TX_DESC_CNT		( TX_DESC_CNT - 1 )
#define MAX_RX_DESC_CNT		( RX_DESC_CNT - 1 )

#define NEXT_TX_BD(x) (((x) & (MAX_TX_DESC_CNT - 1)) ==			\
		(MAX_TX_DESC_CNT - 1)) ?				\
	(x) + 2 : (x) + 1

#define NEXT_RX_BD(x) (((x) & (MAX_RX_DESC_CNT - 1)) ==			\
		(MAX_RX_DESC_CNT - 1)) ?				\
	(x) + 2 : (x) + 1

#define MB_KERNEL_CTX_SHIFT								8
#define MB_KERNEL_CTX_SIZE				( 1 << MB_KERNEL_CTX_SHIFT )
#define MB_KERNEL_CTX_MASK				( MB_KERNEL_CTX_SIZE - 1 )
#define MB_GET_CID_ADDR(_cid)	( 0x10000 + ( ( _cid ) << MB_KERNEL_CTX_SHIFT ) )

#define TX_CID											16
#define RX_CID											0

#define TX_CID_ADDR							GET_CTX_ID_ADDR ( TX_CID )
#define RX_CID_ADDR							GET_CTX_ID_ADDR ( RX_CID )

#define MB_TX_CID_ADDR						MB_GET_CID_ADDR(TX_CID)
#define MB_RX_CID_ADDR						MB_GET_CID_ADDR(RX_CID)

struct bnx2_status_block {
	uint32_t status_attn_bits;
	#define STATUS_ATTN_BITS_LINK_STATE					( 1L << 0 )
	#define STATUS_ATTN_BITS_TX_SCHEDULER_ABORT			( 1L << 1 )
	#define STATUS_ATTN_BITS_TX_BD_READ_ABORT			( 1L << 2 )
	#define STATUS_ATTN_BITS_TX_BD_CACHE_ABORT			( 1L << 3 )
	#define STATUS_ATTN_BITS_TX_PROCESSOR_ABORT			( 1L << 4 )
	#define STATUS_ATTN_BITS_TX_DMA_ABORT				( 1L << 5 )
	#define STATUS_ATTN_BITS_TX_PATCHUP_ABORT			( 1L << 6 )
	#define STATUS_ATTN_BITS_TX_ASSEMBLER_ABORT			( 1L << 7 )
	#define STATUS_ATTN_BITS_RX_PARSER_MAC_ABORT		( 1L << 8 )
	#define STATUS_ATTN_BITS_RX_PARSER_CATCHUP_ABORT	( 1L << 9 )
	#define STATUS_ATTN_BITS_RX_MBUF_ABORT				( 1L << 10 )
	#define STATUS_ATTN_BITS_RX_LOOKUP_ABORT			( 1L << 11 )
	#define STATUS_ATTN_BITS_RX_PROCESSOR_ABORT			( 1L << 12 )
	#define STATUS_ATTN_BITS_RX_V2P_ABORT				( 1L << 13 )
	#define STATUS_ATTN_BITS_RX_BD_CACHE_ABORT			( 1L << 14 )
	#define STATUS_ATTN_BITS_RX_DMA_ABORT				( 1L << 15 )
	#define STATUS_ATTN_BITS_COMPLETION_ABORT			( 1L << 16 )
	#define STATUS_ATTN_BITS_HOST_COALESCE_ABORT		( 1L << 17 )
	#define STATUS_ATTN_BITS_MAILBOX_QUEUE_ABORT		( 1L << 18 )
	#define STATUS_ATTN_BITS_CONTEXT_ABORT				( 1L << 19 )
	#define STATUS_ATTN_BITS_CMD_SCHEDULER_ABORT		( 1L << 20 )
	#define STATUS_ATTN_BITS_CMD_PROCESSOR_ABORT		( 1L << 21 )
	#define STATUS_ATTN_BITS_MGMT_PROCESSOR_ABORT		( 1L << 22 )
	#define STATUS_ATTN_BITS_MAC_ABORT					( 1L << 23 )
	#define STATUS_ATTN_BITS_TIMER_ABORT				( 1L << 24 )
	#define STATUS_ATTN_BITS_DMAE_ABORT					( 1L << 25 )
	#define STATUS_ATTN_BITS_FLSH_ABORT					( 1L << 26 )
	#define STATUS_ATTN_BITS_GRC_ABORT					( 1L << 27 )
	#define STATUS_ATTN_BITS_EPB_ERROR					( 1L << 30 )
	#define STATUS_ATTN_BITS_PARITY_ERROR				( 1L << 31 )

	uint32_t status_attn_bits_ack;
#if __BYTE_ORDER == __BIG_ENDIAN
	uint16_t status_tx_quick_consumer_index0;
	uint16_t status_tx_quick_consumer_index1;
	uint16_t status_tx_quick_consumer_index2;
	uint16_t status_tx_quick_consumer_index3;
	uint16_t status_rx_quick_consumer_index0;
	uint16_t status_rx_quick_consumer_index1;
	uint16_t status_rx_quick_consumer_index2;
	uint16_t status_rx_quick_consumer_index3;
	uint16_t status_rx_quick_consumer_index4;
	uint16_t status_rx_quick_consumer_index5;
	uint16_t status_rx_quick_consumer_index6;
	uint16_t status_rx_quick_consumer_index7;
	uint16_t status_rx_quick_consumer_index8;
	uint16_t status_rx_quick_consumer_index9;
	uint16_t status_rx_quick_consumer_index10;
	uint16_t status_rx_quick_consumer_index11;
	uint16_t status_rx_quick_consumer_index12;
	uint16_t status_rx_quick_consumer_index13;
	uint16_t status_rx_quick_consumer_index14;
	uint16_t status_rx_quick_consumer_index15;
	uint16_t status_completion_producer_index;
	uint16_t status_cmd_consumer_index;
	uint16_t status_idx;
	uint8_t status_unused;
	uint8_t status_blk_num;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
	uint16_t status_tx_quick_consumer_index1;
	uint16_t status_tx_quick_consumer_index0;
	uint16_t status_tx_quick_consumer_index3;
	uint16_t status_tx_quick_consumer_index2;
	uint16_t status_rx_quick_consumer_index1;
	uint16_t status_rx_quick_consumer_index0;
	uint16_t status_rx_quick_consumer_index3;
	uint16_t status_rx_quick_consumer_index2;
	uint16_t status_rx_quick_consumer_index5;
	uint16_t status_rx_quick_consumer_index4;
	uint16_t status_rx_quick_consumer_index7;
	uint16_t status_rx_quick_consumer_index6;
	uint16_t status_rx_quick_consumer_index9;
	uint16_t status_rx_quick_consumer_index8;
	uint16_t status_rx_quick_consumer_index11;
	uint16_t status_rx_quick_consumer_index10;
	uint16_t status_rx_quick_consumer_index13;
	uint16_t status_rx_quick_consumer_index12;
	uint16_t status_rx_quick_consumer_index15;
	uint16_t status_rx_quick_consumer_index14;
	uint16_t status_cmd_consumer_index;
	uint16_t status_completion_producer_index;
	uint8_t status_blk_num;
	uint8_t status_unused;
	uint16_t status_idx;
#endif
};

struct bnx2_l2_fhdr {
#if __BYTE_ORDER == __BIG_ENDIAN
	uint16_t errors;
	uint16_t status;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
	uint16_t status;
	uint16_t errors;
#endif

	uint32_t hash;

#if __BYTE_ORDER == __BIG_ENDIAN
	uint16_t pkt_len;
	uint16_t vlan_tag;
	uint16_t ip_xsum;
	uint16_t tcp_udp_xsum;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
	uint16_t vlan_tag;
	uint16_t pkt_len;
	uint16_t tcp_udp_xsum;
	uint16_t ip_xsum;
#endif
} __attribute__ (( packed ));

struct bnx2_tx_bd {
	uint32_t haddr_hi;
	uint32_t haddr_lo;
	uint16_t nbytes;
	uint16_t reserved;
	uint16_t flags;
	uint16_t vlan_tag;
	#define TX_BD_FLAGS_END			( 1 << 6 )
	#define TX_BD_FLAGS_START		( 1 << 7 )
} __attribute__ (( packed ));

struct bnx2_tx_ring_info {
	uint32_t prod_bseq;
	uint16_t prod;
	uint16_t cons;
	struct bnx2_tx_bd *desc;
};

struct bnx2_rx_bd {
	uint32_t haddr_hi;
	uint32_t haddr_lo;
	uint32_t len;
	uint16_t flags;
	uint16_t reserved;
	#define RX_BD_FLAGS_NOPUSH		( 1 << 0 )
	#define RX_BD_FLAGS_DUMMY		( 1 << 1 )
	#define RX_BD_FLAGS_END			( 1 << 2 )
	#define RX_BD_FLAGS_START		( 1 << 3 )
} __attribute__ (( packed ));

struct bnx2_rx_ring_info {
	uint32_t prod_bseq;
	uint16_t prod;
	uint16_t cons;
	struct bnx2_rx_bd *desc;
};

/** A bnx2 network card */
struct bnx2_nic {
	/** Registers */
	void *regs;

	/** PCI device */
	struct pci_device *pci;
	/** From MISC_ID register */
	uint32_t misc_id;
	/** Shared memory */
	uint32_t shmem;

	struct bnx2_status_block *status_blk;
	/** Firmware write counter */
	uint16_t fw_write_sequence;

	struct bnx2_tx_ring_info tx_ring;
	struct bnx2_rx_ring_info rx_ring;
	
	void *ctx_blk[4];
	int ctx_pages;

	uint32_t phy_addr;

	struct net_device *netdev;
	uint32_t hc_cmd;

	struct io_buffer *rx_iobuf[RX_DESC_CNT];

	/** MII interface */
	struct mii_interface mii;
};

#endif /* _BNX2_H */
