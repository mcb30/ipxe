/*
 * Copyright (C) 2024 Michael Brown <mbrown@fensystems.co.uk>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
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

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <byteswap.h>
#include <ipxe/netdevice.h>
#include <ipxe/ethernet.h>
#include <ipxe/if_ether.h>
#include <ipxe/iobuf.h>
#include <ipxe/dma.h>
#include <ipxe/pci.h>
#include "gve.h"

/** @file
 *
 * Google Virtual Ethernet network driver
 *
 */

/* Disambiguate the various error causes */
#define EINFO_EIO_ADMIN_UNSET						\
	__einfo_uniqify ( EINFO_EIO, 0x00, "Uncompleted" )
#define EIO_ADMIN_UNSET							\
	__einfo_error ( EINFO_EIO_ADMIN_UNSET )
#define EINFO_EIO_ADMIN_ABORTED						\
	__einfo_uniqify ( EINFO_EIO, 0x10, "Aborted" )
#define EIO_ADMIN_ABORTED						\
	__einfo_error ( EINFO_EIO_ADMIN_ABORTED )
#define EINFO_EIO_ADMIN_EXISTS						\
	__einfo_uniqify ( EINFO_EIO, 0x11, "Already exists" )
#define EIO_ADMIN_EXISTS						\
	__einfo_error ( EINFO_EIO_ADMIN_EXISTS )
#define EINFO_EIO_ADMIN_CANCELLED					\
	__einfo_uniqify ( EINFO_EIO, 0x12, "Cancelled" )
#define EIO_ADMIN_CANCELLED						\
	__einfo_error ( EINFO_EIO_ADMIN_CANCELLED )
#define EINFO_EIO_ADMIN_DATALOSS					\
	__einfo_uniqify ( EINFO_EIO, 0x13, "Data loss" )
#define EIO_ADMIN_DATALOSS						\
	__einfo_error ( EINFO_EIO_ADMIN_DATALOSS )
#define EINFO_EIO_ADMIN_DEADLINE					\
	__einfo_uniqify ( EINFO_EIO, 0x14, "Deadline exceeded" )
#define EIO_ADMIN_DEADLINE						\
	__einfo_error ( EINFO_EIO_ADMIN_DEADLINE )
#define EINFO_EIO_ADMIN_PRECONDITION					\
	__einfo_uniqify ( EINFO_EIO, 0x15, "Failed precondition" )
#define EIO_ADMIN_PRECONDITION						\
	__einfo_error ( EINFO_EIO_ADMIN_PRECONDITION )
#define EINFO_EIO_ADMIN_INTERNAL					\
	__einfo_uniqify ( EINFO_EIO, 0x16, "Internal error" )
#define EIO_ADMIN_INTERNAL						\
	__einfo_error ( EINFO_EIO_ADMIN_INTERNAL )
#define EINFO_EIO_ADMIN_INVAL						\
	__einfo_uniqify ( EINFO_EIO, 0x17, "Invalid argument" )
#define EIO_ADMIN_INVAL							\
	__einfo_error ( EINFO_EIO_ADMIN_INVAL )
#define EINFO_EIO_ADMIN_NOT_FOUND					\
	__einfo_uniqify ( EINFO_EIO, 0x18, "Not found" )
#define EIO_ADMIN_NOT_FOUND						\
	__einfo_error ( EINFO_EIO_ADMIN_NOT_FOUND )
#define EINFO_EIO_ADMIN_RANGE						\
	__einfo_uniqify ( EINFO_EIO, 0x19, "Out of range" )
#define EIO_ADMIN_RANGE							\
	__einfo_error ( EINFO_EIO_ADMIN_RANGE )
#define EINFO_EIO_ADMIN_PERM						\
	__einfo_uniqify ( EINFO_EIO, 0x1a, "Permission denied" )
#define EIO_ADMIN_PERM							\
	__einfo_error ( EINFO_EIO_ADMIN_PERM )
#define EINFO_EIO_ADMIN_UNAUTH						\
	__einfo_uniqify ( EINFO_EIO, 0x1b, "Unauthenticated" )
#define EIO_ADMIN_UNAUTH						\
	__einfo_error ( EINFO_EIO_ADMIN_UNAUTH )
#define EINFO_EIO_ADMIN_RESOURCE					\
	__einfo_uniqify ( EINFO_EIO, 0x1c, "Resource exhausted" )
#define EIO_ADMIN_RESOURCE						\
	__einfo_error ( EINFO_EIO_ADMIN_RESOURCE )
#define EINFO_EIO_ADMIN_UNAVAIL						\
	__einfo_uniqify ( EINFO_EIO, 0x1d, "Unavailable" )
#define EIO_ADMIN_UNAVAIL						\
	__einfo_error ( EINFO_EIO_ADMIN_UNAVAIL )
#define EINFO_EIO_ADMIN_NOTSUP						\
	__einfo_uniqify ( EINFO_EIO, 0x1e, "Unimplemented" )
#define EIO_ADMIN_NOTSUP	       					\
	__einfo_error ( EINFO_EIO_ADMIN_NOTSUP )
#define EINFO_EIO_ADMIN_UNKNOWN						\
	__einfo_uniqify ( EINFO_EIO, 0x1f, "Unknown error" )
#define EIO_ADMIN_UNKNOWN						\
	__einfo_error ( EINFO_EIO_ADMIN_UNKNOWN )
#define EIO_ADMIN( status )						\
	EUNIQ ( EINFO_EIO, ( (status) & 0x1f ),				\
		EIO_ADMIN_UNSET, EIO_ADMIN_ABORTED, EIO_ADMIN_EXISTS,	\
		EIO_ADMIN_CANCELLED, EIO_ADMIN_DATALOSS,		\
		EIO_ADMIN_DEADLINE, EIO_ADMIN_NOT_FOUND,		\
		EIO_ADMIN_RANGE, EIO_ADMIN_PERM, EIO_ADMIN_UNAUTH,	\
		EIO_ADMIN_RESOURCE, EIO_ADMIN_UNAVAIL,			\
		EIO_ADMIN_NOTSUP, EIO_ADMIN_UNKNOWN )

/******************************************************************************
 *
 * Device reset
 *
 ******************************************************************************
 */

/**
 * Reset hardware
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_reset ( struct gve_nic *gve ) {
	uint32_t devstat;
	uint32_t pfn;
	unsigned int i;

	/* Clear admin queue page frame number (for older hardware) */
	writel ( 0, gve->cfg + GVE_CFG_ADMIN_PFN );

	/* Trigger reset (for newer hardware) */

	///
	if ( 0 ) {
	writel ( bswap_32 ( GVE_CFG_DRVSTAT_RESET ),
		 gve->cfg + GVE_CFG_DRVSTAT );
	}

	/* Wait for device to reset */
	for ( i = 0 ; i < GVE_RESET_MAX_WAIT_MS ; i++ ) {

		/* Check for reset completion */
		devstat = readl ( gve->cfg + GVE_CFG_DEVSTAT );
		pfn = readl ( gve->cfg + GVE_CFG_ADMIN_PFN );
		if ( ( ( devstat & bswap_32 ( GVE_CFG_DEVSTAT_RESET ) ) ||
		       ( gve->revision == 0 ) ) &&
		     ( pfn == 0 ) ) {
			return 0;
		}

		/* Delay */
		mdelay ( 1 );
	}

	DBGC ( gve, "GVE %p reset timed out (PFN %#08x devstat %#08x)\n",
	       gve, bswap_32 ( pfn ), bswap_32 ( devstat ) );
	return -ETIMEDOUT;
}

/******************************************************************************
 *
 * DMA allocation
 *
 ******************************************************************************
 */

/**
 * Allocate and map DMA-coherent buffer
 *
 * @v gve		GVE device
 * @v map		DMA mapping to fill in
 * @v len		Length of buffer
 * @ret addr		Buffer address, or NULL on error
 */
static inline __attribute__ (( always_inline )) void *
gve_dma_alloc ( struct gve_nic *gve, struct dma_mapping *map, size_t len ) {

	/* The alignment requirements for DMA are almost completely
	 * undocumented.  The public source code seems to rely upon
	 * the implicit page alignment provided by e.g. the Linux
	 * kernel's DMA buffer allocation behaviour.
	 *
	 * Experimentation suggests that everything (even tiny data
	 * structures) must be aligned to a page boundary, and that
	 * the device will write blocks of 64 bytes.  Round up lengths
	 * as needed.
	 */
	len = ( ( len + GVE_LEN_ALIGN - 1 ) & ~( GVE_LEN_ALIGN - 1 ) );

	return dma_alloc ( gve->dma, map, len, GVE_ADDR_ALIGN );
}

/**
 * Free DMA-coherent buffer
 *
 * @v gve		GVE device
 * @v map		DMA mapping
 * @v addr		Buffer address
 * @v len		Length of buffer
 */
static inline __attribute__ (( always_inline )) void
gve_dma_free ( struct gve_nic *gve __unused, struct dma_mapping *map,
	       void *addr, size_t len ) {

	/* Round up length to match that used for allocation */
	len = ( ( len + GVE_LEN_ALIGN - 1 ) & ~( GVE_LEN_ALIGN - 1 ) );

	/* Free buffer */
	dma_free ( map, addr, len );
}

/******************************************************************************
 *
 * Admin queue
 *
 ******************************************************************************
 */

/**
 * Allocate admin queue
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_admin_alloc ( struct gve_nic *gve ) {
	struct gve_admin *admin = &gve->admin;
	struct gve_irqs *irqs = &gve->irqs;
	struct gve_events *events = &gve->events;
	struct gve_scratch *scratch = &gve->scratch;
	size_t admin_len = ( GVE_ADMIN_COUNT * sizeof ( admin->cmd[0] ) );
	size_t irqs_len = ( GVE_IRQ_COUNT * sizeof ( irqs->irq[0] ) );
	size_t events_len = ( GVE_EVENT_MAX * sizeof ( events->event[0] ) );
	size_t scratch_len = sizeof ( *scratch->buf );
	int rc;

	/* Allocate admin queue */
	admin->cmd = gve_dma_alloc ( gve, &admin->map, admin_len );
	if ( ! admin->cmd ) {
		rc = -ENOMEM;
		goto err_admin;
	}

	/* Allocate interrupt channels */
	irqs->irq = gve_dma_alloc ( gve, &irqs->map, irqs_len );
	if ( ! irqs->irq ) {
		rc = -ENOMEM;
		goto err_irqs;
	}

	/* Allocate event counters */
	events->event = gve_dma_alloc ( gve, &events->map, events_len );
	if ( ! events->event ) {
		rc = -ENOMEM;
		goto err_events;
	}

	/* Allocate scratch buffer */
	scratch->buf = gve_dma_alloc ( gve, &scratch->map, scratch_len );
	if ( ! scratch->buf ) {
		rc = -ENOMEM;
		goto err_scratch;
	}

	DBGC ( gve, "GVE %p AQ at [%08lx,%08lx)\n",
	       gve, virt_to_phys ( admin->cmd ),
	       ( virt_to_phys ( admin->cmd ) + admin_len ) );
	return 0;

	gve_dma_free ( gve, &scratch->map, scratch->buf, scratch_len );
 err_scratch:
	gve_dma_free ( gve, &events->map, events->event, events_len );
 err_events:
	gve_dma_free ( gve, &irqs->map, irqs->irq, irqs_len );
 err_irqs:
	gve_dma_free ( gve, &admin->map, admin->cmd, admin_len );
 err_admin:
	return rc;
}

/**
 * Free admin queue
 *
 * @v gve		GVE device
 */
static void gve_admin_free ( struct gve_nic *gve ) {
	struct gve_admin *admin = &gve->admin;
	struct gve_irqs *irqs = &gve->irqs;
	struct gve_events *events = &gve->events;
	struct gve_scratch *scratch = &gve->scratch;
	size_t admin_len = ( GVE_ADMIN_COUNT * sizeof ( admin->cmd[0] ) );
	size_t irqs_len = ( GVE_IRQ_COUNT * sizeof ( irqs->irq[0] ) );
	size_t events_len = ( GVE_EVENT_MAX * sizeof ( events->event[0] ) );
	size_t scratch_len = sizeof ( *scratch->buf );

	/* Leak memory if we were unable to reset the device */
	if ( ! ( admin->cmd && events->event ) )
		return;

	/* Free scratch buffer */
	gve_dma_free ( gve, &scratch->map, scratch->buf, scratch_len );

	/* Free event counter */
	gve_dma_free ( gve, &events->map, events->event, events_len );

	/* Free interrupt channels */
	gve_dma_free ( gve, &irqs->map, irqs->irq, irqs_len );

	/* Free admin queue */
	gve_dma_free ( gve, &admin->map, admin->cmd, admin_len );
}

/**
 * Enable admin queue
 *
 * @v gve		GVE device
 */
static void gve_admin_enable ( struct gve_nic *gve ) {
	struct gve_admin *admin = &gve->admin;
	size_t admin_len = ( GVE_ADMIN_COUNT * sizeof ( admin->cmd[0] ) );
	physaddr_t base;

	/* Reset queue */
	admin->prod = 0;

	/* Program queue addresses and capabilities */
	base = dma ( &admin->map, admin->cmd );
	writel ( bswap_32 ( base / GVE_PAGE_SIZE ),
		 gve->cfg + GVE_CFG_ADMIN_PFN );
	writel ( bswap_32 ( base & 0xffffffffUL ),
		 gve->cfg + GVE_CFG_ADMIN_BASE_LO );
	if ( sizeof ( base ) > sizeof ( uint32_t ) ) {
		writel ( bswap_32 ( ( ( uint64_t ) base ) >> 32 ),
			 gve->cfg + GVE_CFG_ADMIN_BASE_HI );
	} else {
		writel ( 0, gve->cfg + GVE_CFG_ADMIN_BASE_HI );
	}
	writel ( bswap_16 ( admin_len ), gve->cfg + GVE_CFG_ADMIN_LEN );
	writel ( bswap_32 ( GVE_CFG_DRVSTAT_RUN ), gve->cfg + GVE_CFG_DRVSTAT );
}

/**
 * Disable admin queue and reset device
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_admin_disable ( struct gve_nic *gve ) {
	struct gve_admin *admin = &gve->admin;
	int rc;

	/* Reset device */
	if ( ( rc = gve_reset ( gve ) ) != 0 ) {
		DBGC ( gve, "GVE %p could not free AQ: %s\n",
		       gve, strerror ( rc ) );
		/* Leak memory: there is nothing else we can do */
		admin->cmd = NULL;
		return rc;
	}

	return 0;
}

/**
 * Get next available admin queue command slot
 *
 * @v gve		GVE device
 * @ret cmd		Admin queue command
 */
static union gve_admin_command * gve_admin_command ( struct gve_nic *gve ) {
	struct gve_admin *admin = &gve->admin;
	union gve_admin_command *cmd;
	unsigned int index;

	/* Get next command slot */
	index = admin->prod;
	cmd = &admin->cmd[ index % GVE_ADMIN_COUNT ];

	/* Initialise request */
	memset ( cmd, 0, sizeof ( *cmd ) );

	return cmd;
}

/**
 * Wait for admin queue command to complete
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_admin_wait ( struct gve_nic *gve ) {
	struct gve_admin *admin = &gve->admin;
	uint32_t evt;
	unsigned int i;

	/* Wait for any outstanding commands to complete */
	for ( i = 0 ; i < GVE_ADMIN_MAX_WAIT_MS ; i++ ) {

		/* Check event counter */
		rmb();
		evt = bswap_32 ( readl ( gve->cfg + GVE_CFG_ADMIN_EVT ) );
		if ( evt == admin->prod )
			return 0;

		/* Delay */
		mdelay ( 1 );
	}

	DBGC ( gve, "GVE %p AQ %#02x timed out (completed %#02x, "
	       "status %#08x)\n", gve, admin->prod, evt,
	       bswap_32 ( readl ( gve->cfg + GVE_CFG_DEVSTAT ) ) );
	return -ETIMEDOUT;
}

/**
 * Issue admin queue command
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_admin ( struct gve_nic *gve ) {
	struct gve_admin *admin = &gve->admin;
	union gve_admin_command *cmd;
	unsigned int index;
	uint32_t opcode;
	uint32_t status;
	int rc;

	/* Ensure admin queue is idle */
	if ( ( rc = gve_admin_wait ( gve ) ) != 0 )
		return rc;

	/* Get next command slot */
	index = admin->prod;
	cmd = &admin->cmd[ index % GVE_ADMIN_COUNT ];
	opcode = cmd->hdr.opcode;
	DBGC2 ( gve, "GVE %p AQ %#02x command %#04x request:\n",
		gve, index, opcode );
	DBGC2_HDA ( gve, 0, cmd, sizeof ( *cmd ) );

	/* Increment producer counter */
	admin->prod++;

	/* Ring doorbell */
	wmb();
	writel ( bswap_32 ( admin->prod ), gve->cfg + GVE_CFG_ADMIN_DB );

	/* Wait for command to complete */
	if ( ( rc = gve_admin_wait ( gve ) ) != 0 )
		return rc;

	/* Check command status */
	status = be32_to_cpu ( cmd->hdr.status );
	if ( status != GVE_ADMIN_STATUS_OK ) {
		rc = -EIO_ADMIN ( status );
		DBGC ( gve, "GVE %p AQ %#02x command %#04x failed: %#08x\n",
		       gve, index, opcode, status );
		DBGC_HDA ( gve, 0, cmd, sizeof ( *cmd ) );
		DBGC ( gve, "GVE %p AQ error: %s\n", gve, strerror ( rc ) );
		return rc;
	}

	DBGC2 ( gve, "GVE %p AQ %#02x command %#04x result:\n",
		gve, index, opcode );
	DBGC2_HDA ( gve, 0, cmd, sizeof ( *cmd ) );
	return 0;
}

/**
 * Issue simple admin queue command
 *
 * @v gve		GVE device
 * @v opcode		Operation code
 * @v id		ID parameter (or zero if not applicable)
 * @ret rc		Return status code
 *
 * Several admin queue commands take either an empty parameter list or
 * a single 32-bit ID parameter.
 */
static int gve_admin_simple ( struct gve_nic *gve, unsigned int opcode,
			      unsigned int id ) {
	union gve_admin_command *cmd;
	int rc;

	/* Construct request */
	cmd = gve_admin_command ( gve );
	cmd->hdr.opcode = opcode;
	cmd->simple.id = cpu_to_be32 ( id );

	/* Issue command */
	if ( ( rc = gve_admin ( gve ) ) != 0 )
		return rc;

	return 0;
}

/**
 * Get device descriptor
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int gve_describe ( struct net_device *netdev ) {
	struct gve_nic *gve = netdev->priv;
	struct gve_device_descriptor *desc = &gve->scratch.buf->desc;
	union gve_admin_command *cmd;
	int rc;

	/* Construct request */
	cmd = gve_admin_command ( gve );
	cmd->hdr.opcode = GVE_ADMIN_DESCRIBE;
	cmd->desc.addr = cpu_to_be64 ( dma ( &gve->scratch.map, desc ) );
	cmd->desc.ver = cpu_to_be32 ( GVE_ADMIN_DESCRIBE_VER );
	cmd->desc.len = cpu_to_be32 ( sizeof ( *desc ) );

	/* Issue command */
	if ( ( rc = gve_admin ( gve ) ) != 0 )
		return rc;
	DBGC2 ( gve, "GVE %p device descriptor:\n", gve );
	DBGC2_HDA ( gve, 0, desc, sizeof ( *desc ) );

	/* Extract queue parameters */
	gve->events.count = be16_to_cpu ( desc->counters );
	if ( gve->events.count > GVE_EVENT_MAX )
		gve->events.count = GVE_EVENT_MAX;
	gve->tx.count = be16_to_cpu ( desc->tx_count );
	gve->rx.count = be16_to_cpu ( desc->rx_count );
	DBGC ( gve, "GVE %p using %d TX, %d RX, %d/%d events\n",
	       gve, gve->tx.count, gve->rx.count, gve->events.count,
	       be16_to_cpu ( desc->counters ) );

	/* Extract network parameters */
	build_assert ( sizeof ( desc->mac ) == ETH_ALEN );
	memcpy ( netdev->hw_addr, &desc->mac, sizeof ( desc->mac ) );
	netdev->mtu = be16_to_cpu ( desc->mtu );
	netdev->max_pkt_len = ( netdev->mtu + ETH_HLEN );
	DBGC ( gve, "GVE %p MAC %s (\"%s\") MTU %zd\n",
	       gve, eth_ntoa ( netdev->hw_addr ),
	       inet_ntoa ( desc->mac.in ), netdev->mtu );

	return 0;
}

/**
 * Configure device resources
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_configure ( struct gve_nic *gve ) {
	struct gve_events *events = &gve->events;
	struct gve_irqs *irqs = &gve->irqs;
	union gve_admin_command *cmd;
	unsigned int db_off;
	unsigned int i;
	int rc;

	/* Construct request */
	cmd = gve_admin_command ( gve );
	cmd->hdr.opcode = GVE_ADMIN_CONFIGURE;
	cmd->conf.events =
		cpu_to_be64 ( dma ( &events->map, events->event ) );
	cmd->conf.irqs =
		cpu_to_be64 ( dma ( &irqs->map, irqs->irq ) );
	cmd->conf.num_events = cpu_to_be32 ( events->count );
	cmd->conf.num_irqs = cpu_to_be32 ( GVE_IRQ_COUNT );
	cmd->conf.irq_stride = cpu_to_be32 ( sizeof ( irqs->irq[0] ) );

	/* Issue command */
	if ( ( rc = gve_admin ( gve ) ) != 0 )
		return rc;

	/* Disable all interrupts */
	for ( i = 0 ; i < GVE_IRQ_COUNT ; i++ ) {
		db_off = ( be32_to_cpu ( irqs->irq[i].db_idx ) *
			   sizeof ( uint32_t ) );
		DBGC ( gve, "GVE %p IRQ %d doorbell +%#04x\n", gve, i, db_off );
		irqs->db[i] = ( gve->db + db_off );
		writel ( bswap_32 ( GVE_IRQ_DISABLE ), irqs->db[i] );
	}

	return 0;
}

/**
 * Deconfigure device resources
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_deconfigure ( struct gve_nic *gve ) {
	struct gve_events *events = &gve->events;
	int rc;

	/* Issue command (with meaningless ID) */
	if ( ( rc = gve_admin_simple ( gve, GVE_ADMIN_DECONFIGURE,
				       0 ) ) != 0 ) {
		/* Leak memory: there is nothing else we can do */
		events->event = NULL;
		return rc;
	}

	return 0;
}

/**
 * Register queue page list
 *
 * @v gve		GVE device
 * @v qpl		Queue page list
 * @ret rc		Return status code
 */
static int gve_register ( struct gve_nic *gve, struct gve_qpl *qpl ) {
	struct gve_pages *pages = &gve->scratch.buf->pages;
	union gve_admin_command *cmd;
	unsigned int i;
	int rc;

	/* Build page address list */
	for ( i = 0 ; i < qpl->count ; i++ ) {
		pages->addr[i] = cpu_to_be64 ( dma ( &qpl->map[i],
						     qpl->data[i] ) );
	}

	/* Construct request */
	cmd = gve_admin_command ( gve );
	cmd->hdr.opcode = GVE_ADMIN_REGISTER;
	cmd->reg.id = cpu_to_be32 ( qpl->id );
	cmd->reg.count = cpu_to_be32 ( qpl->count );
	cmd->reg.addr = cpu_to_be64 ( dma ( &gve->scratch.map, pages ) );
	cmd->reg.size = cpu_to_be64 ( GVE_PAGE_SIZE );

	/* Issue command */
	if ( ( rc = gve_admin ( gve ) ) != 0 )
		return rc;

	return 0;
}

/**
 * Unregister page list
 *
 * @v gve		GVE device
 * @v qpl		Queue page list
 * @ret rc		Return status code
 */
static int gve_unregister ( struct gve_nic *gve, struct gve_qpl *qpl ) {
	int rc;

	/* Issue command */
	if ( ( rc = gve_admin_simple ( gve, GVE_ADMIN_UNREGISTER,
				       qpl->id ) ) != 0 ) {
		DBGC ( gve, "GVE %p could not free page list: %s\n",
		       gve, strerror ( rc ) );
		/* Leak memory: there is nothing else we can do */
		qpl->data[0] = NULL;
		return rc;
	}

	return 0;
}

/**
 * Construct command to create transmit queue
 *
 * @v queue		Transmit queue
 * @v cmd		Admin queue command
 */
static void gve_create_tx_param ( struct gve_queue *queue,
				  union gve_admin_command *cmd ) {
	struct gve_admin_create_tx *create = &cmd->create_tx;
	const struct gve_queue_type *type = queue->type;

	/* Construct request parameters */
	create->id = cpu_to_be32 ( type->id );
	create->res = cpu_to_be64 ( dma ( &queue->res_map, queue->res ) );
	create->desc =
		cpu_to_be64 ( dma ( &queue->desc_map, queue->desc.raw ) );
	create->qpl_id = cpu_to_be32 ( type->qpl );
	create->notify_id = cpu_to_be32 ( type->irq );
}

/**
 * Construct command to create receive queue
 *
 * @v queue		Receive queue
 * @v cmd		Admin queue command
 */
static void gve_create_rx_param ( struct gve_queue *queue,
				  union gve_admin_command *cmd ) {
	struct gve_admin_create_rx *create = &cmd->create_rx;
	const struct gve_queue_type *type = queue->type;

	/* Construct request parameters */
	create->id = cpu_to_be32 ( type->id );
	create->notify_id = cpu_to_be32 ( type->irq );
	create->res = cpu_to_be64 ( dma ( &queue->res_map, queue->res ) );
	create->desc =
		cpu_to_be64 ( dma ( &queue->desc_map, queue->desc.raw ) );
	create->cmplt =
		cpu_to_be64 ( dma ( &queue->cmplt_map, queue->cmplt.raw ) );
	create->qpl_id = cpu_to_be32 ( type->qpl );
	create->bufsz = cpu_to_be16 ( GVE_BUF_SIZE );
}

/**
 * Create transmit or receive queue
 *
 * @v gve		GVE device
 * @v queue		Descriptor queue
 * @ret rc		Return status code
 */
static int gve_create_queue ( struct gve_nic *gve, struct gve_queue *queue ) {
	const struct gve_queue_type *type = queue->type;
	union gve_admin_command *cmd;
	unsigned int db_off;
	unsigned int evt_idx;
	int rc;

	/* Reset queue */
	queue->prod = 0;
	queue->cons = 0;

	/* Construct request */
	cmd = gve_admin_command ( gve );
	cmd->hdr.opcode = type->create;
	type->param ( queue, cmd );

	/* Issue command */
	if ( ( rc = gve_admin ( gve ) ) != 0 )
		return rc;

	/* Record indices */
	db_off = ( be32_to_cpu ( queue->res->db_idx ) * sizeof ( uint32_t ) );
	evt_idx = be32_to_cpu ( queue->res->evt_idx );
	DBGC ( gve, "GVE %p %s doorbell +%#04x event counter %d\n",
	       gve, type->name, db_off, evt_idx );
	queue->db = ( gve->db + db_off );
	assert ( evt_idx < gve->events.count );
	queue->event = &gve->events.event[evt_idx];
	assert ( queue->event->count == 0 );

	return 0;
}

/**
 * Destroy transmit or receive queue
 *
 * @v gve		GVE device
 * @v queue		Descriptor queue
 * @ret rc		Return status code
 */
static int gve_destroy_queue ( struct gve_nic *gve, struct gve_queue *queue ) {
	const struct gve_queue_type *type = queue->type;
	int rc;

	/* Issue command */
	if ( ( rc = gve_admin_simple ( gve, type->destroy, type->id ) ) != 0 ) {
		/* Leak memory: there is nothing else we can do */
		queue->desc.raw = NULL;
		return rc;
	}

	return 0;
}

/******************************************************************************
 *
 * Network device interface
 *
 ******************************************************************************
 */

/**
 * Allocate queue page list
 *
 * @v gve		GVE device
 * @v qpl		Queue page list
 * @ret rc		Return status code
 */
static int gve_alloc_qpl ( struct gve_nic *gve, struct gve_qpl *qpl ) {
	int i;
	int rc;

	/* Sanity check */
	assert ( qpl->count <= GVE_QPL_MAX );

	/* Allocate pages */
	for ( i = 0 ; i < ( ( int ) qpl->count ) ; i++ ) {
		qpl->data[i] = gve_dma_alloc ( gve, &qpl->map[i],
					       GVE_PAGE_SIZE );
		if ( ! qpl->data[i] ) {
			rc = -ENOMEM;
			goto err_alloc;
		}
		DBGC2 ( gve, "GVE %p QPL %#08x/%#02x at [%08lx,%08lx)\n",
			gve, qpl->id, i, virt_to_phys ( qpl->data[i] ),
			( virt_to_phys ( qpl->data[i] ) + GVE_PAGE_SIZE ) );
	}

	return 0;

 err_alloc:
	for ( i-- ; i >= 0 ; i-- )
		gve_dma_free ( gve, &qpl->map[i], qpl->data[i], GVE_PAGE_SIZE );
	return rc;
}

/**
 * Free queue page list
 *
 * @v gve		GVE device
 * @v qpl		Queue page list
 */
static void gve_free_qpl ( struct gve_nic *gve, struct gve_qpl *qpl ) {
	unsigned int i;

	/* Leak memory if we were unable to unregister the page list */
	if ( ! qpl->data[0] )
		return;

	/* Free page list */
	for ( i = 0 ; i < qpl->count ; i++ )
		gve_dma_free ( gve, &qpl->map[i], qpl->data[i], GVE_PAGE_SIZE );
}

/**
 * Allocate descriptor queue
 *
 * @v gve		GVE device
 * @v queue		Descriptor queue
 * @ret rc		Return status code
 */
static int gve_alloc_queue ( struct gve_nic *gve, struct gve_queue *queue ) {
	const struct gve_queue_type *type = queue->type;
	size_t desc_len = ( queue->count * type->desc_len );
	size_t cmplt_len = ( queue->count * type->cmplt_len );
	size_t res_len = sizeof ( *queue->res );
	uint64_t *offset;
	unsigned int i;
	int rc;

	/* Sanity checks */
	if ( ( queue->count == 0 ) ||
	     ( queue->count & ( queue->count - 1 ) ) ) {
		DBGC ( gve, "GVE %p %s invalid queue size %d\n",
		       gve, type->name, queue->count );
		rc = -EINVAL;
		goto err_sanity;
	}

	/* Calculate maximum fill level */
	assert ( ( type->fill & ( type->fill - 1 ) ) == 0 );
	queue->fill = type->fill;
	if ( queue->fill > queue->count )
		queue->fill = queue->count;
	DBGC ( gve, "GVE %p %s using QPL %#08x with %d/%d descriptors\n",
	       gve, type->name, type->qpl, queue->fill, queue->count );

	/* Allocate queue page list */
	build_assert ( GVE_BUF_SIZE <= GVE_PAGE_SIZE );
	queue->qpl.id = type->qpl;
	queue->qpl.count = ( ( queue->fill + GVE_BUF_PER_PAGE - 1 ) /
			     GVE_BUF_PER_PAGE );
	if ( ( rc = gve_alloc_qpl ( gve, &queue->qpl ) ) != 0 )
		goto err_qpl;

	/* Allocate descriptors */
	queue->desc.raw = gve_dma_alloc ( gve, &queue->desc_map, desc_len );
	if ( ! queue->desc.raw ) {
		rc = -ENOMEM;
		goto err_desc;
	}
	memset ( queue->desc.raw, 0, desc_len );
	DBGC ( gve, "GVE %p %s descriptors at [%08lx,%08lx)\n",
	       gve, type->name, virt_to_phys ( queue->desc.raw ),
	       ( virt_to_phys ( queue->desc.raw ) + desc_len ) );

	/* Allocate completions */
	if ( cmplt_len ) {
		queue->cmplt.raw = gve_dma_alloc ( gve, &queue->cmplt_map,
						   cmplt_len );
		if ( ! queue->cmplt.raw ) {
			rc = -ENOMEM;
			goto err_cmplt;
		}
		memset ( queue->cmplt.raw, 0, cmplt_len );
		DBGC ( gve, "GVE %p %s completions at [%08lx,%08lx)\n",
		       gve, type->name, virt_to_phys ( queue->cmplt.raw ),
		       ( virt_to_phys ( queue->cmplt.raw ) + cmplt_len ) );
	}

	/* Allocate queue resources */
	queue->res = gve_dma_alloc ( gve, &queue->res_map, res_len );
	if ( ! queue->res ) {
		rc = -ENOMEM;
		goto err_res;
	}
	memset ( queue->res, 0, res_len );

	/* Populate descriptor offsets */
	offset = ( queue->desc.raw + type->desc_len - sizeof ( *offset ) );
	for ( i = 0 ; i < queue->count ; i++ ) {
		*offset = cpu_to_be64 ( ( i & ( queue->fill - 1 ) ) *
					GVE_BUF_SIZE );
		offset = ( ( ( void * ) offset ) + type->desc_len );
	}

	return 0;

	gve_dma_free ( gve, &queue->res_map, queue->res, res_len );
 err_res:
	if ( cmplt_len ) {
		gve_dma_free ( gve, &queue->cmplt_map, queue->cmplt.raw,
			       cmplt_len );
	}
 err_cmplt:
	gve_dma_free ( gve, &queue->desc_map, queue->desc.raw, desc_len );
 err_desc:
	gve_free_qpl ( gve, &queue->qpl );
 err_qpl:
 err_sanity:
	return rc;
}

/**
 * Free descriptor queue
 *
 * @v gve		GVE device
 * @v queue		Descriptor queue
 */
static void gve_free_queue ( struct gve_nic *gve, struct gve_queue *queue ) {
	const struct gve_queue_type *type = queue->type;
	size_t desc_len = ( queue->count * type->desc_len );
	size_t cmplt_len = ( queue->count * type->cmplt_len );
	size_t res_len = sizeof ( *queue->res );

	/* Leak memory if we were unable to destroy the queue */
	if ( ! queue->desc.raw )
		return;

	/* Free queue resources */
	gve_dma_free ( gve, &queue->res_map, queue->res, res_len );

	/* Free completions, if applicable */
	if ( cmplt_len ) {
		gve_dma_free ( gve, &queue->cmplt_map, queue->cmplt.raw,
			       cmplt_len );
	}

	/* Free descriptors */
	gve_dma_free ( gve, &queue->desc_map, queue->desc.raw, desc_len );

	/* Free queue page list */
	gve_free_qpl ( gve, &queue->qpl );
}

/**
 * Get buffer address
 *
 * @v queue		Descriptor queue
 * @v index		Buffer index
 * @ret data		Buffer pointer
 */
static inline void * gve_buffer ( struct gve_queue *queue,
				  unsigned int index ) {
	unsigned int page;
	unsigned int subpage;

	/* Sanity check */
	assert ( ( queue->fill & ( queue->fill - 1 ) ) == 0 );

	/* Get address of the relevant buffer within the relevant page */
	index &= ( queue->fill - 1 );
	page = ( index / GVE_BUF_PER_PAGE );
	subpage = ( index & ( GVE_BUF_PER_PAGE - 1 ) );
	return ( queue->qpl.data[page] + ( subpage * GVE_BUF_SIZE ) );
}

/**
 * Refill receive queue
 *
 * @v netdev		Network device
 */
static void gve_refill_rx ( struct net_device *netdev ) {
	struct gve_nic *gve = netdev->priv;
	struct gve_queue *rx = &gve->rx;
	unsigned int prod;

	/* The receive descriptors are prepopulated at the time of
	 * creating the receive queue (pointing to the preallocated
	 * queue pages).  Refilling is therefore just a case of
	 * ringing the doorbell if the device is not yet aware of any
	 * available descriptors.
	 */
	prod = ( rx->cons + rx->fill );
	if ( prod != rx->prod ) {
		rx->prod = prod;
		writel ( bswap_32 ( prod ), rx->db );
	}
}

/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int gve_open ( struct net_device *netdev ) {
	struct gve_nic *gve = netdev->priv;
	struct gve_queue *tx = &gve->tx;
	struct gve_queue *rx = &gve->rx;
	int rc;

	/* Reset transmit I/O buffers */
	memset ( gve->tx_iobuf, 0, sizeof ( gve->tx_iobuf ) );

	/* Allocate and populate transmit queue */
	if ( ( rc = gve_alloc_queue ( gve, tx ) ) != 0 )
		goto err_alloc_tx;

	/* Allocate and populate receive queue */
	if ( ( rc = gve_alloc_queue ( gve, rx ) ) != 0 )
		goto err_alloc_rx;

	/* Register transmit queue page list */
	if ( ( rc = gve_register ( gve, &tx->qpl ) ) != 0 )
		goto err_register_tx;

	/* Register receive queue page list */
	if ( ( rc = gve_register ( gve, &rx->qpl ) ) != 0 )
		goto err_register_rx;

	/* Create transmit queue */
	if ( ( rc = gve_create_queue ( gve, tx ) ) != 0 )
		goto err_create_tx;

	/* Create receive queue */
	if ( ( rc = gve_create_queue ( gve, rx ) ) != 0 )
		goto err_create_rx;

	return 0;

	gve_destroy_queue ( gve, rx );
 err_create_rx:
	gve_destroy_queue ( gve, tx );
 err_create_tx:
	gve_unregister ( gve, &rx->qpl );
 err_register_rx:
	gve_unregister ( gve, &tx->qpl );
 err_register_tx:
	gve_free_queue ( gve, rx );
 err_alloc_rx:
	gve_free_queue ( gve, tx );
 err_alloc_tx:
	return rc;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void gve_close ( struct net_device *netdev ) {
	struct gve_nic *gve = netdev->priv;
	struct gve_queue *tx = &gve->tx;
	struct gve_queue *rx = &gve->rx;

	/* Destroy queues */
	gve_destroy_queue ( gve, rx );
	gve_destroy_queue ( gve, tx );

	/* Unregister page lists */
	gve_unregister ( gve, &rx->qpl );
	gve_unregister ( gve, &tx->qpl );

	/* Free queues */
	gve_free_queue ( gve, rx );
	gve_free_queue ( gve, tx );
}

/**
 * Transmit packet
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int gve_transmit ( struct net_device *netdev, struct io_buffer *iobuf ) {
	struct gve_nic *gve = netdev->priv;
	struct gve_queue *tx = &gve->tx;
	struct gve_tx_descriptor *desc;
	unsigned int count;
	unsigned int index;
	size_t frag_len;
	size_t offset;
	size_t len;

	/* Defer packet if there is no space in the transmit ring */
	len = iob_len ( iobuf );
	count = ( ( len + GVE_BUF_SIZE - 1 ) / GVE_BUF_SIZE );
	if ( ( ( tx->prod - tx->cons ) + count ) > tx->fill ) {
		netdev_tx_defer ( netdev, iobuf );
		return 0;
	}

	/* Copy packet to queue pages and populate descriptors */
	for ( offset = 0 ; offset < len ; offset += frag_len ) {

		/* Sanity check */
		assert ( gve->tx_iobuf[ tx->prod % GVE_TX_FILL ] == NULL );

		/* Copy packet fragment */
		frag_len = ( len - offset );
		if ( frag_len > GVE_BUF_SIZE )
			frag_len = GVE_BUF_SIZE;
		memcpy ( gve_buffer ( tx, tx->prod ), ( iobuf->data + offset ),
			 frag_len );

		/* Populate descriptor */
		index = ( tx->prod++ & ( tx->count - 1 ) );
		desc = &tx->desc.tx[index];
		if ( offset ) {
			desc->type = GVE_TX_TYPE_CONT;
			desc->count = 0;
			desc->total = 0;
		} else {
			desc->type = GVE_TX_TYPE_START;
			desc->count = count;
			desc->total = cpu_to_be16 ( len );
		}
		desc->len = cpu_to_be16 ( frag_len );
		DBGC2 ( gve, "GVE %p TX %#04x %02x:%02x:%04x:%04x:%08llx\n",
			gve, index, desc->type, desc->count,
			be16_to_cpu ( desc->total ), be16_to_cpu ( desc->len ),
			( ( unsigned long long )
			  be64_to_cpu ( desc->offset ) ) );
	}
	assert ( ( tx->prod - tx->cons ) <= tx->fill );

	/* Record I/O buffer against final descriptor */
	gve->tx_iobuf[ ( tx->prod - 1U ) % GVE_TX_FILL ] = iobuf;

	/* Ring doorbell */
	wmb();
	writel ( bswap_32 ( tx->prod ), tx->db );

	return 0;
}

/**
 * Poll for completed transmissions
 *
 * @v netdev		Network device
 */
static void gve_poll_tx ( struct net_device *netdev ) {
	struct gve_nic *gve = netdev->priv;
	struct gve_queue *tx = &gve->tx;
	struct io_buffer *iobuf;
	uint32_t count;

	/* Read event counter */
	count = be32_to_cpu ( tx->event->count );

	/* Process transmit completions */
	while ( count != tx->cons ) {
		DBGC2 ( gve, "GVE %p TX %#04x complete\n", gve, tx->cons );
		iobuf = gve->tx_iobuf[ tx->cons % GVE_TX_FILL ];
		gve->tx_iobuf[ tx->cons % GVE_TX_FILL ] = NULL;
		tx->cons++;
		if ( iobuf )
			netdev_tx_complete ( netdev, iobuf );
	}
}

/**
 * Poll for received packets
 *
 * @v netdev		Network device
 */
static void gve_poll_rx ( struct net_device *netdev ) {
	struct gve_nic *gve = netdev->priv;
	struct gve_queue *rx = &gve->rx;
	struct gve_rx_completion *cmplt;

	//
	cmplt = &rx->cmplt.rx[0];
	if ( cmplt->seq ) {
		DBGC ( gve, "***** RX\n" );
		DBGC_HDA ( gve, 0, cmplt, sizeof ( *cmplt ) );
	}
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void gve_poll ( struct net_device *netdev ) {

	/* Poll for transmit completions */
	gve_poll_tx ( netdev );

	/* Poll for receive completions */
	gve_poll_rx ( netdev );

	/* Refill receive queue */
	gve_refill_rx ( netdev );
}

/** GVE network device operations */
static struct net_device_operations gve_operations = {
	.open		= gve_open,
	.close		= gve_close,
	.transmit	= gve_transmit,
	.poll		= gve_poll,
};

/******************************************************************************
 *
 * PCI interface
 *
 ******************************************************************************
 */

/** Transmit descriptor queue type */
static const struct gve_queue_type gve_tx_type = {
	.name = "TX",
	.param = gve_create_tx_param,
	.qpl = GVE_TX_QPL,
	.id = GVE_TX_ID,
	.irq = GVE_TX_IRQ,
	.fill = GVE_TX_FILL,
	.desc_len = sizeof ( struct gve_tx_descriptor ),
	.create = GVE_ADMIN_CREATE_TX,
	.destroy = GVE_ADMIN_DESTROY_TX,
};

/** Receive descriptor queue type */
static const struct gve_queue_type gve_rx_type = {
	.name = "RX",
	.param = gve_create_rx_param,
	.qpl = GVE_RX_QPL,
	.id = GVE_RX_ID,
	.irq = GVE_RX_IRQ,
	.fill = GVE_RX_FILL,
	.desc_len = sizeof ( struct gve_rx_descriptor ),
	.cmplt_len = sizeof ( struct gve_rx_completion ),
	.create = GVE_ADMIN_CREATE_RX,
	.destroy = GVE_ADMIN_DESTROY_RX,
};

/**
 * Probe PCI device
 *
 * @v pci		PCI device
 * @ret rc		Return status code
 */
static int gve_probe ( struct pci_device *pci ) {
	struct net_device *netdev;
	struct gve_nic *gve;
	unsigned long cfg_start;
	unsigned long db_start;
	unsigned long db_size;
	int rc;

	/* Allocate and initialise net device */
	netdev = alloc_etherdev ( sizeof ( *gve ) );
	if ( ! netdev ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	netdev_init ( netdev, &gve_operations );
	gve = netdev->priv;
	pci_set_drvdata ( pci, netdev );
	netdev->dev = &pci->dev;
	memset ( gve, 0, sizeof ( *gve ) );
	gve->tx.type = &gve_tx_type;
	gve->rx.type = &gve_rx_type;

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Check PCI revision */
	pci_read_config_byte ( pci, PCI_REVISION, &gve->revision );
	DBGC ( gve, "GVE %p is revision %#02x\n", gve, gve->revision );

	/* Map configuration registers */
	cfg_start = pci_bar_start ( pci, GVE_CFG_BAR );
	gve->cfg = pci_ioremap ( pci, cfg_start, GVE_CFG_SIZE );
	if ( ! gve->cfg ) {
		rc = -ENODEV;
		goto err_cfg;
	}

	/* Map doorbell registers */
	db_start = pci_bar_start ( pci, GVE_DB_BAR );
	db_size = pci_bar_size ( pci, GVE_DB_BAR );
	gve->db = pci_ioremap ( pci, db_start, db_size );
	if ( ! gve->db ) {
		rc = -ENODEV;
		goto err_db;
	}

	/* Configure DMA */
	gve->dma = &pci->dma;
	dma_set_mask_64bit ( gve->dma );
	assert ( netdev->dma == NULL );

	/* Reset the NIC */
	if ( ( rc = gve_reset ( gve ) ) != 0 )
		goto err_reset;

	/* Allocate admin queue */
	if ( ( rc = gve_admin_alloc ( gve ) ) != 0 )
		goto err_admin;

	/* Enable admin queue */
	gve_admin_enable ( gve );

	/* Fetch MAC address */
	if ( ( rc = gve_describe ( netdev ) ) != 0 )
		goto err_describe;

	/* Configure device resources */
	if ( ( rc = gve_configure ( gve ) ) != 0 )
		goto err_configure;

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Mark as link up, since we have no way to test link state on
	 * this hardware.
	 */
	netdev_link_up ( netdev );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
	gve_deconfigure ( gve );
 err_configure:
 err_describe:
	gve_admin_disable ( gve );
	gve_admin_free ( gve );
 err_admin:
 err_reset:
	iounmap ( gve->db );
 err_db:
	iounmap ( gve->cfg );
 err_cfg:
	netdev_nullify ( netdev );
	netdev_put ( netdev );
 err_alloc:
	return rc;
}

/**
 * Remove PCI device
 *
 * @v pci		PCI device
 */
static void gve_remove ( struct pci_device *pci ) {
	struct net_device *netdev = pci_get_drvdata ( pci );
	struct gve_nic *gve = netdev->priv;

	/* Unregister network device */
	unregister_netdev ( netdev );

	/* Deconfigure device resources */
	gve_deconfigure ( gve );

	/* Disable admin queue and reset device */
	gve_admin_disable ( gve );

	/* Free admin queue */
	gve_admin_free ( gve );

	/* Unmap registers */
	iounmap ( gve->db );
	iounmap ( gve->cfg );

	/* Free network device */
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** GVE PCI device IDs */
static struct pci_device_id gve_nics[] = {
	PCI_ROM ( 0x1ae0, 0x0042, "gve", "gVNIC", 0 ),
};

/** GVE PCI driver */
struct pci_driver gve_driver __pci_driver = {
	.ids = gve_nics,
	.id_count = ( sizeof ( gve_nics ) / sizeof ( gve_nics[0] ) ),
	.probe = gve_probe,
	.remove = gve_remove,
};
