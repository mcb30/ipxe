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
	struct gve_events *events = &gve->events;
	struct gve_scratch *scratch = &gve->scratch;
	size_t admin_len = ( GVE_ADMIN_COUNT * sizeof ( admin->cmd[0] ) );
	size_t events_len = ( GVE_EVENT_MAX * sizeof ( events->event[0] ) );
	size_t scratch_len = sizeof ( *scratch->buf );
	int rc;

	/* Allocate admin queue */
	admin->cmd = gve_dma_alloc ( gve, &admin->map, admin_len );
	if ( ! admin->cmd ) {
		rc = -ENOMEM;
		goto err_admin;
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
	struct gve_events *events = &gve->events;
	struct gve_scratch *scratch = &gve->scratch;
	size_t admin_len = ( GVE_ADMIN_COUNT * sizeof ( admin->cmd[0] ) );
	size_t events_len = ( GVE_EVENT_MAX * sizeof ( events->event[0] ) );
	size_t scratch_len = sizeof ( *scratch->buf );

	/* Leak memory if we were unable to reset the device */
	if ( ! ( admin->cmd && events->event ) )
		return;

	/* Free scratch buffer */
	gve_dma_free ( gve, &scratch->map, scratch->buf, scratch_len );

	/* Free event counter */
	gve_dma_free ( gve, &events->map, events->event, events_len );

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
	opcode = be32_to_cpu ( cmd->hdr.opcode );
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
 * @ret rc		Return status code
 *
 * Several admin queue commands take either an empty parameter list or
 * a single 32-bit ID parameter which we may choose to always be zero.
 */
static int gve_admin_simple ( struct gve_nic *gve, unsigned int opcode ) {
	union gve_admin_command *cmd;
	int rc;

	/* Construct request */
	cmd = gve_admin_command ( gve );
	cmd->hdr.opcode = cpu_to_be32 ( opcode );

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
	cmd->hdr.opcode = cpu_to_be32 ( GVE_ADMIN_DESCRIBE );
	cmd->desc.addr = cpu_to_be64 ( dma ( &gve->scratch.map, desc ) );
	cmd->desc.ver = cpu_to_be32 ( GVE_ADMIN_DESCRIBE_VER );
	cmd->desc.len = cpu_to_be32 ( sizeof ( *desc ) );

	/* Issue command */
	if ( ( rc = gve_admin ( gve ) ) != 0 )
		return rc;
	DBGC2 ( gve, "GVE %p device descriptor:\n", gve );
	DBGC2_HDA ( gve, 0, desc, sizeof ( *desc ) );

	/* Extract parameters */
	gve->events.count = be16_to_cpu ( desc->counters );
	if ( gve->events.count > GVE_EVENT_MAX )
		gve->events.count = GVE_EVENT_MAX;
	DBGC ( gve, "GVE %p using %d/%d event counters\n",
	       gve, gve->events.count, be16_to_cpu ( desc->counters ) );
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
	union gve_admin_command *cmd;
	int rc;

	/* Construct request */
	cmd = gve_admin_command ( gve );
	cmd->hdr.opcode = cpu_to_be32 ( GVE_ADMIN_CONFIGURE );
	cmd->conf.counters =
		cpu_to_be64 ( dma ( &events->map, events->event ) );
	cmd->conf.num_counters = cpu_to_be32 ( events->count );

	/* Issue command */
	if ( ( rc = gve_admin ( gve ) ) != 0 )
		return rc;

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
	if ( ( rc = gve_admin_simple ( gve, GVE_ADMIN_DECONFIGURE ) ) != 0 ) {
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
 * @ret rc		Return status code
 */
static int gve_register ( struct gve_nic *gve ) {
	struct gve_qpl *qpl = &gve->qpl;
	struct gve_pages *pages = &gve->scratch.buf->pages;
	union gve_admin_command *cmd;
	unsigned int i;
	int rc;

	/* Reset ring buffer and build page address table */
	for ( i = 0 ; i < GVE_QPL_COUNT ; i++ ) {
		qpl->ids[i] = i;
		pages->addr[i] = cpu_to_be64 ( dma ( &qpl->map[i],
						     qpl->data[i] ) );
	}
	qpl->prod = 0;
	qpl->cons = 0;

	/* Construct request */
	cmd = gve_admin_command ( gve );
	cmd->hdr.opcode = cpu_to_be32 ( GVE_ADMIN_REGISTER );
	cmd->reg.count = cpu_to_be32 ( GVE_QPL_COUNT );
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
 * @ret rc		Return status code
 */
static int gve_unregister ( struct gve_nic *gve ) {
	struct gve_qpl *qpl = &gve->qpl;
	int rc;

	/* Issue command */
	if ( ( rc = gve_admin_simple ( gve, GVE_ADMIN_UNREGISTER ) ) != 0 ) {
		DBGC ( gve, "GVE %p could not free page list: %s\n",
		       gve, strerror ( rc ) );
		/* Leak memory: there is nothing else we can do */
		qpl->data[0] = NULL;
		return rc;
	}

	return 0;
}

/**
 * Create transmit queue
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_create_tx ( struct gve_nic *gve ) {
	struct gve_tx *tx = &gve->tx;
	union gve_admin_command *cmd;
	unsigned int db_off;
	unsigned int evt_idx;
	int rc;

	/* Construct request */
	cmd = gve_admin_command ( gve );
	cmd->hdr.opcode = cpu_to_be32 ( GVE_ADMIN_CREATE_TX );
	cmd->create_tx.res = cpu_to_be64 ( dma ( &tx->res_map, tx->res ) );
	cmd->create_tx.desc = cpu_to_be64 ( dma ( &tx->desc_map, tx->desc ) );

	/* Issue command */
	if ( ( rc = gve_admin ( gve ) ) != 0 )
		return rc;

	/* Record indices */
	db_off = ( be32_to_cpu ( tx->res->db_idx ) * sizeof ( uint32_t ) );
	evt_idx = be32_to_cpu ( tx->res->evt_idx );
	DBGC ( gve, "GVE %p TX doorbell +%#x event counter %d\n",
	       gve, db_off, evt_idx );
	tx->doorbell = ( gve->db + db_off );
	assert ( evt_idx < gve->events.count );
	tx->event = &gve->events.event[evt_idx];
	assert ( tx->event->count == 0 );

	return 0;
}

/**
 * Destroy transmit queue
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_destroy_tx ( struct gve_nic *gve ) {
	struct gve_tx *tx = &gve->tx;
	int rc;

	/* Issue command */
	if ( ( rc = gve_admin_simple ( gve, GVE_ADMIN_DESTROY_TX ) ) != 0 ) {
		/* Leak memory: there is nothing else we can do */
		tx->desc = NULL;
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
 * @ret rc		Return status code
 */
static int gve_qpl_alloc ( struct gve_nic *gve ) {
	struct gve_qpl *qpl = &gve->qpl;
	int i;
	int rc;

	/* Allocate pages */
	for ( i = 0 ; i < GVE_QPL_COUNT ; i++ ) {
		qpl->data[i] = gve_dma_alloc ( gve, &qpl->map[i],
					       GVE_PAGE_SIZE );
		if ( ! qpl->data[i] ) {
			rc = -ENOMEM;
			goto err_alloc;
		}
		DBGC2 ( gve, "GVE %p page %#02x at [%08lx,%08lx)\n",
			gve, i, virt_to_phys ( qpl->data[i] ),
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
 */
static void gve_qpl_free ( struct gve_nic *gve ) {
	struct gve_qpl *qpl = &gve->qpl;
	unsigned int i;

	/* Leak memory if we were unable to unregister the page list */
	if ( ! qpl->data[0] )
		return;

	/* Free page list */
	for ( i = 0 ; i < GVE_QPL_COUNT ; i++ )
		gve_dma_free ( gve, &qpl->map[i], qpl->data[i], GVE_PAGE_SIZE );
}

/**
 * Acquire queue page from list
 *
 * @v gve		GVE device
 * @ret id		Queue page ID, or negative error
 */
static int gve_qpl_get ( struct gve_nic *gve ) {
	struct gve_qpl *qpl = &gve->qpl;

	/* Check for an available page */
	if ( ( qpl->prod - qpl->cons ) >= GVE_QPL_COUNT )
		return -ENOBUFS;

	/* Return next available ID */
	return qpl->ids[ qpl->prod++ % GVE_QPL_COUNT ];
}

/**
 * Release queue page back to list
 *
 * @v gve		GVE device
 * @v id		Queue page ID
 */
static void gve_qpl_put ( struct gve_nic *gve, int id ) {
	struct gve_qpl *qpl = &gve->qpl;

	/* Sanity checks */
	assert ( qpl->prod != qpl->cons );
	assert ( ( qpl->prod - qpl->cons ) <= GVE_QPL_COUNT );

	/* Return ID to list */
	qpl->ids[ qpl->cons++ % GVE_QPL_COUNT ] = id;
}

/**
 * Allocate transmit queue
 *
 * @v gve		GVE device
 * @ret rc		Return status code
 */
static int gve_tx_alloc ( struct gve_nic *gve ) {
	struct gve_tx *tx = &gve->tx;
	size_t len = ( sizeof ( tx->desc[0] ) * GVE_TX_COUNT );
	int rc;

	/* Sanity check: ensure TX ring cannot fill up */
	build_assert ( GVE_TX_COUNT > GVE_QPL_COUNT );

	/* Allocate transmit descriptors */
	tx->desc = gve_dma_alloc ( gve, &tx->desc_map, len );
	if ( ! tx->desc ) {
		rc = -ENOMEM;
		goto err_desc;
	}
	memset ( tx->desc, 0, len );

	/* Allocate queue resources */
	tx->res = gve_dma_alloc ( gve, &tx->res_map, sizeof ( *tx->res ) );
	if ( ! tx->desc ) {
		rc = -ENOMEM;
		goto err_res;
	}
	memset ( tx->res, 0, sizeof ( *tx->res ) );

	return 0;

	gve_dma_free ( gve, &tx->res_map, tx->res, sizeof ( *tx->res ) );
 err_res:
	gve_dma_free ( gve, &tx->desc_map, tx->desc, len );
 err_desc:
	return rc;
}

/**
 * Free transmit queue
 *
 * @v gve		GVE device
 */
static void gve_tx_free ( struct gve_nic *gve ) {
	struct gve_tx *tx = &gve->tx;
	size_t len = ( sizeof ( tx->desc[0] ) * GVE_TX_COUNT );

	/* Leak memory if we were unable to destroy the transmit queue */
	if ( ! tx->desc )
		return;

	/* Free queue resources */
	gve_dma_free ( gve, &tx->res_map, tx->res, sizeof ( *tx->res ) );

	/* Free transmit descriptors */
	gve_dma_free ( gve, &tx->desc_map, tx->desc, len );
}

/**
 * Refill receive queue
 *
 * @v netdev		Network device
 */
static void gve_refill_rx ( struct net_device *netdev ) {

	//
	( void ) netdev;
}

/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int gve_open ( struct net_device *netdev ) {
	struct gve_nic *gve = netdev->priv;
	int rc;

	/* Allocate queue page list */
	if ( ( rc = gve_qpl_alloc ( gve ) ) != 0 )
		goto err_qpl;

	/* Allocate transmit queue */
	if ( ( rc = gve_tx_alloc ( gve ) ) != 0 )
		goto err_tx;

	/* Register page list */
	if ( ( rc = gve_register ( gve ) ) != 0 )
		goto err_register;

	/* Create transmit queue */
	if ( ( rc = gve_create_tx ( gve ) ) != 0 )
		goto err_create_tx;

	return 0;

	gve_destroy_tx ( gve );
 err_create_tx:
	gve_unregister ( gve );
 err_register:
	gve_tx_free ( gve );
 err_tx:
	gve_qpl_free ( gve );
 err_qpl:
	return rc;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void gve_close ( struct net_device *netdev ) {
	struct gve_nic *gve = netdev->priv;

	/* Destroy transmit queue */
	gve_destroy_tx ( gve );

	/* Unregister page list */
	gve_unregister ( gve );

	/* Free transmit queue */
	gve_tx_free ( gve );

	/* Free queue page list */
	gve_qpl_free ( gve );
}

/**
 * Transmit packet
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int gve_transmit ( struct net_device *netdev, struct io_buffer *iobuf ) {

	//
	( void ) netdev;
	( void ) iobuf;
	( void ) gve_qpl_get;
	( void ) gve_qpl_put;
	return -ENOTSUP;
}

/**
 * Poll for completed transmissions
 *
 * @v netdev		Network device
 */
static void gve_poll_tx ( struct net_device *netdev ) {

	//
	( void ) netdev;
}

/**
 * Poll for received packets
 *
 * @v netdev		Network device
 */
static void gve_poll_rx ( struct net_device *netdev ) {

	//
	( void ) netdev;
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

	/* Refill receive ring */
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
