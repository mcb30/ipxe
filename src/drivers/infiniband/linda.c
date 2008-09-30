/*
 * Copyright (C) 2008 Michael Brown <mbrown@fensystems.co.uk>.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <gpxe/pci.h>
#include <gpxe/infiniband.h>
#include "linda.h"

/**
 * @file
 *
 * QLogic Linda Infiniband HCA
 *
 */


static void linda_readq ( struct linda *linda, unsigned long offset,
			  uint32_t *dwords ) {
	dwords[0] = readl ( linda->regs + offset + 0 );
	dwords[1] = readl ( linda->regs + offset + 4 );
}
#define linda_readq( _linda, _offset, _ptr ) \
	linda_readq ( (_linda), (_offset), (_ptr)->u.dwords )

/***************************************************************************
 *
 * Completion queue operations
 *
 ***************************************************************************
 */

/**
 * Create completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 * @ret rc		Return status code
 */
static int linda_create_cq ( struct ib_device *ibdev,
			     struct ib_completion_queue *cq ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) cq;
	return -ENOTSUP;
}

/**
 * Destroy completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 */
static void linda_destroy_cq ( struct ib_device *ibdev,
			       struct ib_completion_queue *cq ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) cq;
}

/***************************************************************************
 *
 * Queue pair operations
 *
 ***************************************************************************
 */

/**
 * Create queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @ret rc		Return status code
 */
static int linda_create_qp ( struct ib_device *ibdev,
			     struct ib_queue_pair *qp ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) qp;
	return -ENOTSUP;
}

/**
 * Modify queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v mod_list		Modification list
 * @ret rc		Return status code
 */
static int linda_modify_qp ( struct ib_device *ibdev,
			     struct ib_queue_pair *qp,
			     unsigned long mod_list ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) qp;
	( void ) mod_list;
	return -ENOTSUP;
}

/**
 * Destroy queue pair
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 */
static void linda_destroy_qp ( struct ib_device *ibdev,
			       struct ib_queue_pair *qp ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) qp;
}

/***************************************************************************
 *
 * Work request operations
 *
 ***************************************************************************
 */

/**
 * Post send work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v av		Address vector
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int linda_post_send ( struct ib_device *ibdev,
			     struct ib_queue_pair *qp,
			     struct ib_address_vector *av,
			     struct io_buffer *iobuf ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) qp;
	( void ) av;
	( void ) iobuf;
	return -ENOTSUP;
}

/**
 * Post receive work queue entry
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int linda_post_recv ( struct ib_device *ibdev,
			     struct ib_queue_pair *qp,
			     struct io_buffer *iobuf ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) qp;
	( void ) iobuf;
	return -ENOTSUP;
}

/**
 * Poll completion queue
 *
 * @v ibdev		Infiniband device
 * @v cq		Completion queue
 * @v complete_send	Send completion handler
 * @v complete_recv	Receive completion handler
 */
static void linda_poll_cq ( struct ib_device *ibdev,
			    struct ib_completion_queue *cq,
			    ib_completer_t complete_send,
			    ib_completer_t complete_recv ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) cq;
	( void ) complete_send;
	( void ) complete_recv;
}

/***************************************************************************
 *
 * Event queues
 *
 ***************************************************************************
 */

/**
 * Poll event queue
 *
 * @v ibdev		Infiniband device
 */
static void linda_poll_eq ( struct ib_device *ibdev ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
}

/***************************************************************************
 *
 * Infiniband link-layer operations
 *
 ***************************************************************************
 */

/**
 * Initialise Infiniband link
 *
 * @v ibdev		Infiniband device
 * @ret rc		Return status code
 */
static int linda_open ( struct ib_device *ibdev ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	return -ENOTSUP;
}

/**
 * Close Infiniband link
 *
 * @v ibdev		Infiniband device
 */
static void linda_close ( struct ib_device *ibdev ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
}

/***************************************************************************
 *
 * Multicast group operations
 *
 ***************************************************************************
 */

/**
 * Attach to multicast group
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v gid		Multicast GID
 * @ret rc		Return status code
 */
static int linda_mcast_attach ( struct ib_device *ibdev,
				struct ib_queue_pair *qp,
				struct ib_gid *gid ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) qp;
	( void ) gid;
	return -ENOTSUP;
}

/**
 * Detach from multicast group
 *
 * @v ibdev		Infiniband device
 * @v qp		Queue pair
 * @v gid		Multicast GID
 */
static void linda_mcast_detach ( struct ib_device *ibdev,
				 struct ib_queue_pair *qp,
				 struct ib_gid *gid ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) qp;
	( void ) gid;
}

/***************************************************************************
 *
 * MAD operations
 *
 ***************************************************************************
 */

/**
 * Issue management datagram
 *
 * @v ibdev		Infiniband device
 * @v mad		Management datagram
 * @v len		Length of management datagram
 * @ret rc		Return status code
 */
static int linda_mad ( struct ib_device *ibdev, struct ib_mad_hdr *mad,
		       size_t len ) {
	struct linda *linda = ib_get_drvdata ( ibdev );

	( void ) linda;
	( void ) mad;
	( void ) len;
	return -ENOTSUP;
}

/** Linda Infiniband operations */
static struct ib_device_operations linda_ib_operations = {
	.create_cq	= linda_create_cq,
	.destroy_cq	= linda_destroy_cq,
	.create_qp	= linda_create_qp,
	.modify_qp	= linda_modify_qp,
	.destroy_qp	= linda_destroy_qp,
	.post_send	= linda_post_send,
	.post_recv	= linda_post_recv,
	.poll_cq	= linda_poll_cq,
	.poll_eq	= linda_poll_eq,
	.open		= linda_open,
	.close		= linda_close,
	.mcast_attach	= linda_mcast_attach,
	.mcast_detach	= linda_mcast_detach,
	.mad		= linda_mad,
};

/**
 * Probe PCI device
 *
 * @v pci		PCI device
 * @v id		PCI ID
 * @ret rc		Return status code
 */
static int linda_probe ( struct pci_device *pci,
			 const struct pci_device_id *id __unused ) {
	struct linda *linda;
	struct ib_device *ibdev;
	struct QIB_7220_Revision revision;
	int i;
	int rc;

	/* Allocate Linda device */
	linda = zalloc ( sizeof ( *linda ) );
	if ( ! linda ) {
		rc = -ENOMEM;
		goto err_alloc_linda;
	}
	pci_set_drvdata ( pci, linda );

	/* Allocate Infiniband devices */
	for ( i = 0 ; i < LINDA_NUM_PORTS ; i++ ) {
		ibdev = alloc_ibdev ( 0 );
		if ( ! ibdev ) {
			rc = -ENOMEM;
			goto err_alloc_ibdev;
		}
		linda->ibdev[i] = ibdev;
		ibdev->op = &linda_ib_operations;
		ibdev->dev = &pci->dev;
		ibdev->port = ( LINDA_PORT_BASE + i );
		ib_set_drvdata ( ibdev, linda );
	}

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Get PCI BARs */
	linda->regs = ioremap ( pci->membase, LINDA_BAR0_SIZE );
	DBGC ( linda, "Linda %p has BAR at %08lx\n", linda, pci->membase );

	/* Print some general data */
	linda_readq ( linda, QIB_7220_Revision_offset, &revision );
	DBGC ( linda, "Linda %p is version %ld.%ld\n", linda,
	       BIT_GET ( &revision, R_ChipRevMajor ),
	       BIT_GET ( &revision, R_ChipRevMinor ) );

	/* Register Infiniband devices */
	for ( i = 0 ; i < LINDA_NUM_PORTS ; i++ ) {
		if ( ( rc = register_ibdev ( linda->ibdev[i] ) ) != 0 ) {
			DBGC ( linda, "Linda %p could not register IB "
			       "device: %s\n", linda, strerror ( rc ) );
			goto err_register_ibdev;
		}
	}
	
	return 0;

	i = LINDA_NUM_PORTS;
 err_register_ibdev:
	for ( i-- ; i >= 0 ; i-- )
		unregister_ibdev ( linda->ibdev[i] );
	i = LINDA_NUM_PORTS;
 err_alloc_ibdev:
	for ( i-- ; i >= 0 ; i-- )
		ibdev_put ( ibdev );
	free ( linda );
 err_alloc_linda:
	return rc;
}

/**
 * Remove PCI device
 *
 * @v pci		PCI device
 */
static void linda_remove ( struct pci_device *pci ) {
	struct linda *linda = pci_get_drvdata ( pci );
	int i;

	for ( i = ( LINDA_NUM_PORTS - 1 ) ; i >= 0 ; i-- )
		unregister_ibdev ( linda->ibdev[i] );
	for ( i = ( LINDA_NUM_PORTS - 1 ) ; i >= 0 ; i-- )
		ibdev_put ( linda->ibdev[i] );
	free ( linda );
}

static struct pci_device_id linda_nics[] = {
	PCI_ROM ( 0x1077, 0x7220, "iba7220", "QLE7240/7280 HCA driver" ),
};

struct pci_driver linda_driver __pci_driver = {
	.ids = linda_nics,
	.id_count = ( sizeof ( linda_nics ) / sizeof ( linda_nics[0] ) ),
	.probe = linda_probe,
	.remove = linda_remove,
};
