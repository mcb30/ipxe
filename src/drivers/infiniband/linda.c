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
#include <gpxe/i2c.h>
#include <gpxe/bitbash.h>
#include "linda.h"

/**
 * @file
 *
 * QLogic Linda Infiniband HCA
 *
 */

/** A Linda HCA */
struct linda {
	/** Registers */
	void *regs;
	/** Infiniband devices */
	struct ib_device *ibdev[LINDA_NUM_PORTS];

	/** I2C bit-bashing interface */
	struct i2c_bit_basher i2c;
	/** I2C serial EEPROM */
	struct i2c_device eeprom;
};

/**
 * Read Linda qword register
 *
 * @v linda		Linda device
 * @v dwords		Register buffer to read into
 * @v offset		Register offset
 */
static void linda_readq ( struct linda *linda, uint32_t *dwords,
			  unsigned long offset ) {
	dwords[0] = readl ( linda->regs + offset + 0 );
	dwords[1] = readl ( linda->regs + offset + 4 );
}
#define linda_readq( _linda, _ptr, _offset ) \
	linda_readq ( (_linda), (_ptr)->u.dwords, (_offset) )

/**
 * Write Linda qword register
 *
 * @v linda		Linda device
 * @v dwords		Register buffer to write
 * @v offset		Register offset
 */
static void linda_writeq ( struct linda *linda, const uint32_t *dwords,
			   unsigned long offset ) {
	writel ( dwords[0], linda->regs + offset + 0 );
	writel ( dwords[1], linda->regs + offset + 4 );
}
#define linda_writeq( _linda, _ptr, _offset ) \
	linda_writeq ( (_linda), (_ptr)->u.dwords, (_offset) )

/***************************************************************************
 *
 * I2C bus operations
 *
 ***************************************************************************
 */

/** Linda I2C bit to GPIO mappings */
static unsigned int linda_i2c_bits[] = {
	[I2C_BIT_SCL] = ( 1 << LINDA_GPIO_SCL ),
	[I2C_BIT_SDA] = ( 1 << LINDA_GPIO_SDA ),
};

/**
 * Read Linda I2C line status
 *
 * @v basher		Bit-bashing interface
 * @v bit_id		Bit number
 * @ret zero		Input is a logic 0
 * @ret non-zero	Input is a logic 1
 */
static int linda_i2c_read_bit ( struct bit_basher *basher,
				unsigned int bit_id ) {
	struct linda *linda =
		container_of ( basher, struct linda, i2c.basher );
	struct QIB_7220_GPIO gpiostatus;
	unsigned int status;

	linda_readq ( linda, &gpiostatus, QIB_7220_GPIOStatus_offset );
	status = BIT_GET ( &gpiostatus, GPIO );
	return ( status & linda_i2c_bits[bit_id] );
}

/**
 * Write Linda I2C line status
 *
 * @v basher		Bit-bashing interface
 * @v bit_id		Bit number
 * @v data		Value to write
 */
static void linda_i2c_write_bit ( struct bit_basher *basher,
				  unsigned int bit_id, unsigned long data ) {
	struct linda *linda =
		container_of ( basher, struct linda, i2c.basher );
	struct QIB_7220_EXTCtrl extctrl;
	struct QIB_7220_GPIO gpioout;
	unsigned int bit = linda_i2c_bits[bit_id];
	unsigned int outputs;
	unsigned int output_enables;

	/* Read current GPIO mask and outputs */
	linda_readq ( linda, &extctrl, QIB_7220_EXTCtrl_offset );
	linda_readq ( linda, &gpioout, QIB_7220_GPIOOut_offset );

	/* Update outputs and output enables.  I2C lines are tied
	 * high, so we always set the output to 0 and use the output
	 * enable to control the line.  Write the output enable first;
	 * that way we avoid logic hazards.
	 */
	output_enables = BIT_GET ( &extctrl, GPIOOe );
	output_enables = ( ( output_enables & ~bit ) | ( ~data & bit ) );
	outputs = BIT_GET ( &gpioout, GPIO );
	outputs = ( outputs & ~bit );
	BIT_SET ( &extctrl, 0, GPIOOe, output_enables );
	BIT_SET ( &gpioout, 0, GPIO, outputs );

	linda_writeq ( linda, &extctrl, QIB_7220_EXTCtrl_offset );
	linda_writeq ( linda, &gpioout, QIB_7220_GPIOOut_offset );
}

/** Linda I2C bit-bashing interface operations */
static struct bit_basher_operations linda_i2c_basher_ops = {
	.read	= linda_i2c_read_bit,
	.write	= linda_i2c_write_bit,
};

/**
 * Initialise Linda I2C subsystem
 *
 * @v linda		Linda device
 * @ret rc		Return status code
 */
static int linda_init_i2c ( struct linda *linda ) {
	static int try_eeprom_address[] = { 0x50, 0x51 };
	unsigned int i;
	int rc;

	/* Initialise bus */
	if ( ( rc = init_i2c_bit_basher ( &linda->i2c,
					  &linda_i2c_basher_ops ) ) != 0 ) {
		DBGC ( linda, "Linda %p could not initialise I2C bus: %s\n",
		       linda, strerror ( rc ) );
		return rc;
	}

	/* Probe for devices */
	for ( i = 0 ; i < ( sizeof ( try_eeprom_address ) /
			    sizeof ( try_eeprom_address[0] ) ) ; i++ ) {
		init_i2c_eeprom ( &linda->eeprom, try_eeprom_address[i] );
		if ( ( rc = i2c_check_presence ( &linda->i2c.i2c,
						 &linda->eeprom ) ) == 0 ) {
			DBGC ( linda, "Linda %p found EEPROM at %02x\n",
			       linda, try_eeprom_address[i] );
			return 0;
		}
	}

	DBGC ( linda, "Linda %p could not find EEPROM\n", linda );
	return -ENODEV;
}

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
	linda_readq ( linda, &revision, QIB_7220_Revision_offset );
	DBGC ( linda, "Linda %p board %02lx v%ld.%ld.%ld.%ld\n", linda,
	       BIT_GET ( &revision, BoardID ),
	       BIT_GET ( &revision, R_SW ),
	       BIT_GET ( &revision, R_Arch ),
	       BIT_GET ( &revision, R_ChipRevMajor ),
	       BIT_GET ( &revision, R_ChipRevMinor ) );

	/* Initialise I2C subsystem */
	if ( ( rc = linda_init_i2c ( linda ) ) != 0 )
		goto err_init_i2c;

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
 err_init_i2c:
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
