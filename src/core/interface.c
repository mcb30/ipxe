/*
 * Copyright (C) 2007 Michael Brown <mbrown@fensystems.co.uk>.
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

#include <string.h>
#include <gpxe/interface.h>

/** @file
 *
 * Object interfaces
 *
 */

/*****************************************************************************
 *
 * The null interface
 *
 */

/** Null interface operations */
static struct interface_operation null_intf_op[] = {};

/** Null interface descriptor */
struct interface_descriptor null_intf_desc =
	INTF_DESC ( struct null_interface, intf, null_intf_op );

/** The null interface */
struct null_interface null_intf = {
	.intf = INTF_INIT ( null_intf_desc ),
};

/*****************************************************************************
 *
 * Object interface plumbing
 *
 */

/**
 * Plug an object interface into a new destination object interface
 *
 * @v intf		Object interface
 * @v dest		New destination object interface
 *
 * The reference to the existing destination interface is dropped, a
 * reference to the new destination interface is obtained, and the
 * interface is updated to point to the new destination interface.
 *
 * Note that there is no "unplug" call; instead you must plug the
 * interface into a null interface.
 */
void intf_plug ( struct interface *intf, struct interface *dest ) {
	DBGC ( INTF_COL ( intf ),
	       "INTF " INTF_INTF_FMT " replug to " INTF_FMT "\n",
	       INTF_INTF_DBG ( intf, intf->dest ), INTF_DBG ( dest ) );
	intf_get ( dest );
	intf_put ( intf->dest );
	intf->dest = dest;
}

/**
 * Plug two object interfaces together
 *
 * @v a			Object interface A
 * @v b			Object interface B
 *
 * Plugs interface A into interface B, and interface B into interface
 * A.  (The basic plug() function is unidirectional; this function is
 * merely a shorthand for two calls to plug(), hence the name.)
 */
void intf_plug_plug ( struct interface *a, struct interface *b ) {
	intf_plug ( a, b );
	intf_plug ( b, a );
}

/**
 * Unplug an object interface
 *
 * @v intf		Object interface
 */
void intf_unplug ( struct interface *intf ) {
	intf_plug ( intf, &null_intf.intf );
}

/**
 * Ignore all further operations on an object interface
 *
 * @v intf		Object interface
 */
void intf_nullify ( struct interface *intf ) {
	intf->desc = &null_intf_desc;
}

/**
 * Increment reference count on an object interface
 *
 * @v intf		Object interface
 * @ret intf		Object interface
 */
struct interface * intf_get ( struct interface *intf ) {
	ref_get ( intf->refcnt );
	return intf;
}

/**
 * Get reference to destination object interface
 *
 * @v intf		Object interface
 * @ret dest		Destination object interface
 */
struct interface * intf_get_dest ( struct interface *intf ) {
	return intf_get ( intf->dest );
}

/**
 * Decrement reference count on an object interface
 *
 * @v intf		Object interface
 */
void intf_put ( struct interface *intf ) {
	ref_put ( intf->refcnt );
}

/**
 * Get pointer to object containing object interface
 *
 * @v intf		Object interface
 * @ret object		Containing object
 */
void * intf_object ( struct interface *intf ) {
	return ( ( ( void * ) intf ) - intf->desc->offset );
}

/**
 * Get object interface operation method
 *
 * @v intf		Object interface
 * @v type		Operation type
 * @ret func		Implementing method, or NULL
 */
void * intf_op_untyped ( struct interface *intf, void *type ) {
	struct interface_descriptor *desc = intf->desc;
	struct interface_operation *op = desc->op;
	unsigned int i;

	for ( i = desc->num_op ; i ; i--, op++ ) {
		if ( op->type == type )
			return op->func;
	}
	return NULL;
}

/*****************************************************************************
 *
 * Generic interface operations
 *
 */

/**
 * Close an object interface
 *
 * @v intf		Object interface
 * @v rc		Reason for close
 *
 * Note that this function merely informs the destination object that
 * the interface is about to be closed; it doesn't actually disconnect
 * the interface.  In most cases, you probably want to use
 * intf_shutdown() or intf_restart() instead.
 */
void intf_close ( struct interface *intf, int rc ) {
	struct interface *dest = intf_get_dest ( intf );
	intf_close_TYPE ( void * ) *op = intf_op ( dest, intf_close );
	void *object = intf_object ( dest );

	DBGC ( INTF_COL ( intf ), "INTF " INTF_INTF_FMT " close (%s)\n",
	       INTF_INTF_DBG ( intf, dest ), strerror ( rc ) );

	if ( op ) {
		op ( object, rc );
	} else {
		/* Default is to ignore intf_close() */
	}

	intf_put ( dest );
}

/**
 * Shut down an object interface
 *
 * @v intf		Object interface
 * @v rc		Reason for close
 *
 * Blocks further operations from being received via the interface,
 * executes a close operation on the destination interface, and
 * unplugs the interface.
 */
void intf_shutdown ( struct interface *intf, int rc ) {

	DBGC ( INTF_COL ( intf ), "INTF " INTF_FMT " shutting down (%s)\n",
	       INTF_DBG ( intf ), strerror ( rc ) );

	/* Block further operations */
	intf_nullify ( intf );

	/* Notify destination of close */
	intf_close ( intf, rc );

	/* Unplug interface */
	intf_unplug ( intf );
}

/**
 * Shut down and restart an object interface
 *
 * @v intf		Object interface
 * @v rc		Reason for close
 *
 * Shuts down the interface, then unblocks operations that were
 * blocked during shutdown.
 */
void intf_restart ( struct interface *intf, int rc ) {
	struct interface_descriptor *desc = intf->desc;

	/* Shut down the interface */
	intf_shutdown ( intf, rc );

	DBGC ( INTF_COL ( intf ), "INTF " INTF_FMT " restarting\n",
	       INTF_DBG ( intf ) );

	/* Restore the interface descriptor.  Must be done after
	 * shutdown (rather than inhibiting intf_shutdown() from
	 * nullifying the descriptor) in order to avoid a potential
	 * infinite loop as the intf_close() operations on each side
	 * of the link call each other recursively.
	 */
	intf->desc = desc;
}
