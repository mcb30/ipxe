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
#include <errno.h>
#include <gpxe/job.h>

/** @file
 *
 * Job control interfaces
 *
 */

/**
 * Finish using a job control interface
 *
 * @v job		Job control interface
 * @v rc		Overall job status code
 *
 * Blocks all further messages, sends a close() message to the remote
 * end and unplugs the interface.
 */
void job_finished ( struct job_interface *job, int rc ) {

	/* Sustain existence during close */
	job_get ( job );

	/* Block further incoming messages */
	job_nullify ( job );

	/* Close interface */
	job_close ( job, rc );

	/* Unplug interface */
	job_unplug ( job );

	/* Drop sustaining reference */
	job_put ( job );
}

/****************************************************************************
 *
 * Helper methods
 *
 * These functions are designed to be used as methods in the
 * job_interface_operations table.
 *
 */

void ignore_job_close ( struct job_interface *job __unused, int rc __unused ) {
	/* Nothing to do */
}

void ignore_job_progress ( struct job_interface *job __unused,
			   struct job_progress *progress ) {
	memset ( progress, 0, sizeof ( *progress ) );
}

/** Null job control interface operations */
struct job_interface_operations null_job_ops = {
	.close		= ignore_job_close,
	.progress	= ignore_job_progress,
};

/**
 * Null job control interface
 *
 * This is the interface to which job control interfaces are connected
 * when unplugged.  It will never generate messages, and will silently
 * absorb all received messages.
 */
struct job_interface null_job = JOB_INIT ( null_job, &null_job_ops );
