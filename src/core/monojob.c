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
#include <stdio.h>
#include <errno.h>
#include <gpxe/process.h>
#include <console.h>
#include <gpxe/keys.h>
#include <gpxe/job.h>
#include <gpxe/monojob.h>

/** @file
 *
 * Single foreground job
 *
 */

static int monojob_rc;

static void monojob_close ( struct job_interface *job, int rc ) {
	monojob_rc = rc;
	job_finished ( job, rc );
}

/** Single foreground job operations */
static struct job_interface_operations monojob_operations = {
	.close		= monojob_close,
	.progress	= ignore_job_progress,
};

/** Single foreground job */
struct job_interface monojob = JOB_INIT ( monojob, &monojob_operations );

/**
 * Wait for single foreground job to complete
 *
 * @v string		Job description to display
 * @ret rc		Job final status code
 */
int monojob_wait ( const char *string ) {
	int key;
	int rc;

	printf ( "%s... ", string );
	monojob_rc = -EINPROGRESS;
	monojob.op = &monojob_operations;
	while ( monojob_rc == -EINPROGRESS ) {
		step();
		if ( iskey() ) {
			key = getchar();
			switch ( key ) {
			case CTRL_C:
				rc = -ECANCELED;
				job_finished ( &monojob, rc );
				goto done;
			default:
				break;
			}
		}
	}
	rc = monojob_rc;

done:
	if ( rc ) {
		printf ( "%s\n", strerror ( rc ) );
	} else {
		printf ( "ok\n" );
	}
	return rc;
}
