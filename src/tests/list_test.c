/*
 * Copyright (C) 2011 Michael Brown <mbrown@fensystems.co.uk>.
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

FILE_LICENCE ( GPL2_OR_LATER );

/** @file
 *
 * List function tests
 *
 */

/* Forcibly enable assertions for list_check() */
#undef NDEBUG
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <ipxe/list.h>
#include <ipxe/test.h>

/** A list test structure */
struct list_test {
	/** List element */
	struct list_head list;
	/** Label */
	char label;
};

/** List test elements */
static struct list_test list_tests[] = {
	{ .label = '0' },
	{ .label = '1' },
	{ .label = '2' },
	{ .label = '3' },
	{ .label = '4' },
	{ .label = '5' },
	{ .label = '6' },
	{ .label = '7' },
	{ .label = '8' },
	{ .label = '9' },
};

/** Test list */
static LIST_HEAD ( test_list );

/**
 * Check list contents are as expected
 *
 * @v list		Test list
 * @v expected		Expected contents
 * @v ok		List contents are as expected
 */
static int list_check_contents ( struct list_head *list,
				 const char *expected ) {
	struct list_test *entry;
	size_t num_entries = 0;

	/* Determine size of list */
	list_for_each_entry ( entry, list, list )
		num_entries++;

	{
		char found[ num_entries + 1 ];
		char found_rev[ num_entries + 1 ];
		char *tmp;

		/* Build up list content string */
		tmp = found;
		list_for_each_entry ( entry, list, list )
			*(tmp++) = entry->label;
		*tmp = '\0';

		/* Sanity check reversed list */
		tmp = &found_rev[ sizeof ( found_rev ) - 1 ];
		*tmp = '\0';
		list_for_each_entry_reverse ( entry, list, list )
			*(--tmp) = entry->label;
		if ( strcmp ( found, found_rev ) != 0 ) {
			printf ( "FAILURE: list reversal mismatch (forward "
				 "\"%s\", reverse \"%s\")\n",
				 found, found_rev  );
			return 0;
		}

		/* Compare against expected content */
		if ( strcmp ( found, expected ) == 0 ) {
			return 1;
		} else {
			printf ( "FAILURE: expected \"%s\", got \"%s\"\n",
			 expected, found );
			return 0;
		}
	}
}

/**
 * Report list test result
 *
 * @v list		Test list
 * @v expected		Expected contents
 */
#define list_contents_ok( list, expected ) do {			\
	ok ( list_check_contents ( (list), (expected) ) );	\
	} while ( 0 )

/**
 * Perform list self-test
 *
 */
static void list_test_exec ( void ) {
	struct list_head *list = &test_list;

	/* Test initialiser */
	ok ( list_empty ( list ) );
	list_contents_ok ( list, "" );

	/* Test list_add(), list_add_tail() and list_del() */
	INIT_LIST_HEAD ( list );
	list_contents_ok ( list, "" );
	list_add ( &list_tests[4].list, list ); /* prepend */
	list_contents_ok ( list, "4" );
	list_add ( &list_tests[2].list, list ); /* prepend */
	list_contents_ok ( list, "24" );
	list_add_tail ( &list_tests[7].list, list ); /* append */
	list_contents_ok ( list, "247" );
	list_add ( &list_tests[1].list, &list_tests[4].list ); /* after */
	list_contents_ok ( list, "2417" );
	list_add_tail ( &list_tests[8].list, &list_tests[7].list ); /* before */
	list_contents_ok ( list, "24187" );
	list_del ( &list_tests[4].list ); /* delete middle */
	list_contents_ok ( list, "2187" );
	list_del ( &list_tests[2].list ); /* delete first */
	list_contents_ok ( list, "187" );
	list_del ( &list_tests[7].list ); /* delete last */
	list_contents_ok ( list, "18" );
	list_del ( &list_tests[1].list ); /* delete all */
	list_del ( &list_tests[8].list ); /* delete all */
	list_contents_ok ( list, "" );
}

/** List self-test */
struct self_test list_test __self_test = {
	.name = "list",
	.exec = list_test_exec,
};
