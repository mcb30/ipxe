/**************************************************************************
gPXE -  Network Bootstrap Program

Literature dealing with the network protocols:
	ARP - RFC826
	RARP - RFC903
	UDP - RFC768
	BOOTP - RFC951, RFC2132 (vendor extensions)
	DHCP - RFC2131, RFC2132 (options)
	TFTP - RFC1350, RFC2347 (options), RFC2348 (blocksize), RFC2349 (tsize)
	RPC - RFC1831, RFC1832 (XDR), RFC1833 (rpcbind/portmapper)
	NFS - RFC1094, RFC1813 (v3, useful for clarifications, not implemented)
	IGMP - RFC1112

**************************************************************************/

#include <gpxe/init.h>
#include <gpxe/shell.h>
#include <gpxe/shell_banner.h>
#include <usr/autoboot.h>

#include <io.h>

/**
 * Main entry point
 *
 * @ret rc		Return status code
 */
int main ( void ) {

	void *dump1 = phys_to_virt ( 0x220000 - 64 );
	void *dump2 = phys_to_virt ( 0x420000 - 64 );
	more();
	printf ( "\nAfter relocation:\n" );
	dbg_hex_dump_da ( virt_to_phys ( dump1 ), dump1, 128 );
	printf ( "\n" );
	dbg_hex_dump_da ( virt_to_phys ( dump2 ), dump2, 128 );

	while ( 1 ) {}

	initialise();
	startup();

	/* Try autobooting if we're not going straight to the shell */
	if ( ! shell_banner() ) {
		autoboot();
	}
	
	/* Autobooting failed or the user wanted the shell */
	shell();

	shutdown();

	return 0;
}
