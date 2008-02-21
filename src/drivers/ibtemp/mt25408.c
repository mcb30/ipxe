/**************************************************************************
Etherboot -  BOOTP/TFTP Bootstrap Program
Skeleton NIC driver for Etherboot
***************************************************************************/

/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2, or (at
 * your option) any later version.
 */

/* to get some global routines like printf */
#include "etherboot.h"
/* to get the interface to the body of the program */
#include "nic.h"
/* to get the PCI support functions, if this is a PCI NIC */
//#include "pci.h"
/* to get the ISA support functions, if this is an ISA NIC */
//#include "isa.h"

#include "mt_version.c"
#include "mt25408_imp.c"

/* NIC specific static variables go here */

int prompt_key(int secs, unsigned char *ch_p)
{
	unsigned long tmo;
	unsigned char ch;

	for (tmo = currticks() + secs * TICKS_PER_SEC; currticks() < tmo;) {
		if (iskey()) {
			ch = getchar();
			/* toupper does not work ... */
			if (ch == 'v')
				ch = 'V';
			if (ch == 'i')
				ch = 'I';
			if (ch == 'd')
				ch = 'D';
			if ((ch=='V') || (ch=='I') || (ch=='D')) {
                                *ch_p = ch;
                                return 1;
			}
		}
	}

	return 0;
}

/**************************************************************************
IRQ - handle interrupts
***************************************************************************/
static void mt25408_irq(struct nic *nic, irq_action_t action)
{
	/* This routine is somewhat optional.  Etherboot itself
	 * doesn't use interrupts, but they are required under some
	 * circumstances when we're acting as a PXE stack.
	 *
	 * If you don't implement this routine, the only effect will
	 * be that your driver cannot be used via Etherboot's UNDI
	 * API.  This won't affect programs that use only the UDP
	 * portion of the PXE API, such as pxelinux.
	 */

	if (0) {
		nic = NULL;
	}
	switch (action) {
	case DISABLE:
	case ENABLE:
		/* Set receive interrupt enabled/disabled state */
		/*
		   outb ( action == ENABLE ? IntrMaskEnabled : IntrMaskDisabled,
		   nic->ioaddr + IntrMaskRegister );
		 */
		break;
	case FORCE:
		/* Force NIC to generate a receive interrupt */
		/*
		   outb ( ForceInterrupt, nic->ioaddr + IntrForceRegister );
		 */
		break;
	}
}

/**************************************************************************
POLL - Wait for a frame
***************************************************************************/
static int mt25408_poll(struct nic *nic, int retrieve)
{
	/* Work out whether or not there's an ethernet packet ready to
	 * read.  Return 0 if not.
	 */
	/* 
	   if ( ! <packet_ready> ) return 0;
	 */

	/* retrieve==0 indicates that we are just checking for the
	 * presence of a packet but don't want to read it just yet.
	 */
	/*
	   if ( ! retrieve ) return 1;
	 */

	/* Copy data to nic->packet.  Data should include the
	 * link-layer header (dest MAC, source MAC, type).
	 * Store length of data in nic->packetlen.
	 * Return true to indicate a packet has been read.
	 */
	/* 
	   nic->packetlen = <packet_length>;
	   memcpy ( nic->packet, <packet_data>, <packet_length> );
	   return 1;
	 */
	unsigned int size;
	int rc;
	rc = poll_imp(nic, retrieve, &size);
	if (rc) {
		return 0;
	}

	if (size == 0) {
		return 0;
	}

	nic->packetlen = size;

	return 1;
}

/**************************************************************************
TRANSMIT - Transmit a frame
***************************************************************************/
static void mt25408_transmit(struct nic *nic, const char *dest,	/* Destination */
			     unsigned int type,	/* Type */
			     unsigned int size,	/* size */
			     const char *packet)
{				/* Packet */
	int rc;

	/* Transmit packet to dest MAC address.  You will need to
	 * construct the link-layer header (dest MAC, source MAC,
	 * type).
	 */
	if (nic) {
		rc = transmit_imp(dest, type, packet, size);
		if (rc)
			eprintf("tranmit error");
	}
}

/**************************************************************************
DISABLE - Turn off ethernet interface
***************************************************************************/
static void mt25408_disable(struct nic *nic)
{
	/* put the card in its initial state */
	/* This function serves 3 purposes.
	 * This disables DMA and interrupts so we don't receive
	 *  unexpected packets or interrupts from the card after
	 *  etherboot has finished. 
	 * This frees resources so etherboot may use
	 *  this driver on another interface
	 * This allows etherboot to reinitialize the interface
	 *  if something is something goes wrong.
	 */
	if (nic || 1) {		// ????
		disable_imp();
	}
}

/**************************************************************************
PROBE - Look for an adapter, this routine's visible to the outside
***************************************************************************/

static struct nic_operations mt25408_operations = {
	.connect	= dummy_connect,
	.poll		= mt25408_poll,
	.transmit	= mt25408_transmit,
	.irq		= mt25408_irq,
};

static int mt25408_probe(struct nic *nic, struct pci_device *pci)
{
	int rc;
	unsigned char user_request;

	if (pci->vendor != MELLANOX_VENDOR_ID) {
		eprintf("");
		return 0;
	}

	printf("\n");
	printf("Mellanox Technologies LTD - Boot over IB implementaion\n");
	printf("Build version = %s\n\n", build_revision);

	verbose_messages = 0;
        print_info = 0;
	debug_messages = 0;
	printf("Press within 3 seconds:\n");
	//printf("V - to increase verbosity\n");
	printf("I - to print information\n");

	if (prompt_key(3, &user_request)) {
		if (user_request == 'V') {
			printf("User selected verbose messages\n");
			verbose_messages = 1;
		}
		else if (user_request == 'I') {
			printf("User selected to print information\n");
			print_info = 1;
		}
		else if (user_request == 'D') {
			printf("User selected to print debug messages\n");
			debug_messages = 1;
		}
	}
	printf("\n");

	adjust_pci_device(pci);

	nic->priv_data = NULL;
	rc = probe_imp(pci, nic);

	/* give the user a chance to look at the info */
	if (print_info)
		sleep(5);

	if (!rc) {
		/* store NIC parameters */
		nic->ioaddr = pci->ioaddr & ~3;
		nic->irqno = pci->irq;
		/* point to NIC specific routines */
		nic->nic_op = &mt25408_operations;

		return 1;
	}
	/* else */
	return 0;
}


static struct pci_device_id mt25408_nics[] = {
    PCI_ROM(0x15b3, 0x6340, "MT25408", "MT25408 HCA driver"),
    PCI_ROM(0x15b3, 0x634a, "MT25418", "MT25418 HCA driver"),
    
};

PCI_DRIVER ( mt25408_driver, mt25408_nics, PCI_NO_CLASS );
DRIVER ( "MT25048", nic_driver, pci_driver, mt25408_driver,
	 mt25408_probe, mt25408_disable );
