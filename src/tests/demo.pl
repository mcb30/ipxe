#!/usr/bin/perl -w

use Test::Network::Vde;
use Test::Machine::Qemu;
use strict;
use warnings;

my $network = new Test::Network::Vde;

my $machine = new Test::Machine::Qemu;
$machine->fdd ( "rtl8139.dsk" );
$machine->cdrom ( "win2k3.iso" );
$machine->hdd ( "drive_c.img", "drive_d.img" );
$machine->nic ( { type => "rtl8139", macaddr => "52:54:00:12:34:56" } );
$machine->network ( $network );
$machine->run();
