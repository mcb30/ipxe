package Test::Machine::Qemu;

# Copyright (C) 2009 Michael Brown <mbrown@fensystems.co.uk>.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of the
# License, or any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

=head1 NAME

Test::Machine::Qemu - Test machine using Qemu

=head1 SYNOPSIS

    use Test::Machine::Qemu;

    # Create a new Qemu machine
    my $machine = new Test::Machine::Qemu;
    $machine->fdd ( "rtl8139.dsk" );
    $machine->cdrom ( "win2k8.iso" );
    $machine->nic ( { type => "rtl8139", macaddr => "52:54:00:12:34:56" } );
    $machine->run();

    # Connect Qemu machine to a VDE network
    my $network = new Test::Network::Vde;
    $machine->network ( $network );

    # Power-off Qemu machine
    undef $machine;

=head1 DESCRIPTION

C<Test::Machine::Qemu> provides a C<Test::Machine> implementation using
a C<qemu> virtual machine.

=head1 METHODS

=cut

use Expect;
use Test::Machine;
use strict;
use warnings;

our @ISA = qw ( Test::Machine );

use constant QEMU_TIMEOUT => 5;

=pod

=over

=item C<< new() >>

Create a new C<Test::Machine::Qemu> machine.

=cut

sub new {
  my $class = shift;
  my $this = $class->SUPER::new ( @_ );

  # Start up qemu
  my $qemu = new Expect;
  $qemu->raw_pty ( 1 );
  $qemu->log_stdout ( 1 );

  $qemu->spawn ( "/usr/bin/qemu",
      # Attach monitor to stdio
      "-monitor", "stdio",
      # Default boot order: floppy, hdd, network
      # This will be overridden later
      "-boot", "acn",
      # Do not start CPU at startup
      "-S"
    ) or die "Could not start qemu: $!\n";

  my $reached_prompt = $qemu->expect ( QEMU_TIMEOUT, '(qemu) ' );

  if ( ! $reached_prompt ) {
    $qemu->hard_close();
    die "Failed to start up qemu\n";
  }
  $qemu->clear_accum();

  $this->{qemu} = $qemu;

  return $this;
}

sub DESTROY {
  my $this = shift;
  $this->SUPER::DESTROY ( @_ );

  #
  # |DrV|: make sure that this kills qemu.  It can (and should) be an
  # abrupt power-off.  I think hard_close() will kill the process, but
  # I'm not sure.
  #
  $this->{qemu}->hard_close();
}

############################################################################
#
# Execution control
#

sub run {
  my $this = shift;
  $this->SUPER::run ( @_ );

  #
  # |DrV|: send "cont" command to monitor, and verify that it returns
  # |to the monitor prompt.
  #
}

############################################################################
#
# Storage media
#

sub media {
  my $this = shift;
  my $changes = $this->SUPER::media ( @_ );

  while ( ( my $name, my $change ) = each %$changes ) {
    #
    # |DrV|: send appropriate drive_add etc. monitor commands to bring
    # the VM's media definitions into sync.
    #
    require Data::Dumper;
    print "Medium ".$name." was modified:\n".
	Data::Dumper->Dump ( [ $change->{old}, $change->{new} ],
			     [ "old", "new" ] )."\n";
  }
}

############################################################################
#
# Network cards
#

sub nic {
  my $this = shift;
  my $changes = $this->SUPER::nic ( @_ );

  while ( ( my $vlan, my $change ) = each %$changes ) {

    #
    # |DrV|: send appropriate pci_del, pci_add etc. monitor commands
    # |to bring the VM's NIC definitions into sync.
    #

    require Data::Dumper;
    print "NIC ".$vlan." was modified:\n".
	Data::Dumper->Dump ( [ $change->{old}, $change->{new} ],
			     [ "old", "new" ] )."\n";
  }
}

############################################################################
#
# Host network connections
#

sub host_net_add_vde {
  my $this = shift;
  my $network = shift;

  # Test to see if this is a VDE network
  my $vde_socket;
  eval {
    $vde_socket = $network->vde_socket();
  };
  return undef if $@;

  # Construct host_net_add line for VDE network
  return "host_net_add vde sock=".$vde_socket;
}

sub host_net_add_tap {
  my $this = shift;
  my $network = shift;

  return undef;
}

sub network {
  my $this = shift;
  my $changes = $this->SUPER::network ( @_ );

  while ( ( my $vlan, my $change ) = each %$changes ) {

    require Data::Dumper;
    print "Network ".$vlan." was modified:\n".
	Data::Dumper->Dump ( [ $change->{old}, $change->{new} ],
			     [ "old", "new" ] )."\n";

    #
    # |DrV|: send appropriate $host_net_remove monitor command to
    # disconnect from the network.
    #

    my $network = $change->{new};
    next unless $network;

    # Try all supported network types in turn
    my $host_net_add = ( $this->host_net_add_vde ( $network ) ||
			 $this->host_net_add_tap ( $network ) )
	or die "Unsupported network type\n";

    #
    # |DrV|: send appropriate $host_net_add monitor command to connect
    # to the network.
    #
    print "Connecting to network using \"".$host_net_add."\"\n";
  }
}

1;
