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

sub monitor_command {
  my $this = shift;
  my $cmd = shift;
  my $qemu = $this->{qemu};

  $qemu->send ( "$cmd\n" );
  $qemu->expect ( QEMU_TIMEOUT, "(qemu) " )
    or die "qemu command '$cmd' timed out\n";

  my $before = $qemu->before();

  $qemu->clear_accum();

  if ($before =~ /$cmd.*?\n(.*)/s ) {
    $1;
  } else {
    "";
  }
}

=pod

=over

=item C<< new() >>

Create a new C<Test::Machine::Qemu> machine.

=cut

sub new {
  my $class = shift;
  my $this = $class->SUPER::new ( @_ );

  return $this;
}

sub DESTROY {
  my $this = shift;
  $this->SUPER::DESTROY ( @_ );

  if ( $this->{qemu} ) {
    $this->{qemu}->hard_close();
  }
}

############################################################################
#
# Execution control
#

sub run {
  my $this = shift;
  $this->SUPER::run ( @_ );

  my @cmd = ( "/usr/bin/qemu",
      # Attach monitor to stdio
      "-monitor", "stdio",
      # Default boot order: floppy, hdd, network
      # This will be overridden later
      "-boot", "acn",
      # Do not start CPU at startup
      "-S",
    );

  my $media = $this->{ media };
  foreach my $medium ( keys %{ $media } ) {
    my $type = $media->{ $medium }->{type};
    if ( $type eq "fdd" ) {
      push @cmd, "-fd" . chr(ord("a") + $media->{ $medium }->{index});
    } elsif ( $type eq "hdd" ) {
      push @cmd, "-hd" . chr(ord("a") + $media->{ $medium }->{index});
    } elsif ( $type eq "cdrom" ) {
      push @cmd, "-cdrom";
    } else {
      die "Unknown media type $type\n";
    }
    push @cmd, $media->{ $medium }->{path};
  }

  my $nics = $this->{ nics };
  foreach my $nic ( keys %{ $nics } ) {
    push @cmd, "-net";
    push @cmd, "nic," .
               "vlan=$nic," .
               "macaddr=$nics->{ $nic }->{ macaddr }," .
               "model=$nics->{ $nic }->{ type }";
  }

  my $networks = $this->{ networks };
  foreach my $network ( keys %{ $networks } ) {
    push @cmd, "-net";
    my $host_net_add =
      $this->host_net_add_vde ( $networks->{ $network } ) or
      $this->host_net_add_tap ( $networks->{ $network } ) or
      die "Unsupported network type\n";

    push @cmd, "$host_net_add,vlan=$network";
  }

  print "Running qemu: '@cmd'\n";

  # Start up qemu
  my $qemu = new Expect;
  $qemu->raw_pty ( 1 );
  $qemu->log_stdout ( 1 );

  $qemu->spawn ( @cmd ) or die "Could not start qemu: $!\n";

  my $reached_prompt = $qemu->expect ( QEMU_TIMEOUT, '(qemu) ' );

  if ( ! $reached_prompt ) {
    $qemu->hard_close();
    die "Failed to start up qemu\n";
  }
  $qemu->clear_accum();

  $this->{qemu} = $qemu;

  my $res = $this->monitor_command ( "cont" );

  if ($res) {
    die "cont failed: '$res'\n";
  }

  $this->monitor_command ( "info block" );
  $this->monitor_command ( "info network" );
}

############################################################################
#
# Storage media
#

sub device_from_medium {
  my $medium = shift;
  if ( $medium->{type} eq "cdrom" ) {
    return "ide1-cd$medium->{index}";
  } elsif ( $medium->{type} eq "fdd" ) {
    return "floppy$medium->{index}";
  } elsif ( $medium->{type} eq "hdd" ) {
    my $controller = $medium->{index} / 2;
    my $channel = $medium->{index} % 2;
    return "ide$controller-hd$channel";
  } else {
    die "device_from_medium: unsupported type $medium->{type}\n";
  }
}

sub media {
  my $this = shift;
  my $changes = $this->SUPER::media ( @_ );

  while ( ( my $name, my $change ) = each %$changes ) {

    require Data::Dumper;
    print "Medium ".$name." was modified:\n".
	Data::Dumper->Dump ( [ $change->{old}, $change->{new} ],
			     [ "old", "new" ] )."\n";

    my $medium = $change->{old};
    if ( $medium ) {
      if ( $this->{qemu} && ! $change->{new} ) {
        my $device = device_from_medium ( $medium );
        my $res = $this->monitor_command ( "eject $device" );

        if ( $res ) {
          die "Removing medium failed: $res\n";
        }
      }
    }

    $medium = $change->{new};
    next unless $medium;

    if ( $this->{qemu} ) {
      my $device = device_from_medium ( $medium );
      my $res = $this->monitor_command ( "change $device $medium->{path}" );
      if ( $res ) {
        die "Changing medium failed: $res\n";
      }
    }
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

    require Data::Dumper;
    print "NIC ".$vlan." was modified:\n".
	Data::Dumper->Dump ( [ $change->{old}, $change->{new} ],
			     [ "old", "new" ] )."\n";

    my $nic = $change->{old};    
    if ( $nic ) {
      if ( $this->{qemu} ) {
        my $res = $this->monitor_command ( "host_net_remove $vlan $nic->{type}.$vlan" );
        if ( $res ) {
          die "Removing NIC failed: $res\n";
        }
      }
    }

    $nic = $change->{new};
    next unless $nic;

    if ( $this->{qemu} ) {
      my $res = $this->monitor_command ( "pci_add pci_addr=auto nic vlan=$vlan,macaddr=$nic->{macaddr},model=$nic->{type}" );

      unless ($res =~ /^OK/) {
        die "Adding NIC failed: $res\n";
      }
    }
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
  if ( $this->{qemu} ) {
    # runtime: monitor command
    return "host_net_add vde sock=$vde_socket";
  } else {
    # command line
    return "vde,sock=$vde_socket";
  }
}

sub host_net_add_tap {
  my $this = shift;
  my $network = shift;

  return undef;
}

sub host_net_remove_vde {
  my $this = shift;
  my $network = shift;
  my $vlan = shift;

  my $vde_socket;
  eval {
    $vde_socket = $network->vde_socket();
  };
  return undef if $@;

  if ( $this->{qemu} ) {
    return "host_net_remove $vlan vde.$vlan";
  } else {
    return undef;
  }
}

sub host_net_remove_tap {
  my $this = shift;
  my $network = shift;
  my $vlan = shift;

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

    my $network = $change->{old};
    if ( $network ) {
      my $host_net_remove =
        $this->host_net_remove_vde ( $network, $vlan ) or
        $this->host_net_remove_tap ( $network, $vlan ) or
        die "Unsupported network type\n";

      if ( $this->{qemu} ) {
        print "Disconnecting network using \"$host_net_remove\"\n";

        my $res = $this->monitor_command ( $host_net_remove );

        if ( $res ) {
          die "Disconnecting from network failed: $res\n";
        }
      }
    }

    $network = $change->{new};
    next unless $network;

    # Try all supported network types in turn
    my $host_net_add = ( $this->host_net_add_vde ( $network ) ||
			 $this->host_net_add_tap ( $network ) )
	or die "Unsupported network type\n";

    

    if ( $this->{qemu} ) {
      print "Connecting to network using \"".$host_net_add."\"\n";

      my $res = $this->monitor_command ( $host_net_add );

      if ($res) {
         die "Connecting to network failed: $res\n";
      }

      $this->monitor_command ( "info network\n" );
    }
  }
}

############################################################################
#
# Snapshots
#

sub save_snapshot {
  my $this = shift;
  $this->SUPER::save_snapshot ( @_ );
  my $name = shift;

  unless ( $this->{qemu} ) {
    die "Cannot save snapshot while not running\n";
  }

  print "Saving snapshot '$name'...\n";
  my $res = $this->monitor_command ( "savevm $name" );
  if ($res) {
    die "Saving snapshot failed: $res\n";
  }
}

sub load_snapshot {
  my $this = shift;
  $this->SUPER::load_snapshot ( @_ );
  my $name = shift;

  unless ( $this->{qemu} ) {
    die "Cannot load snapshot while not running\n";
  }

  print "Loading snapshot '$name'...\n";
  my $res = $this->monitor_command ( "loadvm $name" );
  if ($res) {
    die "Saving snapshot failed: $res\n";
  }
}

1;
