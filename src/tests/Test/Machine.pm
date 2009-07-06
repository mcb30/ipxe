package Test::Machine;

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

Test::Machine - An abstraction of a test machine

=head1 SYNOPSIS

    use Test::Machine;

    # Create a new Qemu machine
    my $machine = new Test::Machine;
    $machine->fdd ( "rtl8139.dsk" );
    $machine->cdrom ( "win2k8.iso" );

=head1 DESCRIPTION

C<Test::Machine> provides an abstraction of a test machine.  The
actual implementation is provided by a backend such as
C<Test::Machine::Qemu>.

=head1 METHODS

=cut

use strict;
use warnings;

=pod

=over

=item C<< new() >>

Create a new C<Test::Machine>> machine.

=cut

sub new {
  my $class = shift;
  my $this = {};

  $this->{media} = {};
  $this->{nics} = {};
  $this->{networks} = {};

  bless $this, $class;
  return $this;
}

sub DESTROY {
  my $this = shift;
  # Do nothing
}

=pod

=item C<< run() >>

Start running the C<Test::Machine>.

=cut

sub run {
  my $this = shift;
  # Do nothing
}

=pod

=item C<< media ( $name, $medium, [$medium, ...] ) >>

Define storage media for the C<Test::Machine>.

=cut

sub media {
  my $this = shift;
  my $media_name = shift;
  my @media = @_;

  my $index = 0;
  my $changes = {};
  foreach my $medium ( @media ) {

    # Determine medium name.  Given $media_name="xdd", we will use
    # "xdd" if we were given just a plain medium, or "xdd0", "xdd1",
    # ... if we were given a list.
    my $medium_name = ( ( @media > 1 ) ? $media_name.$index : $media_name );
    $changes->{$medium_name} = {
      old => $this->{media}->{$medium_name},
      new => undef,
    };

    # Treat "undef" as meaning "delete medium"
    if ( ! defined $medium ) {
      delete $this->{media}->{$medium_name};
      next;
    }

    # Treat a plain string $string as equivalent to "path => $string"
    $medium = { path => $medium } unless ref $medium;

    # Allow a list of typeless media to be passed in, using the media
    # name as the media type.  This simplifies the various
    # media-type-specific wrapper methods.
    $medium->{type} ||= $media_name;

    # Allow a list of unindexed media to be passed in.
    $medium->{index} ||= $index;

    # Store media definition
    $this->{media}->{$medium_name} = $medium;

    # Add to list of changed media
    $changes->{$medium_name}->{new} = $medium;

  } continue {
    $index++;
  }

  return $changes;
}

=pod

=item C<< hdd ( $medium, [$medium, ...] ) >>

Define hard disks for the C<Test::Machine>.  Each C<$medium> can be
either a plain filename such as "win2k3.img", or a hash such as

    {
	path => "win2k3.img",
	cyls => 1024,
	heads => 255,
	sectors => 63,
    }

=cut

sub hdd {
  my $this = shift;
  return $this->media ( "hdd", @_ );
}

=pod

=item C<< cdrom ( $medium, [$medium, ...] ) >>

Define CD-ROMs for the C<Test::Machine>, as for C<hdd()>.

=cut

sub cdrom {
  my $this = shift;
  return $this->media ( "cdrom", @_ );
}

=pod

=item C<< fdd ( $medium, [$medium, ...] ) >>

Define floppy disks for the C<Test::Machine>, as for C<hdd()>.

=cut

sub fdd {
  my $this = shift;
  return $this->media ( "fdd", @_ );
}

=pod

=item C<< nic ( $nic, [$nic, ...] ) >>

Define network cards for the C<Test;:Machine>.  Each C<$nic> can be
either a plain NIC type such as "rtl8139", or a hash such as

    {
	type => "rtl8139",
	macaddr => "52:54:00:12:34:56",
    }

=cut

sub nic {
  my $this = shift;
  my @nics = @_;

  my $index = 0;
  my $changes = {};
  foreach my $nic ( @nics ) {

    # Build change list
    $changes->{$index} = {
      old => $this->{nics}->{$index},
      new => undef,
    };

    # Treat "undef" as meaning "remove NIC"
    if ( ! defined $nic ) {
      delete $this->{nics}->{$index};
      next;
    }

    # Treat a plain string $string as equivalent to "type => $string"
    $nic = { type => $nic } unless ref $nic;

    # Store NIC definition
    $this->{nics}->{$index} = $nic;

    # Add to list of changed NICs
    $changes->{$index}->{new} = $nic;

  } continue {
    $index++;
  };

  return $changes;
}

=pod

=item C<< network ( $network, [$network, ...] ) >>

Define network connections for the C<Test::Machine>.  Each C<$network>
must be a C<Test::Network> object.

=cut

sub network {
  my $this = shift;
  my @networks = @_;

  my $index = 0;
  my $changes = {};
  foreach my $network ( @networks ) {

    # Build change list
    $changes->{$index} = {
      old => $this->{networks}->{$index},
      new => $network,
    };

    # Store network definition
    if ( defined $network ) {
      $this->{networks}->{$index} = $network;
    } else {
      delete $this->{networks}->{$index};
    }

  } continue {
    $index++;
  }

  return $changes;
}

=pod

=item C<< save_snapshot ( $name ) >>

Save the state of the C<Test::Machine> to a snapshot named <$name>.

=cut

sub save_snapshot {
  my $this = shift;
  # Do nothing
}

=pod

=item C<< load_snapshot ( $name ) >>

Load the state of the C<Test::Machine> from a snapshot named <$name>.

=cut

sub load_snapshot {
  my $this = shift;
  # Do nothing
}

1;
