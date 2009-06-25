package Test::Network::Vde;

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

Test::Network::Vde - Test network using Virtual Distributed Ethernet

=head1 SYNOPSIS

    use Test::Network::Vde;

    # Create a new VDE network
    my $network = new Test::Network::Vde;
    print "VDE switch available via ".$network->vde_socket()."\n";

    # Destroy VDE network
    undef $network;

=head1 DESCRIPTION

C<Test::Network::Vde> provides a C<Test::Network> implementation using
the external C<vde_switch> program.

=head1 METHODS

=cut

use Expect;
use File::Spec::Functions qw ( tmpdir );
use File::Temp;
use strict;
use warnings;

use constant VDE_TIMEOUT => 5;

=pod

=over

=item C<< new() >>

Create a new C<Test::Network::Vde> network.

=cut

sub new {
  my $class = shift;
  my $this = {};

  # Chose a socket directory
  my $socket = File::Temp::tempnam ( tmpdir(), "vde." );
  $this->{socket} = $socket;

  # Start up vde_switch
  my $vde_switch = new Expect;
  $vde_switch->raw_pty ( 1 );
  $vde_switch->log_stdout ( 0 );
  $vde_switch->spawn ( "/usr/bin/vde_switch", "-sock", $socket )
      or die "Could not start vde_switch: $!\n";
  $this->{vde_switch} = $vde_switch;

  # Check that vde_switch started successfully
  $vde_switch->send ( "\n" );
  my $reached_prompt = $vde_switch->expect ( VDE_TIMEOUT, '-re', '^vde: ' );
  my $before = $vde_switch->before();
  if ( ( ! $reached_prompt ) || ( $before ne "\n" ) ) {
    $vde_switch->hard_close();
    die "Failed to start up vde_switch for ".$socket.":\n".$before."\n";
  }
  $vde_switch->clear_accum();

  bless $this, $class;
  return $this;
}

sub DESTROY {
  my $this = shift;

  $this->{vde_switch}->hard_close();
}

=pod

=item C<< vde_socket() >>

Return the VDE socket directory.  This can be passed to external
programs such as C<qemu> using e.g.

    my $socket = $network->vde_socket();
    system ( "qemu", "-net", "vde,sock=$socket", ... );

=cut

sub vde_socket {
  my $this = shift;

  return $this->{socket};
}

1;
