package Option::ROM;

=head1 NAME

Option::ROM - Option ROM manipulation

=head1 SYNOPSIS

    use Option::ROM;

    # Load a ROM image
    my $rom = new Option::ROM;
    $rom->load ( "rtl8139.rom" );

    # Modify the PCI device ID
    $rom->pci_header->{device_id} = 0x1234;
    $rom->fix_checksum();

    # Write ROM image out to a new file
    $rom->save ( "rtl8139-modified.rom" );

=head1 DESCRIPTION

C<Option::ROM> provides a mechanism for manipulating Option ROM
images.

=head1 METHODS

=cut

##############################################################################
#
# Option::ROM::Fields
#
##############################################################################

package Option::ROM::Fields;

use strict;
use warnings;
use Carp;

sub TIEHASH {
  my $class = shift;
  my $self = shift;

  bless $self, $class;
  return $self;
}

sub FETCH {
  my $self = shift;
  my $key = shift;

  return undef unless $self->EXISTS ( $key );
  my $raw = substr ( ${$self->{data}},
		     ( $self->{offset} + $self->{fields}->{$key}->{offset} ),
		     $self->{fields}->{$key}->{length} );
  return unpack ( $self->{fields}->{$key}->{pack}, $raw );
}

sub STORE {
  my $self = shift;
  my $key = shift;
  my $value = shift;

  croak "Nonexistent field \"$key\"" unless $self->EXISTS ( $key );
  my $raw = pack ( $self->{fields}->{$key}->{pack}, $value );
  substr ( ${$self->{data}},
	   ( $self->{offset} + $self->{fields}->{$key}->{offset} ),
	   $self->{fields}->{$key}->{length} ) = $raw;
}

sub DELETE {
  my $self = shift;
  my $key = shift;

  $self->STORE ( $key, 0 );
}

sub CLEAR {
  my $self = shift;

  foreach my $key ( keys %{$self->{fields}} ) {
    $self->DELETE ( $key );
  }
}

sub EXISTS {
  my $self = shift;
  my $key = shift;

  return ( exists $self->{fields}->{$key} &&
	   ( ( $self->{fields}->{$key}->{offset} +
	       $self->{fields}->{$key}->{length} ) <= $self->{length} ) );
}

sub FIRSTKEY {
  my $self = shift;

  keys %{$self->{fields}};
  return each %{$self->{fields}};
}

sub NEXTKEY {
  my $self = shift;
  my $lastkey = shift;

  return each %{$self->{fields}};
}

sub SCALAR {
  my $self = shift;

  return 1;
}

sub UNTIE {
  my $self = shift;
}

sub DESTROY {
  my $self = shift;
}

sub checksum {
  my $self = shift;

  my $raw = substr ( ${$self->{data}}, $self->{offset}, $self->{length} );
  return unpack ( "%8C*", $raw );
}

##############################################################################
#
# Option::ROM
#
##############################################################################

package Option::ROM;

use strict;
use warnings;
use Carp;

=pod

=item C<< new () >>

Construct a new C<Option::ROM> object.

=cut

sub new {
  my $class = shift;

  my $hash = {};
  tie %$hash, "Option::ROM::Fields", {
    data => undef,
    offset => 0x00,
    length => 0x20,
    fields => {
      signature =>	{ offset => 0x00, length => 0x02, pack => "S" },
      length =>		{ offset => 0x02, length => 0x01, pack => "C" },
      checksum =>	{ offset => 0x06, length => 0x01, pack => "C" },
      undi_header =>	{ offset => 0x16, length => 0x02, pack => "S" },
      pci_header =>	{ offset => 0x18, length => 0x02, pack => "S" },
      pnp_header =>	{ offset => 0x1a, length => 0x02, pack => "S" },
    },
  };
  bless $hash, $class;
  return $hash;
}

=pod

=item C<< load ( $filename ) >>

Load option ROM contents from the file C<$filename>.

=cut

sub load {
  my $hash = shift;
  my $self = tied(%$hash);
  my $filename = shift;

  $self->{filename} = $filename;

  open my $fh, "<", $filename
      or croak "Cannot open $filename for reading: $!";
  read $fh, my $data, -s $fh;
  $self->{data} = \$data;
  close $fh;
}

=pod

=item C<< save ( [ $filename ] ) >>

Write the ROM data back out to the file C<$filename>.  If C<$filename>
is omitted, the file used in the call to C<load()> will be used.

=cut

sub save {
  my $hash = shift;
  my $self = tied(%$hash);
  my $filename = shift;

  $filename ||= $self->{filename};

  open my $fh, ">", $filename
      or croak "Cannot open $filename for writing: $!";
  print $fh ${$self->{data}};
  close $fh;
}

=pod

=item C<< pci_header () >>

Return a C<Option::ROM::PCI> object representing the ROM's PCI header,
if present.

=cut

sub pci_header {
  my $hash = shift;
  my $self = tied(%$hash);

  my $offset = $hash->{pci_header};
  return undef unless $offset != 0;

  return Option::ROM::PCI->new ( $self->{data}, $offset );
}

=pod

=item C<< pnp_header () >>

Return a C<Option::ROM::PnP> object representing the ROM's PnP header,
if present.

=cut

sub pnp_header {
  my $hash = shift;
  my $self = tied(%$hash);

  my $offset = $hash->{pnp_header};
  return undef unless $offset != 0;

  return Option::ROM::PnP->new ( $self->{data}, $offset );
}

=pod

=item C<< checksum () >>

Calculate the byte checksum of the ROM.

=cut

sub checksum {
  my $hash = shift;
  my $self = tied(%$hash);

  return unpack ( "%8C*", ${$self->{data}} );
}

=pod

=item C<< fix_checksum () >>

Fix the byte checksum of the ROM.

=cut

sub fix_checksum {
  my $hash = shift;
  my $self = tied(%$hash);

  $hash->{checksum} = ( ( $hash->{checksum} - $hash->checksum() ) & 0xff );
}

##############################################################################
#
# Option::ROM::PCI
#
##############################################################################

package Option::ROM::PCI;

use strict;
use warnings;
use Carp;

sub new {
  my $class = shift;
  my $data = shift;
  my $offset = shift;

  my $hash = {};
  tie %$hash, "Option::ROM::Fields", {
    data => $data,
    offset => $offset,
    length => 0x0c,
    fields => {
      signature =>	{ offset => 0x00, length => 0x04, pack => "L" },
      vendor_id =>	{ offset => 0x04, length => 0x02, pack => "S" },
      device_id =>	{ offset => 0x06, length => 0x02, pack => "S" },
      device_list =>	{ offset => 0x08, length => 0x02, pack => "S" },
      struct_length =>	{ offset => 0x0a, length => 0x02, pack => "S" },
      struct_revision =>{ offset => 0x0c, length => 0x01, pack => "C" },
      base_class => 	{ offset => 0x0d, length => 0x01, pack => "C" },
      sub_class => 	{ offset => 0x0e, length => 0x01, pack => "C" },
      prog_intf => 	{ offset => 0x0f, length => 0x01, pack => "C" },
      image_length =>	{ offset => 0x10, length => 0x02, pack => "S" },
      revision =>	{ offset => 0x12, length => 0x02, pack => "S" },
      code_type => 	{ offset => 0x14, length => 0x01, pack => "C" },
      last_image => 	{ offset => 0x15, length => 0x01, pack => "C" },
      runtime_length =>	{ offset => 0x16, length => 0x02, pack => "S" },
      conf_header =>	{ offset => 0x18, length => 0x02, pack => "S" },
      clp_entry =>	{ offset => 0x1a, length => 0x02, pack => "S" },
    },
  };
  bless $hash, $class;

  # Retrieve true length of structure
  my $self = tied ( %$hash );
  $self->{length} = $hash->{struct_length};

  return $hash;  
}

##############################################################################
#
# Option::ROM::PnP
#
##############################################################################

package Option::ROM::PnP;

use strict;
use warnings;
use Carp;

sub new {
  my $class = shift;
  my $data = shift;
  my $offset = shift;

  my $hash = {};
  tie %$hash, "Option::ROM::Fields", {
    data => $data,
    offset => $offset,
    length => 0x06,
    fields => {
      signature =>	{ offset => 0x00, length => 0x04, pack => "L" },
      struct_revision =>{ offset => 0x04, length => 0x01, pack => "C" },
      struct_length =>	{ offset => 0x05, length => 0x01, pack => "C" },
      checksum =>	{ offset => 0x09, length => 0x01, pack => "C" },
      manufacturer =>	{ offset => 0x0e, length => 0x02, pack => "S" },
      product =>	{ offset => 0x10, length => 0x02, pack => "S" },
      bcv =>		{ offset => 0x16, length => 0x02, pack => "S" },
      bdv =>		{ offset => 0x18, length => 0x02, pack => "S" },
      bev =>		{ offset => 0x1a, length => 0x02, pack => "S" },
    },
  };
  bless $hash, $class;

  # Retrieve true length of structure
  my $self = tied ( %$hash );
  $self->{length} = ( $hash->{struct_length} * 16 );

  return $hash;  
}

sub checksum {
  my $hash = shift;
  my $self = tied(%$hash);

  return $self->checksum();
}

sub fix_checksum {
  my $hash = shift;
  my $self = tied(%$hash);

  $hash->{checksum} = ( ( $hash->{checksum} - $hash->checksum() ) & 0xff );
}

1;
