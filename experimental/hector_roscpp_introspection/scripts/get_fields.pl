#!/usr/bin/perl

my $msg_file = shift;
my $output = shift;

my $i = 0;
open(MSG, $msg_file) or die "Could not open $msg_file for reading";
while(<MSG>) {
  s/\#.*//;
  s/^\s*//;
  s/\s*$//;

  next if (/\=/);
  next if (length == 0);

  my ($type, $name) = split(/\s+/);

  unless (defined($name)) {
    print STDERR "parsing error: $_ has ".@fields." fields\n";
    next;
  }

  my $base = "SimpleField";
  my $size = 0;
  my $value_type = $type;
  my $if_array = "//";
  my $if_not_array = "";
  my $is_array_type = "false", $is_vector_type = "false", $is_container_type = "false";
  if ($type =~ /(.*)\[(\d*)\]$/) {
    $value_type = $1;
    $if_array = "";
    $if_not_array = "//";
    if ($2) {
      $size = $2;
      $is_array_type = "true";
      $is_container_type = "true";
    } else {
      $size = 0;
      $is_vector_type = "true";
      $is_container_type = "true";
    }
  }

  print <<END if $output eq '--declaration';
  struct field_$i : public Field {
    typedef CLASS::_${name}_type field_type;
    typedef boost::${is_array_type}_type is_array;
    typedef boost::${is_vector_type}_type is_vector;
    typedef boost::${is_container_type}_type is_container;
    $if_array typedef CLASS::_${name}_type::value_type value_type;

    field_$i(const Message& message) : Field(message) {}
    virtual ~field_$i() {}

    const char* getName() const { static const char *name = "$name"; return name; }
    const char* getDataType() const { static const char *data_type = "$type"; return data_type; }
    const char* getValueType() const { static const char *value_type = "$value_type"; return value_type; }
    std::size_t getIndex() const { return $i; }
    std::size_t getSize() const { return $size; }
    const std::type_info& getTypeId() const { return typeid(field_type); }

    bool isArray() const     { return is_array::value; }
    bool isVector() const    { return is_vector::value; }
    bool isContainer() const { return is_container::value; }
    bool isSimple() const    { return ros::message_traits::IsSimple<field_type>::value; }
    bool isFixedSize() const { return ros::message_traits::IsFixedSize<field_type>::value; }
    bool isMessage() const   { return ros::message_traits::IsMessage<field_type>::value; }

    field_type& reference(CLASS& instance) const { return instance.$name; }
    const field_type& reference(const CLASS& instance) const { return instance.$name; }

    FieldPtr access(Accessor& accessor) const { return impl::Accessor::access(*this, accessor); }
    FieldPtr access(ConstAccessor& accessor) const { return impl::ConstAccessor::access(*this, accessor); }
  };
END

  print <<END if $output eq '--constructor';
    add(FieldPtr(new fields::field_$i(*this)));
END

  $i++;
}
close(MSG);

