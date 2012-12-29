#!/usr/bin/perl

use Device::SerialPort;
my $port = new Device::SerialPort("/dev/ttyUSB0") || die "Can't open $port: $!\n";

$port->user_msg(ON); 
$port->baudrate(38400); 
$port->parity("none"); 
$port->databits(8); 
$port->stopbits(1); 
$port->handshake("xoff"); 
$port->write_settings;


$port->lookclear; 
$output_string = "hello from perl\r\n";
$count_out = $port->write($output_string);
  warn "write failed\n"         unless ($count_out);
  warn "write incomplete\n"     if ( $count_out != length($output_string) );

# $port->write("some command to com-port");

my $answer = $port->lookfor;
my $answer=$port->read(255);
