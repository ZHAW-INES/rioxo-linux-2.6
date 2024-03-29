# This script generates an appropriate hardware.h file for Nios II Linux based
# on information within the target hardware's system.ptf file.  This script
# outputs everything to stdout.
#
# usage:
#
#  [SOPC Builder]$ perl gen_hardware.h.pl <target cpu> <exec location>

use PTF::SystemPTF;
use strict;
use integer;

my $target_cpu;
my $exec_location;

if (scalar (@ARGV) < 2) {
	print STDERR "ERROR: Invalid number of parameters.\n";
	print ("#error Invalid number of parameters.\n");
	exit;
} else {
	$target_cpu = $ARGV[0];
	$exec_location = $ARGV[1];
}

#
# startup the parser.
#
my $system = SystemPTF->new;
if (!$system) {
	print STDERR "ERROR: Specified file is not a SYSTEM ptf file.\n";
	print ("#error Specified file is not a SYSTEM ptf file.\n");
	exit;
}

#
# print header for nios2.h
#
print <<ENDOFHEADER;
#ifndef __NIOS2_H__
#define __NIOS2_H__

/*
 * This file contains hardware information about the target platform.
 *
 * This file is automatically generated.  Do not modify.
 */
   
ENDOFHEADER

#
# generate contents for nios2.h
#
my $result; # dummy variable
my $cpu = $system->getCPU ($target_cpu);
if (! $cpu) {
	print STDERR "ERROR: $target_cpu is not a valid CPU in system: " . $system->getName () . ".\n";
	print "#error $target_cpu is not a valid CPU in system: " . $system->getName () . ".\n";
	exit 1;
}

my $exec_module = $system->getModule ($exec_location);
if (! $exec_module) {
	print STDERR "ERROR: $exec_location is not a valid module in the system: " . $system->getName() . ".\n";
	print "#error $exec_location is not a valid module in system: " . $system->getName () . ".\n";
	exit 1;
}

my %found_classes;
my @found_classes_order;

# the SYSPTF environment variable is set by kernel build process.
if ($ENV{SYSPTF} ne "") {
	print "/* Input System: " . $ENV{SYSPTF} .  ":" . $system->getName () . " */\n";
} else {
	print "/* Input System: " . $system->getName () . " */\n";
}
print "/* Target CPU: " . $target_cpu . " */\n";

print "\n";

print <<ENDOFCONSTANTS;
/* Nios II Constants */
#define NIOS2_STATUS_PIE_MSK  0x1
#define NIOS2_STATUS_PIE_OFST 0
#define NIOS2_STATUS_U_MSK    0x2
#define NIOS2_STATUS_U_OFST   1
ENDOFCONSTANTS

print "\n";

print "/*\n";
print " * Outputting basic values from system.ptf.\n";
print " */\n\n";

#
# Start outputing information about each module.
#
my @module_names = $system->getSlaveModules ($target_cpu);
foreach my $module_name (@module_names) {
	my $module = $system->getModule ($module_name);
	my $module_class = $module->getClass ();
	my @module_ports = $module->getPorts ();
	my $mask = 0;
	my $text_printed = 0;
	my $output = "";

	# $output .= "/* $module_name (of type $module_class) */\n";

	if (! exists $found_classes{$module_class}) {
		push @found_classes_order, $module_class;
	}
	push @{$found_classes{$module_class}}, $module_name;
	
	if (scalar (@module_ports) > 0) {
		my $base_address;
		my $mem_size;
		my $mem_end;
		
		# base address information 
		$base_address = $module->getBaseAddress ();
		if ($base_address) {
			$output .= sprintf ("#define na_%-50s %#010x\n", 
				($module_name, hex ($base_address) | $mask));
			$text_printed = 1;
		}
		if ($module->isMemoryDevice()) {
			# output size and end address
			$mem_size = $module->getSize();
			$output .= sprintf ("#define na_%-50s %#010x\n",
				($module_name . "_size", hex ($mem_size)));
			$mem_end = hex ($mem_size) + hex($base_address);
			$output .= sprintf ("#define na_%-50s %#010x\n",
				($module_name . "_end", $mem_end));

			$text_printed = 1;
		}

		# irq information 
		$result = $module->getIRQ ();
		if (defined ($result)) {
			$output .= sprintf ("#define na_%-30s %30s\n", 
				($module_name . "_irq", $result));
			$text_printed = 1;
		}

		# clock information 
		$result = $system->getClockFreq($module_name);
		if (defined ($result)) {
			$output .= sprintf ("#define na_%-30s %30s\n", 
				($module_name . "_clock_freq", $result));
			$text_printed = 1;
		}

	}
	if (scalar (@module_ports) > 1) {
		# if device has multiple ports
		foreach my $port_name (@module_ports) {
			# base address information
			$result = $module->getBaseAddress ($port_name);
			if ($result) {
			$output .= sprintf ("#define na_%-50s %#010x\n", 
				($module_name . "_" . $port_name, hex ($result) | $mask));
				$text_printed = 1;
			}

			# irq information
			$result = $module->getIRQ ($port_name);
			if (defined ($result)) {
			$output .= sprintf ("#define na_%-30s %30s\n", 
				($module_name . "_" . $port_name . "_irq", $result));
				$text_printed = 1;
			}
		}
	}

	if ($text_printed == 1) {
		# $output .= "\n";
		print $output;
	}
}

print "\n";

#
# Handle special cases through customized perl scripts
#
foreach my $class_name (@found_classes_order) {
	my $code = "";
	
	foreach my $dir (@INC) {
		if (-e "$dir/nios2.h/$class_name.pm") {
			print "/* Executing ...scripts/nios2.h/$class_name.pm */\n";
			$code .= "require \"$dir/nios2.h/BasicModule.pm\";";
			$code .= "require \"$dir/nios2.h/$class_name.pm\";";
			$code .= $class_name . "::run(\$system, \@{\$found_classes{\$class_name}});";
			eval $code;
			if ($@) {
				print "#warning Could not execute ...scripts/nios2.h/$class_name.pm\n";
				print "#warning Error message is stored in nios2.h:\n";
				print "/*\n";
				print "$@";
				print "*/\n";
				print STDERR "Could not execute ...scripts/nios2.h/$class_name.pm\n";
				print STDERR "Error message follows:\n";
				print STDERR "$@";
			} 
			last;
		}
	}
}

#
# Write out system information
#
print "/*\n";
print " * Basic System Information\n";
print " */\n";

$result = $cpu->getWSAAssignment ('cache_icache_size');
printf ("#define %-53s %10d\n", ("nasys_icache_size", $result));

$result = $cpu->getConstant ('nasys_icache_line_size');
printf ("#define %-53s %10d\n", ("nasys_icache_line_size", $result));

$result = $cpu->getWSAAssignment ('cache_dcache_size');
printf ("#define %-53s %10d\n", ("nasys_dcache_size", $result));

$result = $cpu->getConstant ('nasys_dcache_line_size');
printf ("#define %-53s %10d\n", ("nasys_dcache_line_size", $result));

print "\n";

printf ("#define %-33s %30s\n", 
	("nasys_program_mem", "na_${exec_location}"));
printf ("#define %-33s %30s\n", 
	("nasys_program_mem_size", "na_${exec_location}_size"));
printf ("#define %-33s %30s\n", 
	("nasys_program_mem_end", "na_${exec_location}_end"));
	
print "\n";

print "\n";
printf ("#define %-33s %30s\n", 
	("na_cpu_clock_freq", $system->getClockFreq($target_cpu)));
	
{	
	my ($reset_location, $reset_offset) = $cpu->getResetLocationOffset();
	my ($reset_module_name, $reset_port_name) = ($reset_location =~ /(.*)\/(.*)/);
	my $reset_module = $system->getModule ($reset_module_name);
	my $reset_address = $reset_module->getBaseAddress ($reset_port_name);
	
	$reset_address = hex ($reset_address) + hex ($reset_offset);
	printf ("#define %-53s %#010x\n", 
		("CPU_RESET_ADDRESS", $reset_address));
}

{	
	my ($except_location, $except_offset) = $cpu->getExceptLocationOffset();
	my ($except_module_name, $except_port_name) = ($except_location =~ /(.*)\/(.*)/);
	my $except_module = $system->getModule ($except_module_name);
	my $except_address = $except_module->getBaseAddress ($except_port_name);
	
	$except_address = hex ($except_address) + hex ($except_offset);
	printf ("#define %-53s %#010x\n", 
		("CPU_EXCEPT_ADDRESS", $except_address));
}

print "\n";

#
# print footer for nios2.h
#
print <<ENDOFFOOTER;
#endif /* __NIOS2_H__ */
ENDOFFOOTER
