# Out-of-context implementation probe.
#
#   vivado -mode batch -nolog -nojournal -source synth/probe.tcl \
#          -tclargs <top> <tag> [GEN=VAL ...] [-files <path> ...]
#
# Reads every non-blank line of synth/files.f (paths relative to the repo root;
# run from the repo root) plus any extra files given after -files, synthesizes
# <top> out of context with the given generics, applies a 1.111 ns clock on
# `clk`, 0 ns input/output delays, runs opt/place/phys_opt/route with default
# directives and prints exactly one result line:
#   PROBE <top> <tag> WNS=<x> WHS=<y> WPWS=<z> RESULT=PASS|FAIL
# Reports go to synth/out/probe_<top>_<tag>/.

set part   xcu250-figd2104-2L-e
set period 1.111

if {[llength $argv] < 2} {
  puts "usage: -tclargs <top> <tag> \[GEN=VAL ...\] \[-files <path> ...\]"
  exit 2
}
set top [lindex $argv 0]
set tag [lindex $argv 1]
set generics {}
set extra {}
set in_files 0
foreach a [lrange $argv 2 end] {
  if {$a eq "-files"} {
    set in_files 1
  } elseif {[string first "=" $a] >= 0} {
    lappend generics $a
    set in_files 0
  } elseif {$in_files} {
    lappend extra $a
  } else {
    puts "ERROR: unexpected argument '$a'"
    exit 2
  }
}

set root [file normalize [file join [file dirname [file normalize [info script]]] ..]]
set outdir [file join $root synth out probe_${top}_${tag}]
file mkdir $outdir

# ---- sources ---------------------------------------------------------------
set fh [open [file join $root synth files.f] r]
set files {}
while {[gets $fh line] >= 0} {
  set line [string trim $line]
  if {$line eq ""} continue
  lappend files $line
}
close $fh
foreach f [concat $files $extra] {
  if {[file pathtype $f] eq "relative"} { set f [file join $root $f] }
  read_verilog -sv $f
}

# ---- synthesis -------------------------------------------------------------
set synth_args [list -top $top -part $part -mode out_of_context]
foreach g $generics { lappend synth_args -generic $g }
synth_design {*}$synth_args

create_clock -name clk -period $period [get_ports clk]
set data_in  [get_ports -filter {DIRECTION == IN} *]
set data_in  [lsearch -all -inline -not -exact $data_in [get_ports clk]]
# -min 0.2: models an upstream register's clk->Q plus a short route, so port->FF hold paths are realistic.
if {[llength $data_in]} {
  set_input_delay -clock clk -max 0.0 $data_in
  set_input_delay -clock clk -min 0.2 $data_in
}
set data_out [get_ports -filter {DIRECTION == OUT} *]
if {[llength $data_out]} { set_output_delay -clock clk 0 $data_out }

# ---- implementation --------------------------------------------------------
opt_design
place_design
phys_opt_design
route_design

write_checkpoint -force [file join $outdir post_route.dcp]
report_utilization    -file [file join $outdir utilization.rpt]
report_timing_summary -file [file join $outdir timing_summary.rpt] -max_paths 10

# ---- slack extraction ------------------------------------------------------
# Any value that cannot be determined stays "NA" and makes the probe FAIL, so a
# missing path / table can never be mistaken for a pass.
proc first_slack {args} {
  set p [get_timing_paths {*}$args -max_paths 1]
  if {[llength $p] == 0} { return "NA" }
  set v [get_property SLACK $p]
  if {$v eq ""} { return "NA" }
  return $v
}
set wns [first_slack -setup]
set whs [first_slack -hold]

# WPWS: parse the "Design Timing Summary" table of the timing summary. Its data
# row starts with 10 numbers: WNS TNS #fail #total WHS THS #fail #total WPWS TPWS (more follow).
# The scan stops at the first blank line after the header (end of the table).
set wpws "NA"
set summary [report_timing_summary -return_string -max_paths 1]
set lines [split $summary "\n"]
for {set i 0} {$i < [llength $lines]} {incr i} {
  if {[string first "WPWS(ns)" [lindex $lines $i]] >= 0} {
    for {set j [expr {$i + 1}]} {$j < [llength $lines]} {incr j} {
      set vals [regexp -all -inline {\S+} [lindex $lines $j]]
      if {[llength $vals] == 0} break
      if {[llength $vals] >= 10 && [string is double -strict [lindex $vals 0]]} {
        set wpws [lindex $vals 8]
        break
      }
    }
    break
  }
}

# PASS iff all three are real numbers >= 0.
set pass 1
foreach v [list $wns $whs $wpws] {
  if {![string is double -strict $v] || $v < 0} { set pass 0 }
}
puts [format "PROBE %s %s WNS=%s WHS=%s WPWS=%s RESULT=%s" $top $tag $wns $whs $wpws [expr {$pass ? "PASS" : "FAIL"}]]
exit [expr {$pass ? 0 : 1}]
