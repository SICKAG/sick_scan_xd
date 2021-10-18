#!/bin/bash
#
# Stops the communication app started by roslaunch.
# After stopping you can start your debug ide to start sick_generic_caller in the ros environment. 
#
# For debugging after killing the "original" sick_generic_caller-instance do the following:
# ===========================================================================
# 1. Edit the calling argument in your ide to the following:
#    Arguments in the IDE-Debugger: __name:=<scanner_type> __log:=/tmp/debuglog.log
#    e.g. sick_generic_caller __name:=sick_mrs_1xxx __log:=/tmp/debuglog.log
# 2. Start debugging in your ide.
#
# Alternative way of killing:
# pkill -e -f sick_generic_caller
# or for other tools as cut&paste template:
# pkill -e -f radar_object_marker
#
echo "Killing all sick_generical_caller instances"
for KILLPID in `ps ax | grep 'sick_generic_caller' | grep -v 'grep' | awk ' { print $1;}'`; do
  kill -9 $KILLPID
done
