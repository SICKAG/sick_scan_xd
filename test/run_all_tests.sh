#!/bin/sh
#
# Test for ROS sick_scan driver
#
# Step #1 check for a given argument. As argument we accept a valid ip address of one scanner.
#         If no argument is given all xml test files are parsed and the associated scanner is checked.
#
echo "The number of arguments is: $#"
echo "With no arguments the test script tries to connect to each scanner type given by the xml file"
givenIpAddress=""
# set -x - only for debugging script
if [ $# -gt 0 ]
then
  givenIpAddress=$1
  echo "Checking only scanner connected to ip-address $givenIpAddress"
else
  echo "Checking all scanners"
fi

for i in *.xml; do
    [ -f "$i" ] || break
    basename=$(basename "$i")
    filename='test/'$basename
    ipaddress=$( grep -E -o "([0-9]{1,3}[\\.]){3}[0-9]{1,3}" "$i")
    doChecking=false
    if [ "$givenIpAddress" = "" ]
    then
        doChecking=true
    else
        if [ "$givenIpAddress" = "$ipaddress" ]; then
          doChecking=true
        fi
    fi
    if [ "$doChecking" == true ]
    then
     echo "Checking $ipaddress"
     ping -c 1 "$ipaddress"
     if [ "$?" -eq 0 ]
     then
       echo 'scanner is alive Starting Test'
       yes 2| rosrun sick_scan sick_scan_test "$filename"
     else
     echo 'scanner is not reachable skipping test'
     echo "$filename"
     echo 'Skipped'
     fi
     echo 'sleeping 5 seconds before starting next Test'
     sleep 5
    fi
done
# switch off debugging: set +x

