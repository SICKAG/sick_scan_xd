#!/bin/bash
#
#  Setting up a new catkin-workspace, checkout devel version of sick_scan
#  and try to build the software.
#
RED='\033[0;31m'
NC='\033[0m' # No Color
instDir=~/AAA_debug
hostname=192.168.0.1
echo "+============================================================+"
echo "| Installing sick_scan and hector_slam as a testbed for SLAM "
echo -e "| ${RED}Expected ip-address of laser scanner MRS1104 $hostname ${NC}"
echo -e "| ROS-Workspace used for testbed: ${RED} $instDir  ${NC}                   "
echo "+============================================================+"
echo "... Installation starts in 5 secs. Please wait ..."
sleep 5
mkdir -p $instDir/src
cd $instDir
catkin_make
source devel/setup.bash
cd src
# git clone -b devel https://github.com/SICKAG/sick_scan.git
git clone https://github.com/SICKAG/sick_scan.git
cd ..
catkin_make






