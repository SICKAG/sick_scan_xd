#!/usr/bin/env bash
sudo mkdir /media/ramdisk
sudo chmod 777 /media/ramdisk
mount -F tmpfs swap /media/ramdisk
mount -v
set ROS_DISTRO=melodic
. /opt/ros/melodic/setup.bash
sudo mount -t tmpfs none /media/ramdisk
cd ~/catkin_sickag_devel_ws
source ./devel/setup.bash
touch /media/ramdisk/recording.txt
cd /media/ramdisk
rosbag record -a
cp /media/ramdisk/* /mnt/hgfs/development/ros
cd ~

