# sudo mkdir /media/ramdisk
# sudo mount -t tmpfs none /media/ramdisk
# rosbag record -a /media/ramdisk
set ROS_DISTRO=melodic
. /opt/ros/melodic/setup.bash
cd catkin_sickag_devel_ws/
source ./devel/setup.bash
roslaunch sick_scan test_002_combi_live_traffic.launch
cd ~


