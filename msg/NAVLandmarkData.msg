# ROS message of NAV350LandmarkData, see sick_nav_scandata_parser.h and NAV-350 telegram listing for details.
std_msgs/Header header

uint8 landmark_filter
uint16 num_reflectors
sick_scan_xd/NAVReflectorData[] reflectors
