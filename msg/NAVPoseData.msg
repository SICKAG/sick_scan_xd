# ROS message of NAV350PoseData, see sick_nav_scandata_parser.h and NAV-350 telegram listing for details.
std_msgs/Header header

# pose in lidar coordinates in mm and mdeg
int32 x
int32 y
uint32 phi
# optional pose data
uint16 opt_pose_data_valid
uint8 output_mode
uint32 timestamp
int32 mean_dev
uint8 nav_mode
uint32 info_state
uint8 quant_used_reflectors

# pose in ros units
int8 pose_valid   # pose_x, pose_y and pose_yaw are valid if pose_valid > 0
float32 pose_x    # x position in ros units in m
float32 pose_y    # y position in ros units in m
float32 pose_yaw  # yaw angle in radians
