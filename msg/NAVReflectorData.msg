# ROS message of NAV350ReflectorData, see sick_nav_scandata_parser.h and NAV-350 telegram listing for details.

# cartesian data in lidar coordinates in mm
uint16 cartesian_data_valid
int32 x
int32 y

# polar data in lidar coordinates in mm and mdeg
uint16 polar_data_valid
uint32 dist
uint32 phi

# optional reflector data
uint16 opt_reflector_data_valid
uint16 local_id
uint16 global_id
uint8 type 
uint16 sub_type
uint16 quality
uint32 timestamp
uint16 size
uint16 hit_count
uint16 mean_echo
uint16 start_index
uint16 end_index

# reflector position in ros coordinates
int8 pos_valid   # pose_x, pose_y and pose_yaw are valid if pose_valid > 0
float32 pos_x    # x position in ros coordinates in m
float32 pos_y    # y position in ros coordinates in m
