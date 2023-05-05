# ROS message for NAV350 velocity/odometry data, see sick_nav_scandata_parser.h and NAV-350 telegram listing for details ("sMN mNPOSSetSpeed").

float32 vel_x # x-component of velocity in the coordinate system defined by coordbase (i.e. in lidar coordinate for coordbase=0) in m/s, -32.0 ... +32.0 m/s
float32 vel_y # y-component of velocity in the coordinate system defined by coordbase (i.e. in lidar coordinate for coordbase=0) in m/s, -32.0 ... +32.0 m/s
float32 omega # angular velocity of the NAV350 in radians/s, -2*PI ... +2*PI rad/s
uint32 timestamp # Timestamp of the Velocity vector related to the NAV350 clock
uint8 coordbase # Coordinate system of the velocity vector (local or global), 0 = local coordinate system of the NAV350, 1 = absolute coordinate system
