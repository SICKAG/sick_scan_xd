#This message contains the counter reading of the encoder at the datagram time (timestamp in header).

# ROS-1: 
#Header header           # same information as in laserscan and Pointcloud2 message
# ROS-2: 
std_msgs/Header header           # same information as in laserscan and Pointcloud2 message

int32 enc_position     # number of ticks
int16 enc_speed         # number of ticks per millimeter
