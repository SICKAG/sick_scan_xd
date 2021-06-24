# This message contains the IMU data of a Sick laser scanner as an IMU message.
# The ticks correspond to the internal time stamps of the laser scanner and increase linearly with time, the tickcounter overflows.
# The quaternion_accuracy has the unit rad.
# The accuracy measures for Lienar acceleration and angular velocity are abitre values, where higher values are better.
# 
#
Header header

sensor_msgs/Imu Imu
#IMU Message

uint32 ticks
# timestamp Ticks from laser scanner

float32 quaternion_accuracy
#quaternion accuracy in rad

uint8 angular_velocity_reliability
#angular velocity reliability 0 low 255 high, value should be over 3

uint8 linear_acceleration_reliability
#linear acceleration reliability 0 low 255 high, value should be over 3
