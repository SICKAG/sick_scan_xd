# Definition of ros message SickLocDiagnosticMsg.
# SickLocDiagnosticMsg publishes diagnostic messages
# of the sick localization driver.

std_msgs/Header header      # ROS Header with sequence id, timestamp and frame id
int32  error_code  # error code (0 := no error)
string message     # diagnostic message
