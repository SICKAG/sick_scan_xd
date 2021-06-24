# Definition of ros message SickLocColaTelegramMsg.
# SickLocColaTelegramMsg publishes Cola-ASCII telegrams for sick localization.
# See Technical_information_Telegram_Listing_NAV_LOC_en_IM0076556.PDF for further details
# about Cola telgrams

std_msgs/Header header        # ROS Header with sequence id, timestamp and frame id

int32 command_type   # One of the SOPAS Commands sRN (1, Read by name request) or sRA (2, Read by name response) or sMN (3, Method by name request) or sMA (4, Method by name response) or sWN (5, Write by name request)

string command_name  # Name of command like "SetAccessMode", "LocSetResultPoseEnabled", "LocRequestTimestamp", etc.

string[] parameter   # Optional parameter, type and number parameter depend on the command, f.e. "sMN SetAccessMode 3 F4724744" with 2 parameter
