# Definition of ros message SickLocResultPortTelegramMsg.
# SickLocResultPortTelegramMsg publishes result port telegrams
# for sick localization. See chapter 5.9 ("About result port telegrams")
# of the operation manual for further details.

#
# Header
#

std_msgs/Header header                                 # ROS Header with sequence id, timestamp and frame id

#
# Result port telegram
#

SickLocResultPortHeaderMsg   telegram_header  # 52 byte header of a result port telegram
SickLocResultPortPayloadMsg  telegram_payload # 52 byte payload of a result port telegram
SickLocResultPortCrcMsg      telegram_trailer #  2 byte CRC trailer of a result port telegram

#
# System time of vehicles pose calculated by ros service "SickLocTimeSync" using a software pll
#

bool   vehicle_time_valid  # true: vehicle_time_sec and vehicle_time_nsec valid, false: software pll still in initial phase
uint32 vehicle_time_sec    # Time of vehicles pose calculated by software pll (seconds part of the system time)
uint32 vehicle_time_nsec   # Time of vehicles pose calculated by software pll (nano seconds part of the system time)
