# Definition of ros message SickLocResultPortTestcaseMsg.
# SickLocResultPortTestcaseMsg publishes testcases for result port telegrams
# in both binary format (simulated result port telegrams from localization controller)
# and ros message format of type SickLocResultPortTelegramMsg (published by sim_loc_driver).
# See chapter 5.9 ("About result port telegrams") of the operation manual for further details.

std_msgs/Header header                             # ROS Header with sequence id, timestamp and frame id
uint8[]                      binary_data  # binary encoded result port telegram
SickLocResultPortTelegramMsg telegram_msg # decoded result port telegram
