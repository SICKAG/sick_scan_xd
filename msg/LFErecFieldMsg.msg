# This message contains a single field of a "sSN LFErec" message of a Sick laser scanner as an ros message.
# See https://github.com/SICKAG/libsick_ldmrs/blob/master/src/sopas/LdmrsSopasLayer.cpp lines 1414 ff.
# Field result field_result_mrs is 0 (invalid/incorrect), 1 (free/clear) or 2 (infringed)
# Note: field indices are sorted in reverse order, i.e. with 2 configured fields, we typically have
# output_msg.fields[0].field_index = 1, output_msg.fields[0].field_result_mrs = 0 (invalid)
# output_msg.fields[1].field_index = 2, output_msg.fields[1].field_result_mrs = 1 or 2 (clear=1 or infringed=2)
# output_msg.fields[2].field_index = 3, output_msg.fields[2].field_result_mrs = 1 or 2 (clear=1 or infringed=2)

uint16 version_number
uint8 field_index
uint32 sys_count
float32 dist_scale_factor
float32 dist_scale_offset
uint32 angle_scale_factor
int32 angle_scale_offset
uint8 field_result_mrs # field result is 0 (invalid/incorrect), 1 (free/clear) or 2 (infringed)
# Time block
uint16 time_state  # No time data: 0, Time data: 1
uint16 year        # f.e. 2021
uint8 month        # 1 ... 12
uint8 day          # 1 ... 31
uint8 hour         # 0 ... 23
uint8 minute       # 0 ... 59
uint8 second       # 0 ... 59
uint32 microsecond # 0 ... 999999
