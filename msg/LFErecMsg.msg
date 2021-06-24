# This message contains a single field of a "sSN LFErec" message of a Sick laser scanner as an ros message.
# See https://github.com/SICKAG/libsick_ldmrs/blob/master/src/sopas/LdmrsSopasLayer.cpp lines 1414 ff.
# Field result field_result_mrs is 0 (invalid/incorrect), 1 (free/clear) or 2 (infringed)
# Note: field indices are sorted in reverse order, i.e. with 2 configured fields, we typically have
# output_msg.fields[0].field_index = 1, output_msg.fields[0].field_result_mrs = 0 (invalid)
# output_msg.fields[1].field_index = 2, output_msg.fields[1].field_result_mrs = 1 or 2 (clear=1 or infringed=2)
# output_msg.fields[2].field_index = 3, output_msg.fields[2].field_result_mrs = 1 or 2 (clear=1 or infringed=2)

Header header

uint16 fields_number # number of fields
sick_scan/LFErecFieldMsg[] fields # 3 fields for TiM871S
