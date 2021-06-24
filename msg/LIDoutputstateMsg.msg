# This message contains the "sSN LIDoutputstate" message of a Sick laser scanner as an ros message.
#

Header header

# Status code
uint16 version_number # Status code version number
uint32 system_counter # Status code system counter (time in microsec since power up, max. 71 min then starting from 0 again)

# Array of output states (state and count)
uint8[] output_state  # output states, each state with value 0 (not active), 1 (active) or 2 (not used)
uint32[] output_count # output counter

# Time block (sensortime from the last change of min. one of the outputs)
uint16 time_state   # No time data: 0, Time data: 1
uint16 year        # f.e. 2021
uint8 month        # 1 ... 12
uint8 day          # 1 ... 31
uint8 hour         # 0 ... 23
uint8 minute       # 0 ... 59
uint8 second       # 0 ... 59
uint32 microsecond # 0 ... 999999
