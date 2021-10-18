# Definition of ros message SickLocResultPortHeaderMsg.
# SickLocResultPortHeaderMsg publishes the header of a result port telegram
# for sick localization (52 byte). See chapter 5.9 ("About result port telegrams")
# of the operation manual for further details.

uint32  magicword        # Magic word SICK (0x53 0x49 0x43 0x4B). Size: 4 × UInt8 = 4 byte
uint32  length           # Length of telegram incl. header, payload, and trailer. Size: UInt32 = 4 byte
uint16  payloadtype      # Payload type: 0x06c2 = Little Endian, 0x0642 = Big Endian. Size: UInt16 = 2 byte
uint16  payloadversion   # Version of PayloadType structure. Size: UInt16 = 2 byte
uint32  ordernumber      # Order number of the localization controller. Size: UInt32 = 4 byte
uint32  serialnumber     # Serial number of the localization controller. Size: UInt32 = 4 byte
uint8[] fw_version       # Software version of the localization controller. Size: 20 × UInt8 = 20 byte
uint32  telegramcounter  # Telegram counter since last start-up. Size: UInt32 = 4 byte
uint64  systemtime       # Not used. Size: NTP = 8 byte

