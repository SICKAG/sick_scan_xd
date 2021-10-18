# Definition of ros message SickLocResultPortCrcMsg.
# SickLocResultPortCrcMsg publishes the CRC trailer of a result port telegram
# for sick localization (2 byte). 
# Checksum details:
# * Width: 16 bits
# * Truncated polynomial: 0x1021 CRC polynomials with orders of x16 + x12 + x5 + 1 (counted without the leading '1' bit)
# * Initial value = 0xFFFF
# See chapter 5.9 ("About result port telegrams") of the operation manual for further details.

uint16 checksum # CRC16-CCITT over length of header (52 bytes) and payload (52 bytes) without 2 bytes of this trailer. Size: UInt16 = 2 byte

