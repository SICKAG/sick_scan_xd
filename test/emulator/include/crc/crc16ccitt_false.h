/*
 * CRC checksums implemented by https://github.com/madler/crcany
 * Sources under the zlib license, permitting free commercial use.
 * See https://github.com/madler/crcany and http://zlib.net/zlib_license.html
 * for further details.
 *
 * This project uses algorithm "CRC-16/CCITT-FALSE" (crc16ccitt_false.c and crc16ccitt_false.h).
 * Other crc checksum algorithms may be used if required.
 */

// The _bit, _byte, and _word routines return the CRC of the len bytes at mem,
// applied to the previous CRC value, crc. If mem is NULL, then the other
// arguments are ignored, and the initial CRC, i.e. the CRC of zero bytes, is
// returned. Those routines will all return the same result, differing only in
// speed and code complexity. The _rem routine returns the CRC of the remaining
// bits in the last byte, for when the number of bits in the message is not a
// multiple of eight. The high bits bits of the low byte of val are applied to
// crc. bits must be in 0..8.

#include <stddef.h>

// Compute the CRC a bit at a time.
unsigned crc16ccitt_false_bit(unsigned crc, void const *mem, size_t len);

// Compute the CRC of the high bits bits in the low byte of val.
unsigned crc16ccitt_false_rem(unsigned crc, unsigned val, unsigned bits);

// Compute the CRC a byte at a time.
unsigned crc16ccitt_false_byte(unsigned crc, void const *mem, size_t len);

// Compute the CRC a word at a time.
unsigned crc16ccitt_false_word(unsigned crc, void const *mem, size_t len);
