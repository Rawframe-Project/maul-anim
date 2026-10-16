// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Binary32 values written as decimals, as ECMAScript writes a Number but
// for binary32 (mani-0023): the fewest significant digits that read back
// to the same value, with integer arithmetic only. Offline.

#ifndef MAUL_ANIM_SRC_DECIMAL_FORMAT_H
#define MAUL_ANIM_SRC_DECIMAL_FORMAT_H

#include <stddef.h>

// Room for the longest text a binary32 takes: a sign, nine digits, a
// point and an exponent of up to two digits with its signs, or a plain
// number of up to 21 digits, or "0." with five zeros and nine digits.
#define MANI_DECIMAL_MAX 48u

// Writes a finite value into out (not terminated); returns its length,
// 0 for a value not finite. Negative zero is "-0".
size_t maniFormatBinary32(float value, char out[MANI_DECIMAL_MAX]);

#endif // MAUL_ANIM_SRC_DECIMAL_FORMAT_H
