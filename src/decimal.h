// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Decimal numbers as JSON writes them, converted to binary32 or binary64
// correctly rounded (half to even), or to whole numbers, with integer
// arithmetic only: the same bits under any floating-point environment,
// and no C library.

#ifndef MAUL_ANIM_SRC_DECIMAL_H
#define MAUL_ANIM_SRC_DECIMAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The binary formats a decimal converts to.
typedef enum maniBinaryFormat
{
    maniBinary32 = 0,
    maniBinary64 = 1,
} maniBinaryFormat;

// Converts a JSON number (RFC 8259's grammar, already checked) to the
// format's bits. False when the value's magnitude rounds past the
// format's largest finite value; values too small round to a signed 0.
bool maniDecimalToBinary(const char* text, size_t length, maniBinaryFormat format,
                         uint64_t* bitsOut);

// Converts a JSON number to a whole number in [minimum, maximum]. False
// for a value with a non-zero fraction or outside the range; exponents
// and zero fractions are allowed (1e2, 3.0), and -0 is 0.
bool maniDecimalToInteger(const char* text, size_t length, int64_t minimum, int64_t maximum,
                          int64_t* valueOut);

#endif // MAUL_ANIM_SRC_DECIMAL_H
