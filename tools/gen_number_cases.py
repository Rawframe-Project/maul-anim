#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The JSON number reader's oracle: decimal texts and their correctly
# rounded binary32 and binary64 bits, from exact rationals (round half to
# even, subnormals, overflow marked), written as a C table for
# test_json.c. Deterministic: a fixed seed.
#
# usage: gen_number_cases.py > test/json_number_cases.h
#        gen_number_cases.py --check   (the header matches)

import os
import random
import sys
import struct
from fractions import Fraction

FORMATS = {32: (23, 8), 64: (52, 11)}


def round_to(value, bits):
    """Bits of the correctly rounded value, or None on overflow."""
    mant, exp_bits = FORMATS[bits]
    bias = (1 << (exp_bits - 1)) - 1
    sign = 0
    if value < 0:
        sign, value = 1, -value
    if value == 0:
        return sign << (bits - 1)
    # e with 2^e <= value < 2^(e+1).
    e = value.numerator.bit_length() - value.denominator.bit_length()
    if Fraction(2) ** e > value:
        e -= 1
    e = max(e, 1 - bias)  # subnormals share the least exponent
    scaled = value / Fraction(2) ** (e - mant)
    q, r = divmod(scaled.numerator, scaled.denominator)
    twice = 2 * r
    if twice > scaled.denominator or (twice == scaled.denominator and q % 2 == 1):
        q += 1
    if q == 1 << (mant + 1):
        q >>= 1
        e += 1
    if e > bias:
        return None
    if q < 1 << mant:  # subnormal
        field = 0
    else:
        field = e + bias
        q -= 1 << mant
    return (sign << (bits - 1)) | (field << mant) | q


# Exponents past this are not evaluated as rationals (10^(10^20) has no
# room): a nonzero mantissa overflows above it and is zero below it.
SATURATED = 100000


def exponent_of(text):
    at = max(text.find("e"), text.find("E"))
    return int(text[at + 1:]) if at >= 0 else 0


def exact(text):
    """The text's value, or "overflow" for a saturating exponent."""
    e = exponent_of(text)
    if abs(e) > SATURATED:
        mantissa = Fraction(text[:max(text.find("e"), text.find("E"))])
        if mantissa == 0 or e < 0:
            return Fraction(0)
        return "overflow"
    return Fraction(text)


def cases():
    rnd = random.Random(0x6A736F6E)
    out = ["0", "-0", "1", "-1", "0.5", "1e0", "1E+2", "1e-2", "123456789",
           "3.4028234663852886e38", "3.4028235677973366e38", "3.4028235677973367e38",
           "1.401298464324817e-45", "7.006492321624085e-46", "7.006492321624086e-46",
           "1.1754942807573643e-38", "1.7976931348623157e308", "1.7976931348623158e308",
           "1.797693134862315807e308", "4.9406564584124654e-324", "2.4703282292062327e-324",
           "2.4703282292062328e-324", "2.2250738585072011e-308", "2.2250738585072012e-308",
           "0.1", "0.2", "0.3", "1e23", "8.589973e9", "9007199254740993",
           "9007199254740992.5", "16777217", "16777217.000000001", "33554435",
           "0.000000000000000000000000000000000000000000001",
           "1" + "0" * 400, "0." + "0" * 400 + "1", "1e400", "1e-400",
           # Exponents no integer holds: they saturate, with no overflow.
           "1e99999999999999999999", "-1e-99999999999999999999",
           "0e99999999999999999999", "-0.000e-99999999999999999999",
           "123.45E+12345678901234567890123"]
    # Halfway points between neighbours, written out exactly, and a hair
    # either side, with a long tail: the sticky flag's cases.
    for bits in (32, 64):
        mant, _ = FORMATS[bits]
        for _ in range(60):
            k = rnd.randrange(1 << mant, 1 << (mant + 1))
            e = rnd.randrange(-140 if bits == 32 else -1070, 100)
            half = Fraction(2 * k + 1) * Fraction(2) ** (e - 1)
            for v in (half, half + Fraction(1, 10 ** 900), half - Fraction(1, 10 ** 900)):
                out.append(decimal_text(v, 950))
    # Random decimals of many shapes.
    for _ in range(600):
        digits = "".join(rnd.choice("0123456789") for _ in range(rnd.randrange(1, 40)))
        digits = digits.lstrip("0") or "0"
        exponent = rnd.randrange(-60, 60)
        sign = rnd.choice(["", "-"])
        out.append(f"{sign}{digits}e{exponent}")
    # Mantissas of thousands of digits, far past the 800-digit buffer:
    # halfway points with a tail at 3,000 places, and one written long.
    for bits in (32, 64):
        mant, _ = FORMATS[bits]
        for _ in range(4):
            k = rnd.randrange(1 << mant, 1 << (mant + 1))
            e = rnd.randrange(-20, 20)
            half = Fraction(2 * k + 1) * Fraction(2) ** (e - 1)
            for v in (half + Fraction(1, 10 ** 3000), half - Fraction(1, 10 ** 3000)):
                out.append(decimal_text(v, 3100))
    out.append("1" + "0" * 3500 + "e-3500")
    for _ in range(200):
        x = struct.unpack("<f", struct.pack("<I", rnd.getrandbits(32) & 0x7F7FFFFF))[0]
        out.append(repr(x))
        out.append(f"{x:.9g}")
    return out


def decimal_text(v, places):
    """v exactly, if it ends within places digits; else truncated."""
    sign = "-" if v < 0 else ""
    v = abs(v)
    whole = v.numerator // v.denominator
    frac = v - whole
    s = []
    for _ in range(places):
        if frac == 0:
            break
        frac *= 10
        d = frac.numerator // frac.denominator
        s.append(str(d))
        frac -= d
    return sign + str(whole) + ("." + "".join(s) if s else "")


def render():
    lines = []
    rows = []
    for text in cases():
        # C guarantees string literals of 4,095 characters.
        if len(text) > 4000:
            continue
        v = exact(text)
        b32 = None if v == "overflow" else round_to(v, 32)
        b64 = None if v == "overflow" else round_to(v, 64)
        if text.startswith("-") and v != "overflow" and v == 0:  # the rational has no -0
            b32 |= 1 << 31
            b64 |= 1 << 63
        rows.append((text, b32, b64))
    lines += ["// SPDX-License-Identifier: MIT",
              "// Copyright (c) 2026 Sirac Ozmen",
              "//",
              "// Generated by tools/gen_number_cases.py: decimal texts and their",
              "// correctly rounded binary32 and binary64 bits from exact rationals;",
              "// an overflow flag where the format has no finite value. Do not edit.",
              "",
              "typedef struct NumberCase",
              "{",
              "    const char* text;",
              "    uint32_t bits32;",
              "    bool overflow32;",
              "    uint64_t bits64;",
              "    bool overflow64;",
              "} NumberCase;",
              "",
              "// clang-format off",
              "static const NumberCase kNumberCases[] = {"]
    for text, b32, b64 in rows:
        lines.append(f'    {{"{text}", 0x{(b32 or 0):08x}u, {"true" if b32 is None else "false"}, '
                     f'0x{(b64 or 0):016x}ull, {"true" if b64 is None else "false"}}},')
    lines += ["};", "// clang-format on"]
    return "\n".join(lines) + "\n"


def main():
    text = render()
    if len(sys.argv) > 1 and sys.argv[1] == "--check":
        path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "test", "json_number_cases.h")
        if open(path, encoding="utf-8").read() != text:
            print("test/json_number_cases.h differs from tools/gen_number_cases.py")
            sys.exit(1)
        return
    sys.stdout.write(text)


if __name__ == "__main__":
    main()
