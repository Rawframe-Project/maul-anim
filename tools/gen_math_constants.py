#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Writes src/generated/math_constants.h and math_constants_float.h, the
# constants of the library's own math functions in binary64
# (src/elementary.c) and binary32 (src/elementary_float.c), a family
# file: the splits of ln 2 and pi / 2 for exact argument reduction,
# the Taylor coefficients of the polynomials, arctangents at multiples of
# 1/8, and where the exponentials overflow and underflow. Each is
# computed in decimal arithmetic to 80 digits and rounded once.
#
# usage: gen_math_constants.py [--check]
#   --check  fail when the file differs from what would be written

import decimal
import os
import sys
from fractions import Fraction

D = decimal.Decimal
decimal.getcontext().prec = 80
GENERATED = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "generated")


def pi():
    # Machin's formula: pi = 16 atan(1/5) - 4 atan(1/239).
    def atan_inv(n):
        x = D(1) / n
        x2 = x * x
        term, total, k = x, x, 1
        while True:
            term *= -x2
            add = term / (2 * k + 1)
            if add == 0:
                return total
            total += add
            k += 1
    return 16 * atan_inv(5) - 4 * atan_inv(239)


def atan(x):
    # Halve the argument until it is small, then sum the series.
    halvings = 0
    while abs(x) > D("0.01"):
        x = x / (1 + (1 + x * x).sqrt())
        halvings += 1
    x2 = x * x
    term, total, k = x, x, 1
    while True:
        term *= -x2
        add = term / (2 * k + 1)
        if add == 0:
            break
        total += add
        k += 1
    return total * (2 ** halvings)


def two_over_pi_words(count):
    """The first 64 * count bits of 2 / pi after the binary point, as
    64-bit words, most significant first: pi from Machin's formula in
    integers with 256 guard bits, so every bit is exact."""
    bits = 64 * count
    scale = bits + 256

    def atan_inv(n):
        x = (1 << scale) // n
        total, k, n2 = x, 1, n * n
        while x:
            x //= n2
            total += -(x // (2 * k + 1)) if k % 2 else x // (2 * k + 1)
            k += 1
        return total

    pi_scaled = 16 * atan_inv(5) - 4 * atan_inv(239)
    value = (1 << (bits + 1 + scale)) // pi_scaled
    return [(value >> (64 * (count - 1 - i))) & ((1 << 64) - 1) for i in range(count)]


def word_table(out, name, count, note):
    out.append(f"// {note}")
    out.append(f"static const uint64_t {name}[{count}] = {{")
    for w in two_over_pi_words(count):
        out.append(f"    0x{w:016X}u,")
    out.append("};")


def to_double(value):
    # The nearest binary64, ties to even: Python's float of a Fraction
    # is correctly rounded.
    return float(Fraction(value))


def to_bits(value, bits):
    # value rounded to `bits` significant bits, so that products with
    # small integers are exact.
    f = Fraction(value)
    if f == 0:
        return 0.0
    e = 0
    while abs(f) >= 2:
        f /= 2
        e += 1
    while abs(f) < 1:
        f *= 2
        e -= 1
    scaled = f * (2 ** (bits - 1))
    n = round(scaled)
    return float(Fraction(n, 2 ** (bits - 1)) * Fraction(2) ** e)


def split(value, bits):
    hi = to_bits(value, bits)
    return hi, D(value) - D(Fraction(hi).numerator) / D(Fraction(hi).denominator)


def exact(x):
    f = Fraction(x)
    return D(f.numerator) / D(f.denominator)


def factorial(n):
    out = 1
    for i in range(2, n + 1):
        out *= i
    return out


def lines():
    ln2 = D(2).ln()
    pio2 = pi() / 2
    out = []

    def const(name, value, note):
        out.append(f"// {note}")
        out.append(f"#define {name} {value.hex()}")

    ln2hi, rest = split(ln2, 40)
    const("MANI_LN2_HI", ln2hi, "ln 2 to 40 bits: k times it is exact for |k| < 2^13.")
    const("MANI_LN2_LO", to_double(rest), "ln 2 minus MANI_LN2_HI.")
    const("MANI_INV_LN2", to_double(1 / ln2), "1 / ln 2.")
    ln2_near = to_double(ln2)
    const("MANI_LN2", ln2_near, "ln 2, the nearest double.")
    const("MANI_LN2_TAIL", to_double(ln2 - exact(ln2_near)), "ln 2 minus MANI_LN2.")
    p1, rest = split(pio2, 33)
    p2, rest = split(rest, 33)
    const("MANI_PIO2_1", p1, "pi / 2 to 33 bits: k times it is exact for |k| < 2^20.")
    const("MANI_PIO2_2", p2, "The next 33 bits of pi / 2.")
    const("MANI_PIO2_3", to_double(rest), "The rest of pi / 2.")
    const("MANI_INV_PIO2", to_double(1 / pio2), "2 / pi.")
    pi_hi = to_double(2 * pio2)
    const("MANI_PI_HI", pi_hi, "pi.")
    const("MANI_PI_LO", to_double(2 * pio2 - exact(pi_hi)), "pi minus MANI_PI_HI.")
    pio2_hi = to_double(pio2)
    const("MANI_PIO2_HI", pio2_hi, "pi / 2.")
    const("MANI_PIO2_LO", to_double(pio2 - exact(pio2_hi)), "pi / 2 minus MANI_PIO2_HI.")

    def table(name, values, note):
        out.append(f"// {note}")
        out.append(f"static const double {name}[{len(values)}] = {{")
        for v in values:
            out.append(f"    {v.hex()},")
        out.append("};")

    table("s_expTaylor", [to_double(D(1) / factorial(n)) for n in range(2, 14)],
          "1 / n! for n from 2 to 13: e^r for |r| <= ln 2 / 2.")
    table("s_exp2Taylor", [to_double(ln2 ** n / factorial(n)) for n in range(1, 14)],
          "(ln 2)^n / n! for n from 1 to 13: 2^f for |f| <= 1/2.")
    table("s_logSeries", [to_double(D(2) / (2 * n + 1)) for n in range(1, 12)],
          "2 / (2n + 1) for n from 1 to 11: the atanh series of the logarithm.")
    table("s_sinTaylor", [to_double(D((-1) ** n) / factorial(2 * n + 1)) for n in range(1, 9)],
          "(-1)^n / (2n + 1)! for n from 1 to 8: sine for |r| <= pi / 4.")
    table("s_cosTaylor", [to_double(D((-1) ** n) / factorial(2 * n)) for n in range(2, 10)],
          "(-1)^n / (2n)! for n from 2 to 9: cosine for |r| <= pi / 4.")
    table("s_atanSeries", [to_double(D((-1) ** n) / (2 * n + 1)) for n in range(1, 7)],
          "(-1)^n / (2n + 1) for n from 1 to 6: arctangent for |u| <= 1/16.")
    eighths = [atan(D(k) / 8) for k in range(0, 9)]
    highs = [to_double(v) for v in eighths]
    table("s_atanEighths", highs, "atan(k / 8) for k from 0 to 8.")
    table("s_atanEighthsLow", [to_double(v - exact(h)) for v, h in zip(eighths, highs)],
          "atan(k / 8) minus s_atanEighths[k].")
    word_table(out, "s_twoOverPi", 19,
               "2 / pi's bits after the point, most significant first: a window of "
               "192 from any bit a double's reduction starts at.")
    return out


def to_float(value):
    # The nearest binary32, ties to even, as a double holding it exactly.
    return to_bits(value, 24)


def float_step(value, direction):
    # The binary32 next to a normal binary32 value, up or down.
    f = Fraction(value)
    e = 0
    a = abs(f)
    while a >= 2:
        a /= 2
        e += 1
    while a < 1:
        a *= 2
        e -= 1
    ulp = Fraction(2) ** (e - 23)
    if direction < 0 and a == 1 and f > 0:
        ulp /= 2
    return float(f + direction * ulp)


def float_hex(value):
    # The hexadecimal literal of a binary32 held in a double, without the
    # trailing zeros a double's 13 digits leave.
    text = value.hex()
    if "." in text:
        mantissa, exponent = text.split("p")
        mantissa = mantissa.rstrip("0").rstrip(".")
        text = mantissa + "p" + exponent
    return text + "f"


def lines_float():
    ln2 = D(2).ln()
    pio2 = pi() / 2
    out = []

    def const(name, value, note):
        out.append(f"// {note}")
        out.append(f"#define {name} {float_hex(value)}")

    ln2hi = to_bits(ln2, 16)
    const("MANI_LN2_HI_F", ln2hi, "ln 2 to 16 bits: k times it is exact for |k| < 2^8.")
    const("MANI_LN2_LO_F", to_float(ln2 - exact(ln2hi)), "ln 2 minus MANI_LN2_HI_F.")
    const("MANI_INV_LN2_F", to_float(1 / ln2), "1 / ln 2.")
    ln2_near = to_float(ln2)
    const("MANI_LN2_F", ln2_near, "ln 2, the nearest float.")
    const("MANI_LN2_TAIL_F", to_float(ln2 - exact(ln2_near)), "ln 2 minus MANI_LN2_F.")
    # Six 8-bit parts, each times k exact for |k| < 2^16, and a 24-bit
    # tail: pi / 2 to about 2^-73, as reducing floats up to 1e5 that lie
    # within 2^-27.8 of a multiple of pi / 2 needs.
    rest = pio2
    parts = []
    for i in range(6):
        part = to_bits(rest, 8)
        parts.append(part)
        rest = rest - exact(part)
    parts.append(to_float(rest))
    out.append("// pi / 2 in six 8-bit parts and a 24-bit tail: k times each of the")
    out.append("// first six is exact for |k| < 2^16.")
    out.append(f"static const float s_pio2PartsF[{len(parts)}] = {{")
    for v in parts:
        out.append(f"    {float_hex(v)},")
    out.append("};")
    const("MANI_INV_PIO2_F", to_float(1 / pio2), "2 / pi.")
    pi_hi = to_float(2 * pio2)
    const("MANI_PI_HI_F", pi_hi, "pi.")
    const("MANI_PI_LO_F", to_float(2 * pio2 - exact(pi_hi)), "pi minus MANI_PI_HI_F.")
    pio2_hi = to_float(pio2)
    const("MANI_PIO2_HI_F", pio2_hi, "pi / 2.")
    const("MANI_PIO2_LO_F", to_float(pio2 - exact(pio2_hi)), "pi / 2 minus MANI_PIO2_HI_F.")
    # The largest float whose exponential is finite, and the smallest
    # whose exponential rounds above 0 (above 2^-150, half the smallest
    # subnormal).
    flt_max = (2 - Fraction(1, 2 ** 23)) * Fraction(2) ** 127
    top = D(flt_max.numerator).ln() - D(flt_max.denominator).ln()
    over = to_float(top)
    while exact(over) > top:
        over = float_step(over, -1)
    while exact(float_step(over, 1)) <= top:
        over = float_step(over, 1)
    const("MANI_EXP_OVERFLOW_F", over, "The largest x whose e^x is a finite float.")
    bottom = -150 * ln2
    under = to_float(bottom)
    while exact(under) <= bottom:
        under = float_step(under, 1)
    while exact(float_step(under, -1)) > bottom:
        under = float_step(under, -1)
    const("MANI_EXP_UNDERFLOW_F", under, "The smallest x whose e^x rounds to a float above 0.")

    def table(name, values, note):
        out.append(f"// {note}")
        out.append(f"static const float {name}[{len(values)}] = {{")
        for v in values:
            out.append(f"    {float_hex(v)},")
        out.append("};")

    table("s_expTaylorF", [to_float(D(1) / factorial(n)) for n in range(2, 8)],
          "1 / n! for n from 2 to 7: e^r for |r| <= ln 2 / 2.")
    table("s_exp2TaylorF", [to_float(ln2 ** n / factorial(n)) for n in range(1, 8)],
          "(ln 2)^n / n! for n from 1 to 7: 2^f for |f| <= 1/2.")
    table("s_logSeriesF", [to_float(D(2) / (2 * n + 1)) for n in range(1, 6)],
          "2 / (2n + 1) for n from 1 to 5: the atanh series of the logarithm.")
    two_thirds = to_float(D(2) / 3)
    const("MANI_TWO_THIRDS_LO_F", to_float(D(2) / 3 - exact(two_thirds)),
          "2/3 minus s_logSeriesF[0], for the series' leading term as a pair.")
    table("s_sinTaylorF", [to_float(D((-1) ** n) / factorial(2 * n + 1)) for n in range(1, 5)],
          "(-1)^n / (2n + 1)! for n from 1 to 4: sine for |r| <= pi / 4.")
    table("s_cosTaylorF", [to_float(D((-1) ** n) / factorial(2 * n)) for n in range(2, 6)],
          "(-1)^n / (2n)! for n from 2 to 5: cosine for |r| <= pi / 4.")
    table("s_atanSeriesF", [to_float(D((-1) ** n) / (2 * n + 1)) for n in range(1, 4)],
          "(-1)^n / (2n + 1) for n from 1 to 3: arctangent for |u| <= 1/16.")
    eighths = [atan(D(k) / 8) for k in range(0, 9)]
    highs = [to_float(v) for v in eighths]
    table("s_atanEighthsF", highs, "atan(k / 8) for k from 0 to 8.")
    table("s_atanEighthsLowF", [to_float(v - exact(h)) for v, h in zip(eighths, highs)],
          "atan(k / 8) minus s_atanEighthsF[k].")
    word_table(out, "s_twoOverPiF", 4,
               "2 / pi's bits after the point, most significant first: a window of "
               "128 from any bit a float's reduction starts at.")
    out.append("// pi / 2, the nearest double: the far reduction's last product.")
    out.append(f"#define MANI_PIO2_BINARY64_F {to_double(pio2).hex()}")
    return out


def render(body, guard, kind, source):
    head = [
        "// SPDX-License-Identifier: MIT",
        "// Copyright (c) 2026 Sirac Ozmen",
        "//",
        "// Generated by tools/gen_math_constants.py; do not edit. The",
        f"// constants of {source} (a family file), each",
        f"// computed to 80 digits and rounded once to {kind}.",
        "// clang-format off",
        "",
        f"#ifndef {guard}",
        f"#define {guard}",
        "",
        "#include <stdint.h>",
        "",
    ]
    return "\n".join(head + body + ["", "#endif", ""])


def outputs():
    return [
        ("math_constants.h", render(lines(), "MANI_MATH_CONSTANTS_H", "binary64", "src/elementary.c")),
        ("math_constants_float.h", render(lines_float(), "MANI_MATH_CONSTANTS_FLOAT_H", "binary32",
                                          "src/elementary_float.c")),
    ]


def main():
    check = len(sys.argv) > 1 and sys.argv[1] == "--check"
    stale = 0
    for name, text in outputs():
        path = os.path.join(GENERATED, name)
        if check:
            current = ""
            if os.path.exists(path):
                with open(path, encoding="utf-8") as f:
                    current = f.read()
            if current != text:
                print(f"src/generated/{name} is stale: run tools/gen_math_constants.py")
                stale = 1
            continue
        os.makedirs(GENERATED, exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        print("wrote " + os.path.normpath(path))
    if check and not stale:
        print("math constants: current")
    return stale


if __name__ == "__main__":
    sys.exit(main())
