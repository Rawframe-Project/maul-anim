# mani-0007. The glTF importer's JSON reader and numbers

Status: Accepted

## Context

glTF arrives as JSON, alone or in a GLB container, from any exporter,
and is hostile input. Its numbers (node transforms, indices, counts)
must become the same binary32 bits on every platform. `strtod` and
`strtof` depend on the C locale's decimal point, rounded some inputs
wrongly in older C libraries, and are C library maths the family keeps
out of results; a double rounded again to float differs from the
correctly rounded float in rare halfway cases.

## Decision

- **The reader** follows RFC 8259 strictly: no comments, no trailing
  commas, no leading zeros, control characters escaped, UTF-8
  throughout, `\u` surrogates paired. A leading UTF-8 byte order mark
  is skipped (RFC 8259 section 8.1, to which glTF points). It runs
  twice over the text, first checking and counting, then filling a
  flat array of values in document order allocated once; it is
  iterative and nests at most 128 deep. Strings and numbers stay
  spans of the text, decoded when read. A refusal names the reason and
  the first byte found wrong.
- **Repeated names:** a lookup takes an object's last member of the
  name, as glTF asks ("glTF client implementations SHOULD override
  lexically preceding values for the same key").
- **Numbers** convert when read, to what the property wants: a whole
  number in a range (a fraction refused, exponents and zero fractions
  allowed as glTF allows them), binary32 or binary64, with integer
  arithmetic only. The conversion is Eisel and Lemire's method first
  (Lemire 2021): the first 19 significant digits as an integer, times
  the power of five truncated to 128 bits from a generated table
  (`tools/gen_decimal_powers.py`, checked current by the gate), the
  high word enough by Mushtak and Lemire's proof; past 19 digits the
  result stands only when the next integer up rounds the same. What it
  leaves open goes to the multiprecision decimal method: the digits
  kept in decimal (800 of them, and a sticky flag for any nonzero digit
  dropped past them, which keeps the result exact for any length),
  halved or doubled by shifts until the value lies in [1/2, 1), then
  rounded half to even at the target format's width, directly, so with
  one rounding. A value past the format's largest is refused, never
  infinite.

## Consequences

Numbers read the same everywhere, under any floating-point environment,
with no C library. The multiprecision method alone spent about 2,000
instructions on each of the 17-digit numbers exporters write, two
thirds of a large rig's skeleton read; the first method takes about a
sixth of that, for a table of 10,416 bytes of import code, and the two
agree on every one of 1,860,000 decimals of 1 to 40 digits and on the
exact midpoints between adjacent values of both formats. The tests check
the cases of an exact rational oracle (`tools/gen_number_cases.py`),
which agrees with Python's binary64 and glibc's binary32 conversions on
every case, and ties in the first 19 digits that a later digit breaks,
which only the second method rounds right.
