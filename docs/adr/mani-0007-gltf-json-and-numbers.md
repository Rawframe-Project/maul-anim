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
  allowed as glTF allows them), binary32 or binary64. The conversion is
  the multiprecision decimal method, with integer arithmetic only: the
  digits kept in decimal (800 of them, and a sticky flag for any
  nonzero digit dropped past them, which keeps the result exact for any
  length), halved or doubled by shifts until the value lies in
  [1/2, 1), then rounded half to even at the target format's width,
  directly, so with one rounding. A value past the format's largest is
  refused, never infinite.

## Consequences

Numbers read the same everywhere, under any floating-point environment,
with no table and no C library. The method is slow next to Eisel and
Lemire's, which does not matter for the few thousand numbers a glTF's
JSON holds; its bulk data is binary. The tests check 1,399 cases
against an exact rational oracle (`tools/gen_number_cases.py`), which
agrees with Python's binary64 and glibc's binary32 conversions on every
case.
