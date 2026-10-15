# mani-0012. The FBX document

Status: Accepted

## Context

FBX has no published specification and its SDK is closed. The binary
form's encoding is described in the Blender Foundation's public-domain
notes (2013), and ufbx, an open reader, shows what files in the wild
hold: versions 3000 to 7700, record headers widened to 64 bits from
7500, big-endian files, arrays raw or zlib-compressed. The ASCII form
holds the same tree as text. Importing animation needs the tree's
objects, properties and connections, not its meshes, and a file is
hostile input.

## Decision

- **One tree from either form:** a flat array of nodes (name, a run of
  properties, first child, next sibling) and one of properties (type
  letter, count, where the payload lies in the file, an array's
  encoding). Node 0 has the top-level nodes as its children.
- **The binary container:** the 27-byte header (magic, 0x1A, an
  endianness byte of 0 or 1, the version); versions 3000 to 7700, others
  refused (Version); headers of 13 bytes before 7500 and 25 from it.
  Every record's end offset must lie within its parent's and match
  where its properties and children end; property bytes must add up to
  the stated length; each child list ends in a record of no end and no
  name (all zeros as written), the top level too; at most 64 levels. A
  record whose end offset is 0 ends with its values and holds no
  children, as ufbx reads the records Maya 2019 writes so. Read in two passes, iteratively:
  the first counts and checks, the second fills. Refusals are
  Container, at the byte found wrong.
- **Values on request:** numbers, whole numbers within a range, raw
  strings (FBX's names hold `\x00\x01` separators), and arrays decoded
  into caller memory, as binary64 or as integers. A zlib array is
  inflated by the family's inflate into exactly its decoded size; a
  stream that gives more or less or fails its checksum is Data at the
  array, a float that is not finite Data at its element (or the array,
  when compressed).
- **The ASCII form,** into the same tree: `Name: value, value
  { children }` (a leading comma allowed), names bare or quoted, `;`
  comments, quoted strings with FBX's `&quot;`, `&cr;` and `&lf;`
  decoded when read, bare words, numbers kept as their text and
  converted exactly when read (mani-0007's decimal conversion; a
  leading `+` allowed; old writers' `1.#INF` kept as a number that is
  none). `*N { a: v, ... }` folds into one array property of exactly N
  numbers, read like a binary array. The version is the first
  comment's (`; FBX 7.4.0 project file`), else
  `FBXHeaderExtension.FBXVersion`, else refused (Version). Refusals are
  Syntax at the token's first byte; nesting is bounded at 64 as in the
  binary form.
- **`maniOpenFbx`, `maniCloseFbx`, `maniFbxVersion`**; anything without
  the binary magic is read as text.

## Consequences

Opening is linear in the file and allocates two arrays; arrays the
import never needs cost nothing. The animation import reads the same
tree whichever form a file came in.
