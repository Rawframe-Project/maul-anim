# mani-0002. The file container

Status: Accepted

## Context

Skeletons, clips, graphs and retargeting maps are files whose bytes the
host holds and the runtime reads in place (mani-0001). They cross
machines and networks, so they are hostile input, must mean the same
on every platform, and must stay readable as their formats grow.
Pointers inside them would tie a file to an address, and conversion on
load would need memory the runtime does not allocate.

## Decision

- **One container for every kind of file:** little-endian, a 64-byte
  header, a table of sections, then the sections, each at an offset
  from the file's start that is a multiple of 16. The bytes are used in
  place from a 16-byte aligned address and may be mapped.
- **The header:** the magic `MANI`; the kind (skeleton, clip, graph,
  retargeting map); the kind's format version; the header size; the
  section count; the version of the library that wrote the file; the
  file size; a 64-bit hash of every byte after the header (`maniHash64`
  from `MANI_HASH_INIT`); the file's identity; zero bytes reserved.
- **The table:** per section a tag, an offset and a size; tags strictly
  increasing, sections in table order without overlap and inside the
  file. A kind defines its tags and what each section holds.
- **Opening checks everything once:** the header, the table, the hash,
  then the kind's own sections (every count, index and value), so no
  later call checks again. A refusal is `mani_errorFormat` with the
  byte offset of the first field found wrong, or `mani_errorVersion`
  for a format version newer than the library reads. The hash catches
  damage, not tampering.
- **Identities:** a skeleton's identity is a hash of its structure,
  names and rest pose; the files built against a skeleton record its
  identity, and binding refuses a mismatch.
- **Versions:** a kind's format version changes with any change to its
  bytes. A release reads every earlier format version of its major
  version, and the files each release writes are kept as test data.
  Before 1.0 a format may change in any minor version.
- **Big-endian targets** are refused at build time.

## Consequences

A file opens in place with no allocation and no conversion, a corrupt
file names the byte that is wrong, files written on one platform play
the same on every other, and a kind adds sections without a new
container. Writers pad sections with zeros, so the same input writes
the same bytes everywhere.
