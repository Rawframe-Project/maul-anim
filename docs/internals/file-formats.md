# File formats

Every asset the runtime reads is a file in one container. Files cross machines and networks, so they are hostile input; they must mean the same on every platform; and the runtime reads them in place without allocating or converting. This page explains the container, the kinds, identities and versions.

## The container

[`src/container.h`](../../src/container.h) defines one layout for every kind: little-endian, a 64-byte header, a table of sections, then the sections, each at a multiple of 16 bytes from the start.

| Offset | Field |
|---|---|
| 0 | `MANI` |
| 4 | kind (u16), format version (u16) |
| 8 | header size, 64 (u32); section count (u32) |
| 16 | the writer's library version (three u16) and a zero u16 |
| 24 | file size (u64) |
| 32 | hash of every byte after the header (`maniHash64` from `MANI_HASH_INIT`) |
| 40 | the file's identity (u64) |
| 48 | zero (16 bytes) |

The table that follows holds a tag, an offset and a size per section: tags strictly increasing, sections in table order, inside the file, without overlap. A kind defines its tags and what each holds; an empty section is left out of the table, and absent means empty. Writers zero every pad byte, so one source writes the same bytes everywhere.

No pointer is stored anywhere: every reference is an index or an offset, so a file means the same at any address and may be mapped. A big-endian target is refused at compile time ([`src/bytes.h`](../../src/bytes.h)) rather than misreading files.

## Opening checks everything once

`maniOpen...` takes the bytes at a 16-byte aligned address and checks, in order: the header, the table, the hash, then every section of the kind: counts, indices, ranges, floats finite, rotations of unit length, names valid UTF-8 and matching their hashes, derived sections equal to what they are derived from, and the identity. Only then does it fill the view. A refusal is `mani_errorFormat` with the offset of the first byte found wrong, or `mani_errorVersion` for a format version newer than the library reads.

Because opening has checked every byte, no later call checks again: a getter reads without branches, a sampler indexes without bounds checks. The hash catches damage, not tampering; the checks are what make a crafted file safe. A file that passes every check but was not made by a builder can give a wrong pose, never an unsafe access. Each opener is tested with files changed at every byte a check reads, and fuzzed.

Opening needs no scratch memory: a skeleton's depth-first order, for instance, is checked in one pass by requiring each bone's parent to be the previous bone or one of its ancestors and closing each subtree as the walk leaves it.

## Kinds

| Kind | Number | Format version | Bound to |
|---|---|---|---|
| Skeleton | 1 | 1 | |
| Clip | 2 | 3 | its skeleton |
| Graph | 3 | 5 | its skeleton, its clips, databases, models, spring sets and maps |
| Retargeting map | 4 | 2 | its source and target skeletons |
| Skin | 5 | 1 | its skeleton |
| Influences | 6 | 1 | its skin |
| Morph targets | 7 | 1 | |
| Motion database | 8 | 3 | its skeleton and clips, and a mirror map for mirrored entries |
| Learned model | 9 | 1 | its skeleton and database |
| Learned checkpoint | 10 | 1 | its trainer's source and options |
| Spring set | 11 | 2 | its skeleton |
| Ragdoll profile | 12 | 1 | its skeleton |
| Mirror map | 13 | 2 | its skeleton |

A compressed clip is a clip file in which the samples section is replaced by the compressed sections ([clips-and-compression.md](clips-and-compression.md)); `maniOpenClip` opens either form.

## Identities

A file's identity is a 64-bit hash. A skeleton's covers its bone count, parents, rests and names: everything that changes what a clip means for it. Every file built for a skeleton records that identity (and usually the bone count), and every call that pairs them compares it, so a clip of another skeleton, a skin of another rig or a map built for other skeletons is refused with `mani_errorInvalid`, not played wrongly. A graph records the identities of the clips, databases, models, spring sets and maps it was built against, and binding matches the host's assets by them. Two skeletons with the same identity are the same skeleton for every purpose; a reimport that changes nothing keeps every dependent file valid.

The other kinds' identities cover every section, so the identity is a fingerprint of the whole file.

## Versions

A kind's format version changes whenever its bytes change. A reader refuses a version newer than it knows with `mani_errorVersion`. Before 1.0.0 a release may drop older versions; files are rebuilt from their sources, which is why every asset has a source form (structs, JSON for graphs and maps, glTF or FBX through the importers). From 1.0.0 a release reads every earlier format version of its major version, and the files each release writes are kept as test data.

The library version a file was written by (bytes 16 to 23) is informational: readers decide by the format version alone.

## Traps

- A new section that a reader of the same format version would not know changes the format version.
- Sections derived from others (a skeleton's name order, a database's search tree) are checked against what they are derived from at open. A motion database's search is exact only because each node's box is the exact minimum and maximum of its frames' features, so opening checks that the tree's order holds every frame once with its entry and recomputes every box, a leaf's from its frames and any other's from its two halves.
- Hash functions here are for identities and damage, never for security.
