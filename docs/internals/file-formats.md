# File formats

Every asset the runtime reads is a file in one container. Files cross machines and networks, so they are hostile input; they must mean the same on every platform; and the runtime reads them in place without allocating or converting. This page explains the container, the kinds, identities and versions.

## The container

[`src/container.h`](../../src/container.h) defines one layout for every kind: little-endian, a 48-byte header, a table of sections, then the sections, each at a multiple of 16 bytes from the start.

| Offset | Field |
|---|---|
| 0 | `MANI` |
| 4 | kind (u16), format version (u16) |
| 8 | header size, 48 (u32); section count (u32) |
| 16 | the writer's library version (three u16) and a zero u16 |
| 24 | file size (u64) |
| 32 | hash of every byte after the header (`maniHash64` from `MANI_HASH_INIT`) |
| 40 | the file's identity (u64) |

The table that follows holds a tag (u32), a zero u32, an offset and a size (u64 each) per section, tags strictly increasing from 1. Each section is a field: it starts where the previous one ends (the first where the table ends), rounded up to 16, the bytes in between are zero, and the file ends where its last section ends. A reader refuses any other placement, a nonzero pad byte or a byte past the last section with `mani_errorFormat`, so a file has one layout and one source writes the same bytes everywhere. A kind defines its tags, from 1 to its last, and what each holds; an empty section is left out of the table.

No layout keeps reserved bytes. The only zeros inside a header or a record align the field after them, where no order of the fields avoids it: the container's u16 at 22 and each table entry's u32 after the tag align the u64 fields that follow. Each kind's layout comment in `src/` names any such zero of its own.

No pointer is stored anywhere: every reference is an index or an offset, so a file means the same at any address and may be mapped. A big-endian target is refused at compile time ([`src/bytes.h`](../../src/bytes.h)) rather than misreading files.

## Opening checks everything once

`maniOpen...` takes the bytes at a 16-byte aligned address and checks, in order: the header, the table, the hash, then every section of the kind: counts, indices, ranges, floats finite, rotations of unit length, names valid UTF-8 and matching their hashes, derived sections equal to what they are derived from, and the identity. Only then does it fill the view. A refusal is `mani_errorFormat` with the offset of the first byte found wrong, or `mani_errorVersion` for any format version but the one the library reads (at offset 6) or a tag past the kind's last (at its table entry, once the hash shows the file undamaged).

Because opening has checked every byte, later calls do not check the file's contents again. The view itself is state the caller holds, though, which the family's conventions check on every call. A skeleton view records the file's start and size, its sections as offsets from the start, and a cookie (`MANI_VIEW_COOKIE` in `src/view.h`: the top bit set, distinct per kind). Every call that takes a skeleton first checks, in constant time, the cookie and that each section, at its count, lies within the size recorded. The start and size are the bytes the open was given, as a pointer and a length are for any argument: a call cannot tell that they no longer match, and a view whose start or size the caller changed is outside the contract. Everything else the view holds is checked against them, so a count or a section changed, or taken from another view, reads only within those bytes, or is refused. Each index a call reads from the bytes (a parent, a subtree's end, a name's span, the name order) is bounded where it is used, so bytes changed after the open are read only within the extents recorded. A clip view records the same, each section's size besides its offset, and every call that takes a clip checks it the same way. A clip's bytes are not checked again when it is sampled, so bytes changed after the open sample to some pose; what a call reads from them is bounded where it is read, so it stays within the file. Every loop runs as many times as the view's counts say, never as the bytes say. A track's bone is held in the skeleton and its channels to the floats a sample holds; a compressed clip's rate to those the codec has, a segment's data offset and every bit it reads to the segment data, its bitset walks to their tracks, and its float constants and animated floats to those the view counts. A name's span and a discrete track's keys read as none when they fall outside their sections. Skin, influences and morph target views record the same and are checked the same way. A skin's joint is held in the skeleton: the getter gives none and the palette reads the root. A vertex's joint is held in the skin's joints, reading joint 0. A channel's targets outside the targets give none and the channel no weights; a target whose deltas or indices fall outside their sections, or a dense one not covering the vertices, adds nothing; a sparse target's vertex outside the job's range is held at its last vertex, so the job writes only the range it was given; a name whose span leaves the names is none. The other views do not yet check themselves this way. The hash catches damage, not tampering; the checks are what make a crafted file safe. A file that passes every check but was not made by a builder can give a wrong pose, never an unsafe access. Each opener is tested with files changed at every byte a check reads, and fuzzed.

Opening needs no scratch memory: a skeleton's depth-first order, for instance, is checked in one pass by requiring each bone's parent to be the previous bone or one of its ancestors and closing each subtree as the walk leaves it.

## Kinds

| Kind | Number | Format version | Bound to |
|---|---|---|---|
| Skeleton | 1 | 2 | |
| Clip | 2 | 5 | its skeleton |
| Graph | 3 | 7 | its skeleton, its clips, databases, models, spring sets and maps |
| Retargeting map | 4 | 4 | its source and target skeletons |
| Skin | 5 | 2 | its skeleton |
| Influences | 6 | 2 | its skin |
| Morph targets | 7 | 2 | |
| Motion database | 8 | 5 | its skeleton and clips, and a mirror map for mirrored entries |
| Learned model | 9 | 3 | its skeleton and database |
| Learned checkpoint | 10 | 2 | its trainer's source and options |
| Spring set | 11 | 3 | its skeleton |
| Ragdoll profile | 12 | 2 | its skeleton |
| Mirror map | 13 | 3 | its skeleton |

A refusal names the tag of the section holding the first byte found wrong (`maniFileResult.tag`, 0 in the header, the table or the padding between sections), so a host can tell which part of a file is damaged without knowing its layout:

| Kind | Section tags |
|---|---|
| Skeleton | 1 parents, 2 subtree ends, 3 rests, 4 name hashes, 5 name order, 6 name spans, 7 name bytes |
| Clip | 1 header, 2 tracks, 3 samples, 4 float tracks, 5 float samples, 6 discrete tracks, 7 discrete keys, 8 events, 9 markers, 10 names, 11 forms, 12 constants, 13 ranges, 14 segments, 15 segment data, 16 root motion, 17 loop policy, 18 additive basis |
| Graph | 1 header, 2 parameters, 3 nodes, 4 inputs, 5 data, 6 clips, 7 sync groups, 8 mask weights, 9 databases, 10 database clips, 11 names, 12 models, 13 spring sets, 14 maps, 15 mirror |
| Retargeting map | 1 header, 2 source, 3 target, 4 plan, 5 pelvis |
| Skin | 1 binding, 2 joints, 3 inverse binds |
| Influences | 1 binding, 2 joints, 3 weights, 4 dual quaternion weights, 5 centers |
| Morph targets | 1 header, 2 attributes, 3 channels, 4 targets, 5 indices, 6 deltas, 7 names |
| Motion database | 1 header, 2 channels, 3 times, 4 clips, 5 entries, 6 normalization, 7 features, 8 boxes, 9 names, 10 mirror, 11 nodes, 12 positions, 13 places, 14 leaf size |
| Learned model | 1 header, 2 channels, 3 times, 4 normalization, 5 bones, 6 networks, 7 layers, 8 values, 9 weights, 10 names, 11 name bytes, 12 weight format |
| Spring set | 1 header, 2 colliders, 3 springs, 4 joints, 5 links, 6 names |
| Ragdoll profile | 1 header, 2 bodies, 3 names |
| Mirror map | 1 header, 2 bones, 3 names, 4 strings |
| Learned checkpoint | 1 progress, 2 latents, 3 and on the networks' training state, 16 and on the networks |

Every kind's header section (tag 1) is required, and so is every section of a skeleton and of a skin: a file without one is refused with `mani_errorFormat`. Any other section that is absent is empty, which is its default: no records of that table. An opener refuses an empty section that the header's counts need with `mani_errorFormat`, as it refuses a section of the wrong size, so a writer leaves a section out exactly when it holds no records. The sections that mean something when absent are a clip's root motion (16, a clip without root motion), loop policy (17, `maniLoopClamp`) and additive basis (18, `maniAdditiveNone`), each present only off that default, a graph's mirror (15, no mirror map), a retargeting map's pelvis (5, no pelvis: the up axis +y, the scales and the height ratio 1, the blends, weight and positions 0), the influences' dual quaternion weights and centers (4 and 5, no dual quaternion skinning), a motion database's mirror (10, no mirrored entries) and its leaf size (14, `MANI_MATCH_DEFAULT_LEAF`, 16 frames, present only off it), and a learned model's weight format (12, binary32 weights, present only for 16-bit ones).

The learned checkpoint is a state, not an asset ([overview.md](overview.md)): only the release that wrote it reads it, so its reader takes no tag a later writer adds and refuses any section count but its own with `mani_errorFormat`.

A compressed clip is a clip file in which the samples section is replaced by the compressed sections ([clips-and-compression.md](clips-and-compression.md)); `maniOpenClip` opens either form, and tells them apart by the header's segment count (0 at full precision, at least 1 compressed).

## Identities

A file's identity is a 64-bit hash. A skeleton's covers its bone count, parents, rests and names: everything that changes what a clip means for it. Every file built for a skeleton records that identity (and usually the bone count), and every call that pairs them compares it, so a clip of another skeleton, a skin of another rig or a map built for other skeletons is refused with `mani_errorInvalid`, not played wrongly. A graph records the identities of the clips, databases, models, spring sets and maps it was built against, and binding matches the host's assets by them. Two skeletons with the same identity are the same skeleton for every purpose; a reimport that changes nothing keeps every dependent file valid.

The other kinds' identities cover every section, so the identity is a fingerprint of the whole file.

## Versions

A kind's format version changes whenever its bytes change. Before 1.0.0 a reader reads its own format version alone and refuses every other one, earlier or later, with `mani_errorVersion` at offset 6; files are rebuilt from their sources, which is why every asset has a source form (structs, JSON for graphs and maps, glTF or FBX through the importers). From 1.0.0 a kind's format version no longer changes: a later release adds a field as a section with a new tag, which an earlier reader refuses with `mani_errorVersion`, and a writer leaves out a field that holds its default, so a file that uses no later field opens in an earlier release too.

The library version a file was written by (bytes 16 to 23) is informational: readers decide by the format version alone.

## Traps

- Before 1.0.0, a new section that a reader of the same format version would not know changes the format version.
- A new value per record arrives as a new section with its own tag, parallel to the records, not as a longer record: record strides stay fixed.
- Sections derived from others (a skeleton's name order, a database's search tree) are checked against what they are derived from at open. A motion database's search is exact only because each node's box is the exact minimum and maximum of its frames' features, so opening checks that the tree's order holds every frame once with its entry and recomputes every box, a leaf's from its frames and any other's from its two halves.
- Hash functions here are for identities and damage, never for security.
