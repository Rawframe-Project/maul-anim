# mani-0019. Compressed clips

Status: Accepted

## Context

A full-precision clip (mani-0006) stores every channel at every sample
as binary32: the reference, and too large to ship. Animation
Compression Library (ACL) by Nicholas Frechette is the published state
of the art for uniformly sampled clips: constant and default tracks,
segments, range reduction over the clip and each segment, per-segment
bit rates chosen by an error metric over the skeleton's hierarchy. Its
measured size on a public clip set is the target to meet, and the
library's results must be the same bits on every platform, the
compressor's bytes included.

## Decision

- **One kind, two forms.** A compressed clip is a clip file whose
  samples sections are replaced by five more: forms, constants, clip
  ranges, a segment table and segment data. Identity, rate, duration,
  loop, float and discrete tracks, events, markers and names are as the
  full form stores them. `maniOpenClip` opens either; every query and
  sampler takes either. Empty sections are left out of a clip file's
  table (absent is empty; the identity does not change).
- **The method** is ACL's: each bone channel and float track is a
  sub-track, default (the rest, nothing stored), constant (one value) or
  animated. Samples fall into segments of about 16 (one segment up to
  31). Animated sub-tracks keep a clip range in binary32 and, with
  several segments, a segment range on 8 bits a component rounded
  outward, and per segment a bit rate: 1 to 23 bits a component, raw
  binary32, or 0 for a value constant in the segment, kept on 16 bits.
  Rotations store three components; the fourth, made positive, is
  rebuilt. Unlike ACL, which always drops w, a rotation drops the
  component whose smallest magnitude over the clip is largest (w on
  ties): rebuilding a component near 0 loses precision whatever the
  bits, and the choice costs two spare bits of a byte already stored.
- **The error metric:** points at a shell distance on each bone's axes
  carried through the hierarchy, in binary64; the error is the
  farthest one moves. Folding a channel to default or constant and the
  rate search both use it. The search visits bones leaves first, tries
  each bone's rate combinations in increasing size, and keeps the
  first size at which the bone and all its descendants stay within
  both their own shell and a dominant one grown over their
  descendants; of that size, the combination with the least room left
  wins. Every bone stays within its precision at every sample whenever
  binary32 can hold it; the report says when it cannot.
- **The same bytes everywhere:** the compressor measures through the
  sampler's own arithmetic (binary32 operations in a fixed order, no
  contraction, no estimates) and nothing depends on the machine.
- **Playback:** random access decodes the two samples a time falls
  between; a subset of bones decodes nothing else; a forward cursor in
  caller memory keeps the two decoded samples, bound to its clip by
  identity, address and shape, and gives random access's bits.
- **The ACL differential** (`tools/acl_differential.cc`, the one C++
  program, built with `MAUL_ANIM_DIFFERENTIAL` against ACL and RTM at
  pinned commits) hands the same imported samples to both and prints
  sizes, errors by one metric, compression times and playback costs; a
  weekly job runs it on ozz-animation 0.17.0's Quaternius clips.

## Consequences

On those clips (53 bones, 46 clips, 67.7 s, 0.1 mm at 3 cm) Maul Anim
writes 4,198 bytes a second against ACL's 3,920, its compressed
sections 0.978 of ACL's bytes, the rest the container and the track
table; its largest error is 0.1000 mm where ACL's is 0.31 mm, and it
compresses in 0.18 of ACL's time. Playback costs about twice ACL's
instructions, which uses SIMD four components at a time; the library
takes the family's eight-lane vectors before closing that gap. The
format version is 2; full-precision files of version 1 do not open.
