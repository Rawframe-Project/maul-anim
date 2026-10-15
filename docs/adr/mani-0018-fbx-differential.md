# mani-0018. FBX fixtures and the ufbx differential

Status: Accepted

## Context

The FBX import follows ufbx's reading wherever FBX leaves a choice
(mani-0012 to mani-0017), so ufbx is its reference: a file both read
should give the same skeletons, skins, clips and weights, and a file
one refuses the other should too, but for reasons recorded. ufbx keeps
a corpus of 690 FBX files from Maya, 3ds Max, MotionBuilder, Blender
and others, versions 2000 to 8000, ASCII and binary, many of them made
to break readers. Its licence covers the repository as a whole; files
from many tools are not marked one by one, and the corpus weighs 62 MB.

## Decision

- **The differential** is `tools/fbx_differential.c`, built only with
  `MAUL_ANIM_DIFFERENTIAL` against ufbx fetched at a pinned commit and
  never linked into the library. For each file it compares whether
  each opens, the frame rate, every skeleton's rest worlds (the
  placement times each bone's model matrix, against ufbx's node
  worlds, since inherit modes are baked into rests; in a file of no
  skeleton, every top model's subtree), every skin's joints and
  inverse binds (against the clusters' geometry-to-bone matrices, or
  where ufbx has none, the same fallbacks from its worlds), and every
  clip of the first skeleton, built with the placement folded, at five
  of its sample times: bone worlds against
  ufbx's scene evaluated at the same stack time, and float tracks
  against blend shapes' effective weights or nodes' properties.
  Values agree within 1e-4 of their size (at least 1); a matrix
  element's size is its column's length.
- **Deviations** are listed in `tools/fbx-deviations.txt`, a file name
  part (or `*`) and an aspect each, with the decision behind it: they
  are reported and not counted. Anything else that disagrees is a
  defect, fixed in maul-anim or ruled a deviation by a decision first.
- **Where it runs:** a scheduled CI job, weekly and by hand, over the
  whole corpus fetched at the same commit, under the sanitizers; not in
  the commit gate, as it fetches and builds ufbx.
- **Fixtures:** scenes written for maul-anim, kept as ASCII under
  `test/data/fbx` and converted to binary forms (13- and 25-byte
  records, deflated arrays, big-endian) by `tools/fbx_binary.py`,
  which shares no code with the library. Their expected values are
  ufbx's, written by the differential's `--expect` into a header the
  commit gate's tests read: only checks that agree, deviations left
  out, and nothing when any other check disagrees. The scheduled job
  writes the header again and holds it against the committed one,
  values within 1e-6 (libm and build flags move the last digits).

## Consequences

The first full run found real defects, all fixed: helper bones were
decided from the rest alone though the file's animation can need them,
and files ufbx reads were refused (two objects of one id, binary
records with an end of 0, words in ASCII arrays, a model of several
parents). Every later change to the import faces the same corpus. The
corpus stays outside the repository, and the commit gate stays fast.
