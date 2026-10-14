# mani-0003. Transforms, skeletons and their files

Status: Accepted

## Context

Skeletons come from glTF, FBX and code, in any order and with any
number of roots, and every other part (clips, graphs, masks, IK,
retargeting, skinning) indexes their bones. glTF composes nodes as
matrices, so a non-uniform scale on a parent shears its children; a
format that refuses that, or that needs a flag per bone, cannot play
what was authored. Opening a file must check it completely without
allocating (mani-0001, mani-0002).

## Decision

- **Transforms:** a bone's local transform is a translation, a unit
  quaternion (x, y, z, w) and a scale per axis, applied scale, then
  rotation, then translation (`maniTransform`). A bone's model matrix
  is its parent's times its local matrix, a 3x4 affine matrix
  (`maniAffine`), exactly as glTF composes nodes; mirrors and shear are
  exact.
- **Skeletons:** bones in depth-first order (every subtree a
  contiguous range, children and roots in their authored order),
  16-bit indices, `MANI_NO_BONE` for no parent, at most 65,535 bones,
  the only limit. Each bone has a unique, non-empty UTF-8 name.
- **The skeleton file** (container kind 1, format version 1) holds
  parents, subtree ends, rests, name hashes, a name order by hash, name
  spans and name bytes. Its identity is a hash of the bone count,
  parents, rests, name spans and name bytes; the other sections are
  derived from those and checked against them at open.
- **Opening** (`maniOpenSkeleton`) checks every section in one pass and
  no scratch memory: parents before children and depth-first order
  (each bone's parent is the previous bone or one of its ancestors,
  every subtree end checked as the walk closes it), finite rests and
  unit rotations, well-formed names matching their hashes, a name
  order with strictly increasing hashes (so names are unique and found
  by binary search), and the identity. The view then reads without
  checks; its getters answer bones past the count with defined values.
- **Building** is offline (`maniBuilder`, created from a def with an
  allocator, one object for every kind of file): bones in any order
  with parent indices, each refused with its index and a reason, are
  ordered depth-first, rotations normalized in binary64, and the file
  kept for a copy into caller memory with the map from source bones to
  skeleton bones.

## Consequences

Any glTF hierarchy plays as written, every range of bones a part needs
is one contiguous span, binding by name is a binary search over data
the file already holds, and a skeleton file is as hostile-input-safe as
the container it sits in. A later format version that adds a section
not derived from the identity's sections adds it to the identity.
