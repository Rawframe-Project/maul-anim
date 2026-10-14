# mani-0009. glTF skeletons and skins

Status: Accepted

## Context

glTF has no skeleton object: a skin lists joint nodes, which must share
a common root, and several skins (a body and a head mesh) often animate
one rig. Node names are optional and may repeat. Hosts differ in up
axis and handedness, and a skeleton's root usually sits under scene
nodes that place the whole character. Hostile files can be shaped to
make naive tree walks quadratic.

## Decision

- **Hierarchy,** checked at open: child indices in range, no node with
  two parents, no cycle. Depths come from walks that stop at known
  nodes; common ancestors from a binary lifting table, so every step is
  linear or n log n whatever the document's shape.
- **A skin's bones** are its joints' deepest common ancestor and every
  node on the paths from it to each joint, so a skeleton has one root
  and nodes between joints are bones without palette entries. Skins
  whose bones share a node form one skeleton (their union, rooted at the
  common ancestor of their roots), numbered by their lowest skin. The
  grouping walks claim each node once and jump past an earlier skin's
  root when they meet its nodes. `skin.skeleton` is checked as an index
  only. Any node's subtree may also be built as a skeleton
  (`maniGltfBuildNodeSkeleton`).
- **Rests:** a node's TRS, or its matrix decomposed (mani-0004's
  decomposition), refused when projective or when the result does not
  compose back within 1e-4 of the largest column (the shear glTF
  forbids). Inverse binds must be affine within 1e-5.
- **Names:** a node's name, or, when it has none or repeats an earlier
  bone's in node order, the name followed by `#` and the node index.
  A collision after that is refused. Skins (and clips) find bones by
  these names.
- **Placement:** the product of the nodes above the root, reported by
  `maniGltfSkeletonPlacement`, or folded into the root's rest on
  request. Inverse binds need no change either way: the palette times
  the placement is the bind's world inverse.
- **Basis change:** a signed permutation of axes, applied exactly as
  B G B^-1: translations and rotation vector parts permuted and signed
  (rotations also times det B), scales permuted, matrices conjugated.
- **Skins** (`maniGltfBuildSkin`): joints found by name in an opened
  skeleton, inverse binds from a MAT4 float accessor without a stride
  (identity when absent), repeated joints refused.

## Consequences

The tests prove the pieces agree end to end: inverse binds computed as
the true world inverses give identity palettes at the bind pose,
reported or folded, under each basis change; skin grouping matches a
brute force over random forests.
