# mani-0014. FBX transforms, inherit modes and skeletons

Status: Accepted

## Context

An FBX model's local transform is a chain of pivots, offsets and pre-
and post-rotations around its translation, rotation and scale, with
six Euler orders, and three ways to inherit a parent's scale, two of
which (segment scale compensation among them) no single composition
rule reproduces. The runtime has one rule (mani-0004): a bone's model
matrix is its parent's times its local TRS.

## Decision

- **The local transform** is evaluated in binary64 as ufbx evaluates
  it: `T Roff Rp Rpre R Rpost^-1 Rp^-1 Soff Sp S Sp^-1` is exactly one
  TRS (every factor but the rotations and S is a translation), its
  rotation `Rpre R Rpost^-1`. Rotation orders 0 to 5, 6 (spheric) as
  XYZ, pre- and post-rotations in XYZ and applied only in the rotation
  space (`RotationActive` 1 and `RotationSpaceForLimitOnly` 0). Angles
  are reduced in degrees exactly. Values not finite, missing, or an
  order or inherit type out of range are refused (Property).
- **Worlds** follow ufbx's evaluation of inherit types, with each
  model's unscaled world and inherited scale; a bone's local is its
  parent bone's world inverted times its own, decomposed.
- **Helper bones**, named `<parent>#unscaled` and holding the inverse
  of the parent's inherited scale, are added only where that local
  does not decompose exactly (a non-uniform scale under a non-normal
  child), at rest or at any sample of a stack on its default clip grid
  (the scene's rate up to 120 a second, else 30); on request under
  every parent of a non-normal child. A clip sample needing a helper
  the target skeleton lacks is refused (Matrix).
- **Skeletons** are skins' linked models and limb chains with the
  models between, merged when they share a model, rooted at their
  common ancestor; any model's subtree can be built too. Names are the
  models' names, renamed `name#index` when empty or repeated.
- **Placement, axes and units** are reported, or applied on request:
  the placement folded into the root, target axes as a signed
  permutation and a target unit, applied as `B G B^-1` with
  translations scaled.

## Consequences

FBX's whole transform model lands on the runtime's one rule exactly;
helper bones appear only in files whose scale inheritance needs them.
ufbx serves as the differential reference with no expected
differences beyond rounding.
