# mani-0020. Blending: blends, masks, differences and additive layers

Status: Accepted

## Context

Graphs (mani-0006's sampling feeds them) mix poses by weights, per bone
through masks, and add differences on top. The layer below graphs is
public (mani-0004): free functions over poses in caller memory. ozz's
blending job is the reference both for its rules and for the speed to
meet: it sums layers in order with the rotation's sign taken against
the running sum, lets the rest pose fill in below a threshold, and
normalizes. Its estimates make its bits depend on the instruction set,
and it zeroes negative weights silently.

## Decision

- **Blend:** `maniBlendPoses` takes layers (a pose, a weight, optional
  per-bone weights) in order, a threshold and an output. Rotations add
  on the arc that agrees with the sum so far, translations and scales
  linearly. Where a bone's total is below the threshold, its rest fills
  the shortfall, so poses slide into the rest as weights fall; where
  nothing weighed, the rest is copied. Rotations are normalized exactly
  (the identity for a zero sum), translations and scales divided by the
  total. Graphs use a threshold of 1e-5 (`MANI_BLEND_THRESHOLD`).
- **Weights** are finite and at least 0; anything else, or a threshold
  not above 0, refuses the call before the output is touched.
- **Masks** are a float per bone in skeleton order. `maniBuildMask`
  makes them from chains: a chain on the bone alone first, else the
  subtree chain on the nearest ancestor or the bone, else a default;
  the later of two chains on one bone and kind wins.
- **Kinds:** a pose records whether it holds a difference. Blends take
  layers of one kind and write it (a difference's rest is the
  identity); the model pass and subset sampling refuse differences;
  making a pose, setting its rest and sampling a whole clip of poses
  make a pose of the skeleton. Differences from a basis and additive layers
  (`t + w d_t`, `r nlerp(1, d_r, w)`, `s (1 + w (d_s - 1))`, weights
  at least 0, no subtraction) follow the same rule.
- **Additive clips:** a clip source names a basis, the skeleton's rest
  or the source's first frame, and the builder stores each channel as
  its difference from it, computed in binary64. The header's last word
  holds the basis (readers that predate it refuse a non-zero word).
  Sampling an additive clip writes a difference, its channels without
  keys the identity; a subset writes into a pose of the clip's kind.
  The compressor folds channels to the identity, not the rest, and
  measures every difference applied to the rest at weight 1, whichever
  the basis (the first frame is not kept).
- **Bits:** binary32 in a fixed order, no contraction or estimates; the
  same on every platform, pinned by a hash in the tests.

## Consequences

The blend agrees with ozz's rules, so the differential compares within
ozz's estimate error, and with a binary64 reference of the same rule
to binary32 precision. Order matters, as in ozz and Unreal, and is
stated. Refusing negative weights moves a caller's mistake to the call
that made it. Carrying the kind in the pose keeps poses and differences
apart below graphs as graphs keep them apart when built.
