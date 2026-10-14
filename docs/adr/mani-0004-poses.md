# mani-0004. Poses in caller memory

Status: Accepted

## Context

Every part of the library reads and writes poses: sampling and blending
in local space, IK and skinning in model space. They are made per
character and per thread, saved with instances and sent between
machines, and the runtime allocates nothing (mani-0001). The fastest
layout for passes over many bones (components in runs for vectors) is
not the one a host wants to read (a transform per bone), and it may
change as the passes are measured.

## Decision

- **Memory:** a pose lives in caller memory of `maniPoseSize`
  bytes at `MANI_POSE_ALIGNMENT`; `maniInitPose` makes it from a
  skeleton, at rest. It holds no pointers, so copying its bytes copies
  the pose.
- **Opaque layout:** a header recording the bone count and the
  skeleton's identity, local transforms per component in groups of
  eight bones, model matrices as 3x4 columns. The layout belongs to the
  library and may change between minor versions; hosts read and write
  through functions.
- **Access:** local transforms per bone and in ranges as
  `maniTransform`; model matrices per bone and in ranges as
  `maniAffine`. Setting a local marks the model stale, which
  `maniPoseModelCurrent` reports.
- **The model pass** `maniComputeModel` composes each bone's model
  matrix as its parent's times its local matrix (mani-0003), in bone
  order, and refuses a skeleton whose identity is not the pose's.
- **Decomposition** (`maniDecomposeAffine`, `maniPoseModelTransform`)
  splits a matrix into translation, rotation and scale: the rotation of
  the polar decomposition, computed in binary64 by the scaled Newton
  iteration (N. J. Higham, 1986), so it is exact up to rounding without
  shear and the nearest rotation with it; a mirror as a negative x
  scale; a matrix with a zero or flat axis split from the axes left,
  never refused.

## Consequences

Hosts see plain transforms and matrices; the library keeps the freedom
to lay poses out for speed; a pose given the wrong skeleton is refused
by one comparison; and a pose can be saved, copied or sent as bytes.
