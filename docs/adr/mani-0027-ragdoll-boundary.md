# mani-0027. The ragdoll boundary

Status: Accepted

## Context

A character falls, is hit or goes limp, and a physics engine moves its
body; then it gets up and animation takes it back. The library does
not simulate (a host has its own engine) but owns the skeleton, the
poses and the graph, so the seam between the two is its to give: which
bones carry bodies and how they are joined, the targets an engine's
motors and kinematic bodies need from animated poses, poses back from
the bodies, blending the physics pose over the animation, and where and
how to get up. Jolt Physics, Unreal Engine, Godot and the glTF
`OMI_physics_*` extensions hold the same pieces: bodies on bones with
primitive shapes and masses, swing-twist joints with limits, and
drives with stiffness, damping and a largest force.

## Decision

- **Profiles** (container kind 12, built from a source and opened in
  place, every byte checked): bodies on distinct bones, a subset of the
  skeleton; each with its frame relative to its bone's model frame
  (translation and unit rotation, the bone's scale left out), its shape
  at the frame's origin (a sphere's radius; a capsule along the frame's
  y, its radius and the distance between its caps' centers; a box's
  half extents; unused sizes 0), its mass and a name. The parent of a
  body is the body of its bone's nearest ancestor with one; exactly one
  root. Bodies are kept by bone, so parents come first and the root is
  body 0. Every other body has a joint: its frame in the body, and the
  same place's frame in the parent body, worked out at build from the
  rest pose (each bone's rigid model frame M, the body's frame B = M F,
  the joint's B C, and in the parent P⁻¹ B C), so the joint is at rest
  in the rest pose; twist about the joint frame's x within
  [min, max] ⊂ [-π, π], swing about y and z within an elliptic cone of
  half angles in [0, π]; a drive's stiffness, damping and largest
  torque (all 0 for none). The root body's facing and up axes in its
  frame, unit and perpendicular, are kept for getting up.
- **Bodies from poses:** each body's frame in the world is its bone's
  model frame (the pose's model matrix, its scale left out) under the
  placement's translation and rotation (its scale left out too: a
  profile is in the skeleton's units), then the body's frame. With the
  pose before, its placement and the seconds between, a body's linear
  velocity is the difference of its positions over the time and its
  angular velocity the rotation vector of the shortest turn between
  its rotations over the time, as Jolt's kinematic drive and
  world-space drives take them.
- **Joint targets:** per joint, the child's joint frame turned in the
  parent's (w kept at least 0) and the child's angular velocity less
  the parent's about the parent joint frame's axes: the local motor
  targets of Jolt's swing-twist constraint and Unreal's local mode.
- **Poses from bodies:** bones in order; a bone with a body takes the
  model frame the body's place gives with the body's frame taken off,
  its local translation through the parent's model matrix inverted,
  its local rotation through the parent's rotation, its scale from a
  reference pose; a bone without one keeps the reference's local (no
  reorienting of chains between bodies). The model matrices are made
  in the same pass as `maniComputeModel` makes them.
- **Pose inputs in graphs:** a graph parameter of a seventh type,
  pose, rather than a list of its own: a parameter already has a name
  unique in the graph, lookup by name and index, a place in the source,
  the file, the JSON form (`"type": "pose"`, no default) and a
  subgraph's parameter mapping. Its value is 0. The instance keeps a
  pose of the graph's skeleton per pose parameter, the rest pose at
  creation; the host sets one with `maniSetGraphPose` (a pose, not a
  difference, its locals copied) and reads it with `maniGetGraphPose`;
  `maniSetGraphParameter` refuses the type. The saved state keeps each
  pose parameter's locals bone by bone after the state machines'
  blocks, checked on restore (every word finite, rotations of length 1
  within a thousandth).
- **The pose input node:** kind 17, no inputs, one data word (its pose
  parameter, checked at build and on open), a pose (never differences)
  copied from the instance's at evaluation. It holds no time: nothing
  beneath it plays, so it leads no sync group, passes no events and
  gives no root motion (the ragdoll's placement is the host's). A
  ragdoll is laid over the animation with the blend node (a weight
  parameter) or the mask node (a partial ragdoll), with nothing new.
  A select, weight, blend space position or condition reading a pose
  parameter is refused, each checking its own types.
- **Getting up:** `maniRagdollGetUpPlacement` takes the bodies and a
  ground the host names (a point on it, its unit normal, and the
  character's model forward and up axes, which no library convention
  fixes). Face up when the root body's facing (out of the chest) points
  along the normal or across it (ties face up, as Advanced Locomotion
  System's roll test). The heading is the root body's up axis (toward
  the head) on the ground's plane, reversed face up, plus the facing on
  the plane: lying, the character stands looking where its head lay
  (face down) or toward its feet (face up), as Advanced Locomotion
  System's yaw (the pelvis's, less 180 degrees face up); upright, where
  its chest faces. The sum is continuous through upright, where one
  axis's yaw is undefined; it vanishes only inverted at 135 degrees,
  where the longer projection is taken. The placement stands on the
  plane under the root body, model up on the normal, model forward on
  the heading, scale 1; a pose from bodies under it is the pose a
  get-up clip is blended from.
- **The OMI importer** follows;
  each part is added to this record as it lands.
- **Not here:** the solver, collision, creating and stepping the bodies
  (the host's engine); shapes other than the three primitives (a host
  attaches its own); inertia tensors (each engine derives them from
  shape and mass).

## Consequences

A profile maps onto any engine's bodies and constraints without the
library knowing the engine. Bones without bodies stay the animation's,
so one skeleton serves both sides without a mapper. A profile is tied
to its skeleton by identity and its joints to the rest pose: a
skeleton with another rest pose needs its profile built again.

Under a parent with a scale not the same on every axis, a bone's model
frame from a body is met as nearly as a translation, rotation and
scale can (the shear is the parent's).

Opening is fuzzed; every check is tested against a file changed at its
byte; the builder's refusals are tested with their reasons; each
joint's two frames are tested to meet in the rest pose under turned
and scaled bones. Bodies placed from a bent pose under a turned
placement make the pose back within float rounding; velocities, joint
targets at rest and bent, and every refusal are tested.
