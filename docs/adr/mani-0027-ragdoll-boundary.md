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
  [min, max] ⊂ [-π, π] (min equal to max holds it there), swing about
  y and z within an elliptic cone of half angles each in [0, π] (0
  locks that axis, π leaves it free), each bound checked by the builder
  and on opening; a drive's stiffness, damping and largest torque (all
  0 for none). Lengths are the skeleton's units, angles radians, times
  seconds; masses and drives are the host engine's units, passed
  through, never converted or scaled. The root body's facing and up axes in its
  frame, unit and perpendicular, are kept for getting up.
- **Bodies from poses:** each body's frame in the world is its bone's
  model frame (the pose's model matrix, its scale left out) under the
  placement's translation and rotation (its scale left out too: a
  profile is in the skeleton's units), then the body's frame. With the
  pose before, its placement and the seconds between, a body's linear
  velocity is the difference of its positions over the time and its
  angular velocity the rotation vector of the shortest turn between
  its rotations over the time, as Jolt's kinematic drive and
  world-space drives take them. The shortest turn is at most half a
  turn, so an angular velocity is at most π over the time: a bone
  turning further in one step reads as the shorter turn the other way
  (at 60 Hz, past 30 turns a second). On a cut the host passes what the
  step means: no pose before on the first frame or a clip switched
  without a blend (the bodies at rest), the current placement as the
  one before on a teleport (the animation's motion alone, no spike),
  the same pose before for the placement's motion alone.
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
  the plane weighed by how upright the up axis stands (its component
  along the normal): lying on its back, front or side, the character
  stands looking where its head lay (face down) or toward its feet
  (face up), exactly Advanced Locomotion System's yaw (the pelvis's,
  less 180 degrees face up); upright, where its chest faces. The sum is
  continuous through upright, where one axis's yaw is undefined; it
  vanishes only inverted at about 142 degrees, its terms equal and
  opposite, where the head's projection is taken. (The unweighed sum,
  first built, headed 45 degrees off on a side; mutation testing found
  it.) The placement stands on the
  plane under the root body, model up on the normal, model forward on
  the heading, scale 1; a pose from bodies under it is the pose a
  get-up clip is blended from.
- **The OMI importer:** `maniGltfReadRagdoll` reads the three OMI
  extensions (Stage 1) on an opened skeleton. A dynamic or kinematic
  body sits on the bone of its node or the node's nearest ancestor that
  is a bone (bodies no bone holds, a scene's floor or props, are left
  out); its frame is its one collider's node frame, rigid, in the
  bone's model frame; its shape OMI's (a box's size halved, a sphere's
  radius, a capsule's height between the caps' centers and its radius)
  times that node's scale when even on the three axes. A joint joins a
  body to its nearest ancestor body, either way round, its two sides'
  frames together at rest within a thousandth; every linear axis
  fixed; angular limits as twist (x, within [-pi, pi]) and symmetric
  swings (y, z; a free axis pi); angular force-mode drives equal on the
  three axes and without targets as the drive, an unbounded largest
  force the largest finite float. The root's facing and up axes come
  from the caller in glTF's basis. Refused at its value with
  `maniImportReasonUnsupported`: static bodies on bones, a body without
  one collider, uneven collider scale, cylinders, convex and triangle
  meshes, tapered capsules, a center of mass off the origin, a gravity
  factor, joints between bodies not parent and child or apart at rest,
  a body without its joint, free or moving linear axes, soft limits,
  asymmetric swings, linear or acceleration-mode drives, targets,
  unequal drives. Ignored: inertia (the engines derive it), initial
  velocities, triggers, collision settings and materials. Units stay
  the file's; converting force-mode stiffness and damping to an
  engine's frequency and damping ratio needs the inertia the engine
  derives, and is the host's.
- **Mutants:** 65 of the profile's checks and arithmetic are in the
  registry. Their first run found the getting-up heading's flaw (the
  plain sum above, before its weighting), a drive read before it was
  set, and cases no test held; each is fixed and tested. One is kept
  as equivalent with its reason: a body's frame normalized once more
  changes no bit the tests or a host can see.
- **The cost:** a chain of 64 bones with a body on every fourth, a
  Release build, instructions a frame: bodies from a pose 10,788,
  with velocities 26,472, the joints' motor targets 4,376, a pose
  from the bodies 41,000. Jolt's `SkeletonMapper` on the same chain:
  `MapReverse` 807 (a copy of sixteen matrices, the quaternions left
  to `Ragdoll::SetPose`), `Map` 7,631. Most of the first cost was the
  polar factor of every body's bone matrix; a rigid split now takes
  the columns divided by their lengths when they are orthogonal, the
  polar factor exactly, and iterates only under shear (mani-0004's
  decomposition; the answers are the same within rounding). What is
  left of a pose from bodies is its scalar pass over every bone, one
  bone at a time; a pass over the pose's groups of eight bones is its
  next step.
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
placement make the pose back within float rounding; velocities (a
teleport passed as the current placement without a spike, the
placement's motion alone, either side of the half turn), joint targets
at rest and bent, every refusal and each limit's bound (the float just
past π, NaN) are tested.
