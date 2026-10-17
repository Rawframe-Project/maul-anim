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
- **Targets, velocities and poses from bodies, pose inputs in graphs,
  getting up and the OMI importer** follow in the order given; each
  part is added to this record as it lands.
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

Opening is fuzzed; every check is tested against a file changed at its
byte; the builder's refusals are tested with their reasons; each
joint's two frames are tested to meet in the rest pose under turned
and scaled bones.
