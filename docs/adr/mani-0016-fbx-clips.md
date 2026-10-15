# mani-0016. FBX clips

Status: Accepted

## Context

An FBX file animates through stacks (takes), each holding layers of
curve nodes that drive properties of models, each channel a curve of
keys with Bézier tangents in several kinds (auto with bias and
clamping, TCB, user, weighted), constant and linear segments, and
extrapolation before and after its keys. Layers combine by weight in
additive, override and passthrough modes, rotations as Euler angles.
None of this is one of the clip builder's source interpolations
(mani-0005), and a model's local transform is a chain of properties
(mani-0014) any of which may be animated.

## Decision

- **Clips** are the file's `AnimationStack`s in object order, built one
  at a time for an opened skeleton, as glTF animations are
  (mani-0012). A clip's range is the stack's local times, else its
  reference times, each its own or its template's, as ufbx finds them;
  a stack with no properties of its own takes its take's local and
  reference times as its own, ahead of the template; else the span of
  its keys. Clip time 0 is the range's start.
- **Curves** are decoded and evaluated as ufbx does, in binary64 with
  tangents rounded to binary32 where ufbx keeps them: keys and their
  run-length attributes become Bézier handles; segments are constant
  (previous or next value), linear, or cubic, solved for the time by
  Newton's method; outside the keys each end extrapolates as constant,
  along its tangent, or by repeating (plain, relative, mirrored) in
  whole ticks. Curves whose arrays disagree in size, whose runs leave
  keys uncovered or whose times go back are refused.
- **Layers** combine in connection order with their weights (animated
  too), blend modes and accumulation modes, rotations through
  quaternions and back to Euler angles in the model's order.
- **Sampling:** every animated property feeds the models' locals and
  worlds at each sample of the clip's grid; each bone's local is
  formed exactly as its rest was (its own transform, compensation,
  helper bones), refused where it does not decompose. The importer
  gives the builder linear keys at the sample times. The default rate
  is the scene's frame rate up to 120 a second, else 30; sample times
  are computed in whole ticks so keys on frames land on samples. A
  bone gets a track only when it moves.
- **Placement, axes and units** apply as they do to the skeleton:
  folded placement composes the worlds above the root per sample;
  reported placement leaves animation above the root out.
- **Other animated numbers** of a model become float tracks named
  `bone/property` (with `/` and a component index for vectors), as
  pointer tracks are for glTF.

## Consequences

Clips play what authoring tools play to within the sample grid, agree
with ufbx's evaluation for the differential, and stay consistent with
skeletons built from the same file, helper bones included. Sampling at
the scene's frame rate keeps authored keys exact; anything between
frames is approximated by the builder's linear samples, measured by
its reported error.
