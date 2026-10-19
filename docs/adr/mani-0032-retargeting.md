# mani-0032. Retargeting

Status: Accepted

## Context

A clip made for one skeleton should play on another whose bones have
other names, other counts along the spine and limbs, other rest
orientations and other proportions. Matching names fails across
different rests, and rewriting a rig's rests to a profile loses data
(Godot's documentation warns so). Godot's runtime retargeting carries
each bone's turn from its rest through the rests' model frames; Unreal's
IK Retargeter adds chains of different lengths, a pelvis whose motion
scales with the characters' heights, and IK goals after the transfer.

## Decision

- **The map** is an asset of its own (kind 4), built offline for two
  skeletons named by identity and opened in place. Each target bone
  has a plan: its reference (unmapped), one source bone (a pair, a
  chain's bone, the pelvis, a name), or two neighbors along a chain and
  its place between them; a rotation weight; its translation (its own,
  the source's change scaled, the source's itself, or the pelvis's) and
  whether its scale carries. Both reference poses are stored as each
  skeleton's rest or a pose given for it, with their model rotations
  (each the product of its root path's local rotations).
- **Chains** of any lengths on each side (at most 32 bones) map their
  bones by rotation mode: none, interpolated (each bone placed by its
  distance from the chain's start over the chain's length along the
  reference pose, by index when the chain has none, and a target bone
  between the two source bones around its place), one to one from the
  start, or from the end; extra target bones keep their reference.
- **The pelvis** stores the ratio of the target reference pelvis's
  height over the source's along an up axis, times horizontal and
  vertical factors, the per-axis blends toward the unscaled change and
  toward the source's position, and a translation weight. Without a
  pelvis its fields have one form (the axis +y, the scales 1, the rest
  0), so a map has one file.
- **Building** checks the references (finite, rotations within a
  thousandth of length 1), pairs, chains, the pelvis and names in that
  order, each target bone mapped once; refusals name the item and, for
  a bone mapped twice, the bone. **Opening** checks every word in its
  one form (a reference plan all none and 0, a one-source plan's place
  0, a two-source plan's place inside (0, 1), the pelvis's plan its
  source and translation) and the identity; what needs the skeletons
  is checked where the map is used, by their identities.

## Consequences

The tests build maps between a source of three spine bones and a
two-bone arm and a target of four spine bones and a three-bone arm:
each target bone's plan read from the file (an interpolated spine's
ends on the source's ends and its middle bones between two at places
2/3 and 1/3; one to one leaving the extra arm bone at its reference;
one to one reversed; the pelvis with heights 1 and 0.9; the root and
head by name); a given reference pose's locals and model rotations
stored and the places taken from its lengths; every builder refusal
with its item; every crafted word refused at its offset. A fuzz target
opens maps and reads every plan.
