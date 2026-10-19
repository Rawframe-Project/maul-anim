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
- **The transfer** (`maniRetargetPose`): model rotations are each root
  path's local rotations' product on both sides. A mapped target bone's
  model rotation is its source bone's times the inverse of the source's
  reference model rotation (two bones' such turns slerped at the
  bone's place), slerped from none by its weight, times its own
  reference model rotation, and its local is taken against its parent
  as just made: Godot's global form, which is right across bones only
  one rig has, with the target's own lengths. Unmapped bones keep
  their reference locals. A scaled translation is the target
  reference's plus the source's change from its reference times the
  height ratio, turned from the source parent's reference model frame
  to the target parent's (Godot's local form for translations); an
  absolute one the source's own, turned the same; a carried scale its
  reference's times the source's over the source reference's. The
  pelvis's model position is the target reference's plus the source
  pelvis's change, its horizontal and vertical parts scaled, blended
  per model axis toward the unscaled change and then the source's own
  position, weighted, and taken into its parent's frame as made.
  `maniRetargetRootMotion` scales a root motion delta the same way.
  The caller lends scratch for a rotation per bone of both skeletons
  and a matrix per target bone; nothing is kept.
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

The transfer's tests, between rigs whose rests turn spine and arm bones
on both sides, scale the source's hand and offset the target's root:
the source's reference giving the target's (all but the pelvis's
translation, which its source blend pulls toward the source's own
place); on a posed source with a moved and turned root, each mapped
bone's model turn from its reference the source bone's within 1e-11 of
a binary64 one (the interpolated spine's middle bones the slerps at
2/3 and 1/3, a chain of weight 0.5 half the turns); an extra bone at
its reference local bit for bit and own translations kept; a reversed
chain's reference bone between mapped ones; a scaled translation
turned from the source arm's reference frame into the turned upper
arm's and the scale's ratio carried; the pelvis by its formula in
binary64 (scaled by parts, blended, weighted 0.75, under the turned
root); root motion scaled; the refusals (another identity of the same
count, a target pose of another rig, a source of differences, scratch
short or misaligned) and the output marked a pose; a transfer's bits
pinned, the same under gcc and clang. Its first mutants found the
first rig too regular to tell the reference turns, the parents' paths
and the weights apart.
