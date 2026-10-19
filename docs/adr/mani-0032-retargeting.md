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
  The caller lends scratch for a rotation per bone of both skeletons;
  the pelvis's parent's matrix is taken from the locals made so far;
  nothing is kept.
- **Baking clips** (`maniBuildRetargetedClip`, offline): a clip of the
  source becomes a full-precision clip of the target on the same grid
  (rate, duration, loop policy, additive basis): each sample's pose
  carried by the transfer into linear keys at the sample times, a
  channel left out where it is the target's rest at every sample; a
  compressed source is sampled as it plays. An additive clip's
  differences are each added to the source's rest, carried, and taken
  as differences from the source's rest carried (so the source's rest
  carries to no difference), then stored against the target's rest;
  the first-frame basis keeps its zero first sample. Float tracks keep
  their samples and interpolation (a step stays a step); discrete
  tracks, events and markers are copied. Root motion carries as the
  source's stored moves, not extracted again: a translation turned
  from the root motion bone's parent's reference model frame into the
  model, scaled by `maniRetargetRootMotion`, and turned into the
  target bone's parent's frame, a rotation turned between the two
  frames; the target bone is the first that follows the source's alone
  or one the caller names. The declared axes follow the same map, and
  where it takes an axis off the axes (a parent's frame turned by
  other than quarter turns, or the up axis tilted against unequal
  scales) all three axes and the whole rotation are declared, which
  stays true of the moves. Unreal's retargeter copies the source
  root's motion scaled by the relative height (or makes it from the
  target's pelvis), and Godot divides position tracks, the root's
  among them, by the hips' height at import and multiplies them by
  the target's: both scale the root's translation by the heights.
  Carrying the stored moves does the same, independent of the root
  bone's translation mode and exact at the samples.
- **Playing in graphs:** a graph's clip slot may name a map (the clip
  then of the map's source); the graph keeps a map slot per map (its
  identity and its source skeleton's) and each clip slot's map slot.
  The map belongs to the slot, not the clip node, so every reader of
  the slot (clip nodes, events, markers and sync, root motion) sees
  one clip carried one way. Binding takes the maps and their source
  skeletons; a mapped clip's root motion names the target bone that
  follows its own alone (none refuses the binding). Evaluation samples
  a mapped clip into a pose of the source and transfers it in a part of
  the scratch sized at binding, its poses made once per source
  skeleton; an additive clip is carried as the bake carries it, by two
  transfers. Each update's move is carried as the bake carries the
  stored ones, here on the move between two times: exact where the
  move's starting rotation turns about the up axis or the scales are
  equal. The JSON form needs nothing new: its clip slots are the
  host's named clip sources, which carry their maps.
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

The bake's tests carry a clip with a moving, turning root, a tilting
pelvis, a bending spine, a turning arm and a scaling hand: every one
of its 31 samples within 1e-5 (rotations within 1e-10 of their dot
product's 1) of the transfer of the source's, from the full-precision
and the compressed source; the timing kept; a linear float track's
values and a step one's steps, two discrete tracks, two events and two
markers kept; a map of chains and the pelvis only leaving the root,
the head and the hand past the arm chain's source out (7 tracks of
10); rest and first-frame additive clips' differences carried from the
source's rest; root motion's moves over three spans the source's
scaled, with the poses in place; a root whose rest turns a quarter
about x turning the hips' x and z axes into x and y and the yaw twist
into one about z, the moves turned with them; the arm's motion under
chests whose rests turn apart carried into the model and out again,
off the axes, so all axes and the whole rotation declared; a
horizontal scale of 0 keeping the walk's axes; the refusals (each
skeleton and the clip of another identity of the same count, a root
motion bone past the target, none following it alone, an additive
clip over a basis of scale 0); the baked clip's identity pinned. Their
first mutants found a two-source plan taken for a follower, the source
frame, the zero-scale axes, a second discrete track's keys, the
clip's identity check (its count caught the target's clip) and the clip
builder's checks of given motion
untested.

The graph tests play a walk with root motion and an additive lean of
the source, both mapped, under an additive node of the target's graph:
over twelve updates of 0.1 s (one wrapping the loop) each pose within
1e-5 (rotations within 1e-10 of their dot product's 1) of the walk
carried plus the lean carried by two transfers, each move the walk's
scaled; the map slot read back; the builder's refusals (a map for
another target, a source clip without a map, clips of the target and
of a rig of the source's count through the map, a map past 256), the
binder's (no map, no source skeleton, one of another rig,
only a map for another target, maps NULL with a count, the map slot's
place after a spring set slot, root motion no target bone follows);
two graphs whose maps take different source rigs evaluated in turn
with one scratch, each carrying its own clip; the opener's (a map slot past the maps, a map
twice); the JSON form built back to the same file; a subgraph playing
the walk through the same map joining its slot, through another map
taking one of its own.

Timings, by instruction count (wall times taken on a loaded machine):
a 53-bone humanoid source (three spine bones, a neck, five three-bone
fingers a hand) to a 60-bone target (five spine bones, two neck bones,
a twist bone in each upper arm and forearm), mapped by interpolated
spine, neck and arm chains, the pelvis and names. The transfer took
49,056 instructions, a third of them composing a model matrix for
every target bone that only the pelvis's parent's inverse reads; that
matrix is now taken from the locals made so far, and the transfer
takes 33,427 (about 7 microseconds) with the same bits, its scratch a
rotation a bone. About a third of what remains is normalizing each
bone's model and local rotations in binary64, kept. A graph's mapped
clip node updated and evaluated takes 44,738 against 12,474 for the
baked clip played natively; baking the one-second clip at 30 Hz takes
2.8 million. The budget: a humanoid's transfer within 50 thousand
instructions (about 0.01 milliseconds) on one core, met; a crowd that
shares clips bakes them, a character that plays another rig's clips
live pays about three native clip samples more a frame.
