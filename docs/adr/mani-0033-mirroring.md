# mani-0033. Mirroring

Status: Accepted

## Context

A clip of a character turning left should also turn it right, and a
motion database is twice as rich with each clip's mirror. Mirroring a
pose across the body's plane swaps left and right bones and reflects
their motion, but rigs rarely give a left bone the mirrored axes of its
right partner: reflecting local rotations as they are twists a limb
whose axes differ. Unreal's mirror table corrects every bone through
the reference pose's component-space rotations so mirroring "works
with any joint orientation"; Holden's database generator reflects
global rotations with a sign pattern that fits his rig only.

## Decision

- **The mirror map** is an asset of its own (kind 13), built offline
  for one skeleton named by identity and opened in place: the plane
  through the model's origin normal to x, y or z; each bone's partner
  (itself when unpaired), from explicit pairs first and then
  find/replace rules on the names (prefix, suffix or anywhere; the
  first rule that matches and names a bone; rules run one way, so both
  directions are given; a rule's partner must pair back); name pairs
  for float tracks, events and markers, given both ways or made by the
  rules from names the caller lists, looked up by hash. A bone in two
  pairs, a rule's partner taken or not pairing back, a name paired
  with two partners are refused with the item.
- **Corrections:** with P a bone's parent's reference model rotation
  (the identity for a root), R its own and m its partner, each bone
  keeps `A = P_b^-1 mirror(P_m)` and `B = mirror(R_m)^-1 R_b`, mirror
  negating a quaternion's two vector components off the normal; the
  reference pose is the rest unless one is given, model rotations the
  products of the root path's local rotations.
- **The pose** (Unreal's local form): bone b's mirrored local is
  `A mirror(T_m)` (the normal's component of its partner's translation
  negated, turned) and `A mirror(Q_m) B`, its partner's scale carried
  by B: `R(B)^T S_m R(B)`, an axis scale exactly when B takes axes to
  axes (each axis then takes the partner's axis B maps to it, a
  permutation found at build, every entry of R(B) within 1e-4 of 0 or
  1 in magnitude), its diagonal otherwise (exact for a uniform scale,
  the nearest axis scale for another; the bone is flagged).
  Since mirror distributes over products, this is Unreal's
  `P_b^-1 mirror(P_m Q_m R_m^-1) R_b` with the references folded in at
  build: two products and a rotation a bone, no model pass, no scratch.
  It is exact on a mirror-symmetric hierarchy (a bone's parent's
  partner its partner's parent); the map flags every bone that breaks
  it (a prop under one hand, say) and records the first, without
  refusing them, and such a bone takes its partner's local turn
  mirrored. `maniMirrorBoneFlags` reads a bone's flags (unmatched,
  scale approximate). Two names of one hash are refused at build, a
  name repeated with its partner kept once. Format version 2.
- **Clips:** a mirrored clip is baked on the clip's grid by the same
  core as a retargeted clip: each sample mirrored (an additive clip's
  differences added to the rest, mirrored, taken from the rest
  mirrored), every float track, discrete track, event and marker name
  renamed by the name pairs, the root motion moved to its bone's
  partner. A stored move is in its bone's parent's frame, so it is
  reflected and carried by the partner's A (`A t`, `A r A^-1`; the
  identity for a root) and the declared axes follow A; a graph's moves
  are in the model and are only reflected.
- **Graphs:** a graph names at most one mirror map, of its skeleton
  (kept by identity, bound like its other assets; a subgraph's is the
  graph's or taken by it). A mirror node mirrors its input's pose in
  place and takes poses only. The builder marks each node with the
  parity of the mirror nodes above it and refuses a node reached both
  mirrored and not (`maniBuildReasonMirrorParity`, naming the node and
  the reader that reaches it the other way; a subgraph shared so takes
  a second instance); the update reads the mark, so a mirrored clip,
  match or learned node gives its moves reflected and its events and
  sync markers renamed by hash (a mirrored walk follows its unmirrored
  leader foot for foot), and two mirrors cancel.
- **Databases:** an entry flagged mirrored is its clip's span
  mirrored through the database's mirror map (whose identity the file
  keeps): its frames sampled with each pose mirrored, each move
  reflected and each float track channel read from the partner track,
  exactly a mirrored bake's features without the bake. A graph whose
  database has mirrored entries names the same map; its matching node
  plays a mirrored entry mirrored (pose, moves, events), combined with
  the node's own mirror parity.

## Consequences

The map's tests pair a rig of left and right clavicles, arms, hands and
legs and a prop under the right hand by suffix rules: each bone's
partner, the prop its own and the first unmatched bone; pairs taken
before rules; names paired both ways and by the rules, a repeat with the
same partner kept once, an unpaired name returned as it is; every
builder refusal with its item (a bone past the rig or in two pairs, a
rule's partner taken or not pairing back, a rule's empty find, unknown
place, replace not UTF-8 or reserved word, a reference not finite or of
length 0, an unknown axis, a name paired twice, empty (also as a rule
leaves it) or not UTF-8, and
the misuses); every crafted word refused at its offset. A fuzz target
opens maps and reads every partner and name. The pose's tests turn that
rig's every bone and mirror it across each axis: every bone but the
prop at its partner's model position reflected, turned by its partner's
model rotation mirrored and carried to its own reference axes; the
rest's model rotations kept; mirroring twice giving the pose back; in
place equal to apart; a scale exact on the axes a quarter turn takes it
to and back, the nearest axis scale under a 30 degree turn, every
unmatched bone flagged and a turned pose of them finite and the same
bits twice; moves reflected; the
refusals (another skeleton or bone count, a pose of another skeleton, a
difference). The clip's tests bake a clip of that rig with float, discrete,
event and marker names: every sample the mirror of the clip's, the
bits pinned, baked twice the clip; names renamed and values kept;
additive clips of both bases; root motion on the root reflected and on
the left arm moved to the right arm, its model change reflected and its
axes and twist all taken; the refusals. The graph's tests play a walk
under one mirror node (each pose the walk's mirrored, each move
reflected, its events renamed) and under two (the walk as it is), a
mirrored walk following its leader by its renamed markers half a loop
on, the map read back, every builder refusal (no map, a map of another
rig, a node reached both ways by the mirror node, differences, a
subgraph's other map),
the binder's (no map, another, a crafted view of another rig) and the
opener's (an identity of 0, a section of 4 bytes, a mirror node
without the section), the JSON form built back the same, and
subgraphs taking the map. The database's tests build a walk and the
walk mirrored and compare every frame with a database over the walk
and its mirrored bake (bone positions, velocities and facings, the
trajectory and its facing, a float track from its partner), the map
kept, the builder's refusals (no map, a map of another rig, a partner
track missing), the opener's (an identity of 0, a section of 4 bytes,
a mirrored entry without the section), and a graph playing the
mirrored entry mirrored and, under a mirror node, as it is, with its
build and bind refusals.

Timings, by instruction count: a 54-bone humanoid (three spine bones,
a neck and head, five three-bone fingers a hand, legs to the toes)
whose left and right sides rest on unmirrored axes, paired by suffix
rules. Mirroring a pose takes 13,454 instructions (about 250 a bone,
some 3 microseconds; 12,428 before the scale was carried by the
correction): two quaternion products and a rotation a bone,
no model pass, about what a native clip node's update and sample
cost, so a graph's mirror node adds about one clip node a frame.
Baking a one-second clip at 30 Hz takes 2.38 million; a database
of a walk and the walk mirrored takes 9.17 million to build against
7.15 million for two plain entries, the mirrored entry's poses
mirrored as they are sampled. A search over a database whose entries
are each mirrored too costs what one over twice the plain data does
(6.24 instructions a frame against 6.22, 225,273 for 36,120 frames).
