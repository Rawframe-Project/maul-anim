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
  negated, turned) and `A mirror(Q_m) B`, its partner's scale as it is.
  Since mirror distributes over products, this is Unreal's
  `P_b^-1 mirror(P_m Q_m R_m^-1) R_b` with the references folded in at
  build: two products and a rotation a bone, no model pass, no scratch.
  It is exact on a mirror-symmetric hierarchy (a bone's parent's
  partner its partner's parent); the map records the first bone that
  breaks it (a prop under one hand, say) without refusing it, and such
  a bone takes its partner's local turn mirrored.
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
  mirrored and not; the update reads the mark, so a mirrored clip,
  match or learned node gives its moves reflected and its events and
  sync markers renamed by hash (a mirrored walk follows its unmirrored
  leader foot for foot), and two mirrors cancel.

## Consequences

The map's tests pair a rig of left and right clavicles, arms, hands and
legs and a prop under the right hand by suffix rules: each bone's
partner, the prop its own and the first unmatched bone; pairs taken
before rules; names paired both ways and by the rules, a repeat with the
same partner kept once, an unpaired name returned as it is; every
builder refusal with its item (a bone past the rig or in two pairs, a
rule's partner taken or not pairing back, a rule's empty find, unknown
place, replace not UTF-8 or reserved word, a reference not finite or of
length 0, an unknown axis, a name paired twice, empty or not UTF-8, and
the misuses); every crafted word refused at its offset. A fuzz target
opens maps and reads every partner and name. The pose's tests turn that
rig's every bone and mirror it across each axis: every bone but the
prop at its partner's model position reflected, turned by its partner's
model rotation mirrored and carried to its own reference axes; the
rest's model rotations kept; mirroring twice giving the pose back; in
place equal to apart; the partner's scale; moves reflected; the
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
rig, a node reached both ways, differences, a subgraph's other map),
the binder's (no map, another, a crafted view of another rig) and the
opener's (an identity of 0, a section of 4 bytes, a mirror node
without the section), the JSON form built back the same, and
subgraphs taking the map.
