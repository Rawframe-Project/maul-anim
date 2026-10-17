# mani-0026. Spring bones

Status: Accepted

## Context

Hair, tails and cloth strips sway behind a character's motion without a
physics engine. VRM 1.0 carries them with avatars as
`VRMC_springBone-1.0` (chains of joints, colliders, centers) and two
complete extensions, `VRMC_springBone_extended_collider-1.0` (inside
spheres and capsules, planes) and `VRMC_springBone_limit-1.0` (cone,
hinge and spherical limits); mani-0011 reads the base extension into
sources. VRM's reference integrates once a rendered frame: the inertia
carries the last frame's displacement whole while stiffness and gravity
scale with the frame's time, so the motion changes with the frame rate,
and content is tuned at the rate its author previewed it (60 Hz on a
common display; 50 Hz for UniVRM's fixed update). The library's results
are the same bits everywhere (mani-0001) and must not depend on how a
host splits time.

## Decision

- **Sources:** colliders gain the plane shape and inside spheres and
  capsules; joints gain a limit (none, cone, hinge, spherical: angle,
  or pitch and yaw, and a rotation); a set gains its reference rate (60
  Hz by default) and step rate (the reference rate by default), both up
  to `MANI_SPRING_MAX_RATE`.
- **The file** (container kind 11): the skeleton's identity, the rates,
  colliders, springs (their joints, the colliders of their groups in
  order without repeats, a center bone, a name) and joints, each a
  segment from its bone to the next joint's. `maniBuildSpringSet`
  refuses what VRM forbids (a chain whose joints are not each below the
  last, a joint in two chains, a center neither the first joint nor
  above it, or on or below another chain's joint) and every value out
  of its range, each refusal with its reason and items.
- **Fixed steps:** each joint's drag d, stiffness and gravity, given per
  frame of the reference rate f0, are converted at build to a step of h
  seconds: the drag 1 - (1 - d)^(h f0), stiffness and gravity times
  h (dh / d) (h² f0 for d = 0). This makes the free motion (inertia,
  drag, stiffness, gravity) the same at any step, the solution of the
  equation the reference integrates exactly over each of its frames; at
  the default step, the reference's own.
- **Limits** keep their angles clamped as the extension says and a
  frame per joint built from the head's rest direction to its tail (the
  turn from +y, 180° about x for exactly -y) and the limit's rotation.
- **Import:** `maniGltfReadSprings` (mani-0011) reads a collider's
  `VRMC_springBone_extended_collider` in place of its fallback shape
  and a joint's `VRMC_springBone_limit` but the last's, each of version
  1.0 (another version is ignored, as a reader without it would, which
  the fallback exporters write is for), every value in its range and
  each refusal at its value. Under a basis change B the limit's frame
  must be glTF's frame F carried over, B F M with M the mirror of the
  limit's x when det B < 0 (every limit is symmetric in x), so the
  rotation read is turned to give that frame from the output's rest
  direction; with no basis change it is kept as written.
- `maniOpenSpringSet` opens a file in place, every record checked;
  the skeleton the set was built for is checked where a state is made.
- **The state** is a block of 32-bit words after a small header in
  caller memory, holding no pointer in the block, so a graph instance
  can hold it and a save copies it: per joint its tail now and a step
  before, and the last call's inputs (heads, tails, colliders' and
  centers' bones in model space, the placement), with the time since
  the last step as binary64.
- **Steps:** `maniStepSprings` runs the steps due at the step rate, at
  most `MANI_SPRING_MAX_STEPS` a call (a longer delta loses the rest,
  as fixed-step physics does), each with its inputs interpolated
  between the last call's and this one's at its time (positions
  linearly, rotations along the shorter arc), so the motion does not
  depend on how a host splits time. Per spring, from the root:
  `next = now + (now - before)(1 - drag) + stiffness * the pose's
  direction + gravity`, both times the head's largest absolute scale;
  then the bone's length from the head, the limit, and for each
  collider in order a push and the length and limit again, as VRM
  orders them. The pose's direction and length are read through the
  rigid motion that the turns above have made, so the result is the
  reference's, which reads each joint's turned parent. Gravity is in
  the world; tails keep in the center bone's space, or in model space
  as the last step placed it, carried into each step's placement, so
  the character's motion in the world sways them. The first step
  places every tail where the pose puts it.
- **Output:** `maniApplySprings` turns each joint toward the last
  step's tails moved on at their velocity for the time since that step:
  the last step's own at a host rate the step rate divides (no lag
  behind the steps), continuous to the second difference across a step
  at faster hosts, the tails going on with heads that the pose moved
  (interpolating between the last two steps, Fiedler's render, lagged a
  step and pointed tails behind moving heads). Its local rotation turns
  by `P* q P`, q
  the model-space turn and P its parent's model rotation as the chain
  has turned it, mirrored (`D q D`, D the scale's signs) under a parent
  whose model scale is a mirror. A mirrored head mirrors its limit's
  frame likewise; the limits are alike under the x mirror a model
  scale holds.
- **Spring set slots in graphs:** a graph lists its spring sets as it
  lists learned models (an opened set of its skeleton, a name; section
  tag 13, its count in the header, now 64 bytes), bound by identity
  through `maniGraphAssets`; the JSON form names them in a "springSets"
  list; subgraphs carry them.
- **The graph node** (kind 16): one input, a pose of the skeleton (not
  differences); a spring set slot; position (vec3), rotation (quat)
  and reset (trigger) parameters, each or none. The update adds the
  time its parents give it (scaled, its magnitude) to the node's block;
  its reset trigger and a seek above it drop the motion. The evaluation
  steps the block with the input pose it has just computed and turns
  the joints, giving the same bits as the standalone state over the
  same poses, placements and deltas. The block (the time kept, binary64,
  then the state's) is the instance's and saved with it, checked on
  restore as the standalone state is; binding checks the set's state
  words against the node's, at most 2^24. Subgraphs carry the node and
  its slot; the JSON form names it "spring".
- **Saved states** (`MSST`, the set's identity, the size, the block)
  are checked as external bytes on restore: every float finite,
  rotations unit within a thousandth, scales not negative, the clock in
  its step, zeros without motion.

## Consequences

A set's motion is decided once at build from its source and rates, and
plays the same on hosts of any frame rate; content tuned at another
rate than 60 Hz says so in its reference rate. Between steps the
output is a short extrapolation, which may cross a collider or limit by
up to a step's motion until the next step. VRM's push and then length
lets a tail pressed on a collider back in by a little, as in the
reference. A seek beneath a spring node jumps its input without
dropping the motion; the reset trigger is the host's for such cuts.

Opening is fuzzed; every check is tested against a file changed at its
byte; the conversion is tested against its binary64 formula. The steps
are tested against a binary64 reference of the update; at host rates
of 30, 60 and 144 Hz with the character still, moving and turning, in
model and center space; a character turned, moved and scaled against
the same one at the origin; a character on its back against gravity
turned in the set; colliders and limits on a mirrored bone; and every
check of a saved state against bytes changed at its word. The graph
node is tested against the standalone state bit for bit over 90 frames
of mixed deltas with a reset and a seek, and over a blend under a time
scale with updates summed and backward; its saved block (a rotation
restored off unit length included), refusals, opening's checks (blocks
past 2^32 words included), binding, JSON form and subgraphs; the graph
state and JSON fuzz targets' hosts hold one. The importer is tested on every
shape and limit, the fallbacks kept, a last joint's malformed limit
unread, each refusal at its value, and the limit's frame against glTF's
under a proper, a mirrored and a turned and mirrored basis; its sources
build.
