# mani-0023. Graphs

Status: Accepted

## Context

A character's animation is chosen and mixed by a graph: clips blended
by parameters, masked per bone, layered with differences, switched,
slowed and seeked. Hosts author graphs, ship them as data, and play
one graph on many characters at once, on several threads, saving and
restoring each character's state for rollback. The library's other
assets are views over the caller's bytes (mani-0002), and nothing at
runtime allocates (mani-0001).

## Decision

- **Shape:** a graph is a DAG. A node may feed several parents and is
  then played and evaluated once, so one playhead serves every reader.
  Nodes are stored in topological order, each after its inputs, the
  output last, as the builder orders them: depth first from the output,
  each node's inputs in their order.
- **Built against one skeleton and its clips:** the builder takes an
  opened skeleton and opened clips; a clip slot keeps its clip's
  identity and a name. Masks become per-bone weights at build. Each
  node is marked as giving poses or differences, and a graph that
  blends one into the other, adds a pose as a layer, adds onto
  differences, or outputs differences is refused.
- **Slots:** each node's pose has a slot, taken before its inputs'
  slots are freed and reused once its last reader has run, so
  evaluating needs a pose per slot, a number the file states.
- **Parameters:** bool, int, float, vec2, vec3, quat (normalized) and
  trigger, each with a default and, for int and float, optional bounds;
  a number a node takes is a literal or a float parameter.
- **Nodes:** clip (a slot, a sync group, a speed, a loop override, a
  leader flag), blend (normalized or direct weights), mask (two inputs
  and per-bone weights), additive (a base and layers with weights),
  select (by an int or bool parameter), time scale, and seek (by a
  trigger, to a time).
- **The file** (kind graph, format version 1): a header of counts and
  the skeleton's identity, then parameters, node records, inputs, node
  data words, clip slots, sync groups, mask weights and names. Opening
  checks every record as hostile input: kinds, counts, every input
  before its node and in another slot, data in range, what each node
  gives as its inputs make it, names and their hashes, and the
  identity. A file that passes but was not made by the builder can give
  a wrong pose, never an unsafe access.

- **Binding:** each clip slot is bound to the host's clip of its
  identity, of the graph's skeleton and of the slot's kind, in caller
  memory of a size the graph states; one binding serves every instance.
- **Instances:** caller memory of a size the binding states: a 16-byte
  cell per parameter and per node (its effective weight, a word its
  kind keeps, its playhead in binary64), then a cursor per clip node.
  Nothing points into the instance, so a `memcpy` copies it. Parameters
  are set by index with their declared type: ints and floats clamped to
  their bounds, quaternions normalized, anything not finite refused.
- **Saved state:** a 24-byte header (magic, the graph's identity, the
  size) and the cells in little-endian order, then every state
  machine's block word by word, without pointers; restoring checks it as
  external bytes (every value as setting it would keep it, a quaternion
  unit within 1e-5, weights finite and not negative, playheads finite, a
  select's word below its inputs; a machine's current state in range,
  its running transition one into that state, clocks finite and not
  negative, weights 0 to 1, timers not negative) and starts the cursors
  over, which cache nothing a pose depends on.

- **Update:** weights are shared from the output down, in reverse node
  order: a normalized blend by its weights over their total (or over
  1e-5 when the total is smaller), a direct blend by its weights clamped
  to 1, a mask's inside by its largest bone weight and its outside by
  the largest remainder, an additive node's layers by their weights,
  a select's chosen input whole. A node fed by several parents sums
  their shares and takes its time factor (the product of the time
  scales above it) from the first. Every clip node's playhead moves by
  the step, its speed and its factor, wrapped into a repeating clip or
  clamped; a seek whose trigger is set moves the playheads below it to
  its time and consumes the trigger. Clip nodes weighed past 1e-5 list
  the events they pass, in the clip API's windows (forward `[from, to)`,
  backward `(to, from]`), across any number of wraps, counting whole
  loops past the buffer. Root motion is the clip nodes' moves blended by
  root weights, which differ from weights at masks (the weight at the
  root motion bone, which the binding fixes) and additive layers
  (none); a clip without root motion adds the move of no motion.

- **Evaluation:** a scratch per thread holds a pose per slot (made once
  for the binding it last served), one node's layers and two bone weight
  rows. Each node the last update weighed writes its slot in order: a
  clip node samples its clip at its playhead through its cursor; a
  blend calls the blending function over its weighed inputs with
  normalized (or direct) weights; a mask blends its inside by the mask's
  bone weights and its outside by their remainders; an additive node
  copies its base and adds its weighed layers; a select, time scale or
  seek copies its input. A node whose weighed inputs are none gives the
  rest (or the difference of nothing). Before the first update, or with
  the output unweighed, the pose is the rest. The result equals the
  blending functions applied by hand, bit for bit.
- **Sync groups:** each update picks a group's leader, the first member
  declared one, else the member weighed most (the first of equals). The
  leader moves by its own speed and time factor; each follower starts
  from the leader's phase (its playhead over its duration) and moves as
  far through its own clip as the leader through its own, so members of
  different lengths stay in step. Only the leader's events fire unless
  the group asks for every member's; a seeking leader moves its
  followers to its new phase, firing nothing.
- **By markers:** a group whose members all repeat and share a marker
  name syncs by the markers common to all. A position is the common
  marker last passed (one at the playhead is still ahead), the next one
  and the fraction between them. Each follower is first placed silently
  where its markers carry the leader's starting names at the leader's
  starting fraction (the nearest such place), then passes as many common
  markers as the leader passed, continues (at most a loop) until its
  markers carry the leader's final names, and lands at the leader's
  final fraction; loops past two are skipped arithmetically. A follower
  whose clip starts on the other foot thus keeps in step without
  passing a loop of events.
- **Subgraphs** are a source-only node kind: at build the subgraph's
  output takes the node's place and its other nodes follow the source's
  (nested ones after them), its parameters are read through the node's
  map (each to a parameter of the same type), its clips join the graph's
  by identity and its sync groups by name. Nesting deeper than 16 (a
  graph inside itself) is refused. A graph built from subgraphs plays
  bit for bit as the same graph written out.
- **Blend spaces** are nodes whose input weights follow a position
  among points, one point per input (at most 256). The position is a
  vec2 parameter, or two numbers (each a literal or a float parameter).
  A 1D space (`maniGraphNodeBlendSpace1D`) blends the two points either
  side of x, held at the ends; given a period it wraps x and blends the
  last point with the first a period on, so a direction can turn past
  the seam. Its points are stored with their ascending order, found by
  halves at run time. A triangulated space
  (`maniGraphNodeBlendSpaceTriangles`) takes the author's triangles
  (anticlockwise, none degenerate, no two overlapping, every point a
  corner) or the points' Delaunay triangulation, made by the builder
  with exact predicates: each coordinate times 2^149 is an integer, and
  the orientation and in-circle determinants are computed on those
  integers (up to 1117 bits), so the file is the same on every platform
  even for grids, where every cell is a tie. The points are swept by x,
  then y, into a fan, each joined to the hull edges it sees strictly;
  Lawson's flips then turn every edge whose far point lies strictly
  inside the circle across it, so ties stay as the sweep made them. The
  file reserves 2n - 5 triangles (the most n points allow), unused ones
  zero. At run time the first triangle holding the position weighs its
  corners barycentrically; outside every triangle, the nearest point on
  any triangle's edge weighs that edge's ends, which is continuous with
  the inside and needs no tolerance where rounding leaves a position
  between two triangles. A banded space (`maniGraphNodeBlendSpaceBands`)
  weighs each point by Johansen's gradient bands (2009, section 6.3):
  the least, over the other points, of one minus the position's
  projection onto the band between the two, clamped at 0, then
  normalized. Polar bands measure magnitudes over the pair's mean and
  twice the signed angle, so a space of speeds and directions blends by
  speed along one direction and by angle around it; where no band
  reaches the position (gaps past half a turn), the nearest point takes
  it all. The update shares weights by the space's and the evaluation
  blends by the same binary32 weights, so a space plays bit for bit as
  a blend of those weights. The builder refuses points not finite or
  equal (`maniBuildReasonPoint`, other the point), 1D points that span
  a period, triangle points all on one line, and authored triangles that
  are clockwise, degenerate or overlapping (`maniBuildReasonSpaceTriangle`,
  other the triangle); opening checks the points, the stored order and
  each triangle's corners.
- **State machines** (`maniGraphNodeMachine`) take their states as
  inputs and output one normalized blend of them by per-state weights
  the instance keeps. A transition runs from a state, or from any state,
  to another; the candidates are the current state's and the any-state
  ones, in priority order (lower first; the builder refuses equal
  priorities among a state's candidates, and the file keeps the
  transitions sorted, so a crafted tie still plays the same everywhere),
  and the first whose conditions all hold fires. A condition compares a
  bool (equal or not), an int (any order) or a float (no equality)
  parameter with a value, reads a trigger (consumed when its transition
  fires), or asks for a time in the current state. A machine decides at
  the start of an update from what the updates before it left, as
  Unreal's does: its clocks advance by its time (a time scale above it
  counts), a fade whose time is up completes, then transitions fire. A
  cross-fade to D over T seconds keeps each state's weight at its start
  and moves D's to 1 and every other to 0 along a linear or smoothstep
  curve, so a fade interrupted goes on from the weights it reached:
  continuous, with no pose kept and no stack of transitions. A running
  transition allows interruption by none, by a lower priority value, or
  by any. Cooldowns hold a transition back for seconds after it fires;
  a transition of no duration completes at once and the machine looks
  again, at most its chain limit (1 to 16, 4 by default) per update.
  States of weight 0 neither advance nor evaluate; one entered from its
  start (the default entry, when it is entered at weight 0) has its
  playheads set to 0 as a seek does, and a nested machine so entered
  starts at its entry; one still weighted keeps its time, and "keep"
  entries always do. A machine that no parent reached in an update does
  not step, so its triggers wait. Each machine's block (current state,
  running transition, clocks, the record below, two weights per state,
  a timer per transition, a bit per condition) lies in the instance
  after the cursors and joins the saved state.
- **Leading clips.** A machine listens when a transition of it has a
  phase, finished or event condition or a match entry; its build lists,
  per state, the clip nodes beneath the state in node order and its
  nearest machine (the machine node of greatest index beneath it). After
  the clips move, a listening machine records its current state's
  leading clip (greatest effective weight, the first of equals): its
  normalized position before and after and the whole loops it wrapped
  (taken from where the playhead landed, so a crossing at a loop's end
  counts in one update only), the loops moved since the state was
  entered, whether the state finished, and which event conditions a
  clip beneath the state heard (listed events, so a clip below the
  event threshold is not heard). These conditions read the update
  before, as Unreal's rules read the players' last tick. A phase below
  1 holds when the position passed it (reaching counts, leaving does
  not) at any loop; 1 or more when the loops since entry passed it,
  once. Finished: the nearest machine in a final state (the source's
  `finals`) with no transition running, or with none, the leading clip
  not repeating and at the end it moves toward. A match entry seeks the
  state entered at weight 0 to the leading clip's position, each clip
  at that fraction of its own duration. Event names are kept as their
  64-bit hashes, as clips keep them.
- **One-shots** (`maniGraphNodeOneShot`) take a base and a shot. Like a
  machine, a one-shot that a parent reached decides at the start of an
  update: its fade clock advances (a fade in settles to playing at 1, a
  fade out to idle at 0), then its triggers are read and consumed, the
  first set of abort, fire and fade-out acting. Abort stops the shot at
  once; fire starts the shot over (a seek to 0 below it) and fades it in
  from the weight it has, so firing again never pops; fade-out fades it
  out from its weight. Without a trigger, a shot whose leading clip (as
  a machine's, beneath the shot), not repeating, has no more clip time
  left than the fade-out's seconds fades out, as Unreal's montages
  begin their blend-out by position. The shot's weight w, times an
  optional mask's per-bone weights, shares and blends as a mask node of
  those weights does: bit for bit the same pose. A seek above a
  one-shot stops its shot. Its block (phase, fade clock, fade-start
  weight, weight, clip seconds left) lies with the machines' and joins
  the saved state, which refuses a phase out of range and a weight its
  phase does not allow.
- **Inertialization** (`maniGraphNodeInertialize`, Bollo's GDC 2018
  and SIGGRAPH 2017 supplemental) takes one input and eases out its
  jumps. A machine transition whose fade is `inertial` switches at once
  (as a fade of 0) and requests its duration of the inertialize node on
  its first parents' path; the node's optional trigger requests its
  own duration scalar; the least request of an update is taken. The
  builder refuses a machine with an inertial transition that the output
  reaches without passing an inertialize node (Unreal can only log
  this at runtime). Poses exist only in evaluation, so the node
  captures on its first evaluation after a request: per bone,
  translation and scale by the direction and length of the offset from
  its last output to the input, rotation by the axis and angle of the
  turn on the shorter arc, the earlier output projected on that
  direction (a rotation's twist about the axis) for the speed. The
  speed is used only when its last two outputs came from consecutive
  updates (a skipped evaluation gives none; a second evaluation in one
  update replaces the last, not the one before), clamped to at most 0
  (moving away does not overshoot), and the duration cut to -5 x0 / v0
  when sooner. The offset decays by the quintic of zero jerk at its
  end, evaluated in the fraction of the duration; its clock is its own
  time, the update's delta times the factor it is reached with, so the
  first output after a request is one step in. A seek above the node
  clears it. Its block (clocks, the pending request, serials, the last
  two outputs and the capture per bone) joins the saved state, which
  refuses non-finite words, negative seconds and directions not of
  unit length.
- **Text form:** `maniWriteGraphJson` writes a source as JSON after
  building it, so only a source that builds is written; the builder
  keeps the text as it keeps files. The top object carries `format`
  ("maul-anim graph") and `version` (1), then `parameters` (name, type,
  default, and bounds when bounded), `clips` (slot names), `syncGroups`
  (name, `allEvents`), `nodes` and `output`. Nodes name their kind and
  list their inputs by place in `nodes`; parameters, clip slots, sync
  groups and the bones of mask chains are named, never indexed, so a
  file survives a reimport that changes identities or bone order. A
  number a node takes is a number or `{"parameter": name}`. A subgraph
  node carries its graph nested, with its parameters mapped by name.
  Each list item takes one line, so a change to one item is a one-line
  diff. Numbers are the shortest decimal that reads back to the same
  binary32, in ECMAScript's layout, found with integers only (`-0`
  kept), so one source has one text on every platform.
- **Reading the text:** `maniBuildGraphJson` takes the text, the
  skeleton and the host's clips with their slot names, and builds the
  file as `maniBuildGraph` does. Members are strict (one not taken, or
  given twice, is refused); those whose off value is natural (false,
  null, 0, an empty list, speed 1, loop "clip", the identity quaternion)
  may be left out. Names resolve through tables sorted by hash, so a
  hostile text costs n log n. A text that is not JSON is refused as
  `Json`, one that is not the form as `Form`, each with the byte offset
  of the fault and its kind; the text a source writes reads back to the
  bytes the source builds.

## Consequences

A graph costs nothing to load beyond the checks, and its names and
masks never reach the runtime. New node kinds (blend spaces, state
machines, inertial transitions) add kinds and data words, not
sections, so the format grows without breaking the readers of earlier
kinds. Nodes no output reaches are refused rather than kept, so a
file holds only what plays.

The graph adds little to the work it orders: on a 53-bone skeleton, a
graph of two clips and a blend takes 5.6% more instructions a step than
sampling the two clips through cursors and blending them by hand
(30,850 against 29,203), within the 10% bound kept since; the update's
weights, playheads, events and root motion and the evaluation's scratch
checks and final copy are the difference. A graph of ten nodes, one of
each kind, takes 75,128 instructions a step, the cost of its samples,
blends and copies. Nothing at runtime allocates: binding, instance and
scratch are the caller's memory, sized by the functions that name them.

A blend space's weights are cheap at the usual sizes: an update of 25
clips under a 1D space takes 7,029 instructions, under triangles 13,334,
under cartesian bands 28,558 and polar bands 43,501 (a plain blend of
the same clips, all weighted, 13,155). Bands grow with the square of
the points, every pair's band being the method: 2.0 million
instructions at 256 points, 3.6 million polar, where each point's
direction is taken once and a pair's turn is the difference of two.
Lines and triangles grow linearly. The builder's exact triangulation of
a 16 by 16 grid, every cell a tie, takes 15 ms.

A state machine costs about its weighted states: a clip node no parent
reaches is not moved (it could not move: no time factor, no weight),
outside sync groups, whose followers keep pace with their leader. A
machine of 16 states, one clip each, fading a fifth of the time, takes
4,289 instructions an update, 5,433 when it listens to its leading
clips (a plain blend of the same clips, 5,199); at 256 states 51,113
and 62,002, the machine's own share about 190 instructions a state. A
one-shot over a clip takes 1,404 (a blend of the two clips, 1,167).

Every check, choice and refusal is tested against a change made to its
line: of 203 such changes to the graph's sources, the JSON forms and
the binary32 writer, every one is caught but seven that cannot change a
result (a zero weight's layer, which adds nothing and never picks a
hemisphere; a scratch kept for another binding of the same graph; a
range check another check already implies; flags any later check
refuses at the same byte; names equal in the family hash, which no test
can find). Of 81 changes to the blend spaces, the exact predicates and
the triangulation, every one is caught but five that cannot change a
result (an end of a line met exactly, a position on a triangle's edge,
the bands' fallback no position reaches, and two forms of a bounding
box test the exact edge test repeats). Of 102 changes to the state
machines and the one-shots, every one is caught but two that
cannot change a result (whole wraps rounded from a difference that is
already whole, and a greatest index the search meets first anyway).
Opening, saved state and the JSON reader are fuzzed.
