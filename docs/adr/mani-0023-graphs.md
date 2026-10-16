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
  size) and the cells in little-endian order, without pointers;
  restoring checks it as external bytes (every value as setting it
  would keep it, a quaternion unit within 1e-5, weights finite and not
  negative, playheads finite, a select's word below its inputs) and
  starts the cursors over, which cache nothing a pose depends on.

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

## Consequences

A graph costs nothing to load beyond the checks, and its names and
masks never reach the runtime. New node kinds (blend spaces, state
machines, inertial transitions) add kinds and data words, not
sections, so the format grows without breaking the readers of earlier
kinds. Nodes no output reaches are refused rather than kept, so a
file holds only what plays.
