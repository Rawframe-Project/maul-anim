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

Binding, instances, update and evaluation follow in this record as
they land (item 8).

## Consequences

A graph costs nothing to load beyond the checks, and its names and
masks never reach the runtime. New node kinds (blend spaces, state
machines, inertial transitions) add kinds and data words, not
sections, so the format grows without breaking the readers of earlier
kinds. Nodes no output reaches are refused rather than kept, so a
file holds only what plays.
