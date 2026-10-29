# Graphs

A graph chooses and mixes a character's animation from parameters the host sets each frame: clips blended, masked and layered, switched by state machines, eased by inertialization, adjusted by IK and springs. One graph file plays on many characters at once, on several threads, and each character's state can be saved and restored for rollback. This page explains the shape, the lifecycle of a frame and the reasoning behind the node kinds.

## Contents

- [Shape](#shape)
- [Build, bind, instance](#build-bind-instance)
- [A frame: update, then evaluate](#a-frame-update-then-evaluate)
- [Sync groups](#sync-groups)
- [Blend spaces](#blend-spaces)
- [State machines and one-shots](#state-machines-and-one-shots)
- [Inertialization and dead blending](#inertialization-and-dead-blending)
- [Other node kinds](#other-node-kinds)
- [Subgraphs and the JSON form](#subgraphs-and-the-json-form)
- [Introspection and subsets](#introspection-and-subsets)
- [Traps](#traps)

## Shape

A graph is a DAG of nodes. A node may feed several parents; it is then played and evaluated once, so one playhead serves every reader. The builder ([`src/graph_build.c`](../../src/graph_build.c)) stores nodes in topological order, depth first from the output, each after its inputs, the output last; `maniGraph_GetNodeSourcePlace` maps a stored node back to its place in the source or JSON text, for tools.

Each node's pose has a slot, taken before its inputs' slots are freed and reused once its last reader has run, so evaluation needs a pose per slot, a count the file states, rather than one per node.

The builder marks each node as giving poses or differences and refuses a graph that blends one into the other, adds a pose as a layer, adds onto differences, or outputs differences. Nodes no output reaches are refused rather than kept, so a file holds only what plays.

Parameters are bool, int, float, vec2, vec3, quat (normalized when set), trigger and pose, each with a default and, for int and float, optional bounds. A number a node takes is a literal or a float parameter.

## Build, bind, instance

- **Build** (offline): `maniBuildGraph` from a source struct, or `maniBuildGraphJson` from text, against one opened skeleton and the host's opened clips, databases, learned models, spring sets and maps, each with a slot name. Masks become per-bone weights; every node is checked against the skeleton. The file keeps the identity of each asset, not the asset.
- **Open:** `maniOpenGraph` checks every record as hostile input ([file-formats.md](file-formats.md)): kinds, counts, every input before its node and in another slot, data in range, what each node gives against what its inputs make, and each node's recorded time factor against its readers' (each edge alone, so opening needs no memory). Names and masks never reach the runtime.
- **Bind:** `maniBindGraph` matches each slot to the host's asset of the recorded identity, in caller memory. One binding serves every instance of the graph.
- **Instance:** `maniInitGraphInstance` makes a character's state in caller memory ([memory-and-threads.md](memory-and-threads.md#working-memory-is-the-callers)): a cell per parameter and node, a cursor per clip node, each stateful node's block. Parameters are set by index with their declared type; ints and floats are clamped to their bounds, anything not finite is refused.

New node kinds add kinds and data words, not sections, so the format grows without breaking readers of earlier kinds.

## A frame: update, then evaluate

`maniGraphInstance_Update` advances time; `maniGraphInstance_Evaluate` writes the pose. They are separate because the update is cheap and decides everything the host needs for game logic (events, root motion, machine states), while the evaluation is the expensive part and may run on another thread, at another rate, or not at all for an off-screen character.

The **update** ([`src/graph_update.c`](../../src/graph_update.c)):

1. Weights flow from the output down, in reverse node order: a normalized blend shares its weight by its inputs' weights over their total (over 1e-5 when the total is smaller), a direct blend by its weights clamped to 1, a mask's two inputs by the largest bone weight and the largest remainder, an additive node's layers by their weights, a select's chosen input whole. A node fed by several parents sums their shares.
2. Every clip node's playhead moves by the step times its speed and the product of the time scales above it, wrapping a repeating clip and clamping another. A step that is not finite moves nothing. A seek whose trigger is set moves the playheads below it and consumes the trigger.
3. Clip nodes weighed above 1e-5 list the events they pass, forward `[from, to)` or backward `(to, from]`, across any number of wraps.
4. Root motion is the clip nodes' moves blended by root weights. Root weights differ from weights at masks (the weight at the root motion bone) and additive layers (none), so a masked upper-body clip does not move the character.

The **evaluation** ([`src/graph_evaluate.c`](../../src/graph_evaluate.c)) visits every node the last update weighed, in order: a clip node samples its clip through its cursor, a blend calls `maniSkeleton_BlendPoses` over its weighed inputs, a mask blends by the mask's bone weights and their remainders, an additive node copies its base and adds its layers, a select, time scale or seek copies its input. A node whose weighed inputs are none gives the rest. The result equals the blending functions applied by hand, bit for bit, which the tests check node by node.

The graph adds little to the work it orders: a frame of two clips and a blend takes 5.1% more instructions than sampling and blending them by hand ([size-and-performance.md](size-and-performance.md#graph-overhead), `graph.frame` against `graph.byhand`, [#62](https://github.com/Rawframe-Project/maul-anim/pull/62)), the difference being the update's weights, events and root motion. The clip nodes' step lives in its own unit, and its calls from the update add 0.21% to a frame.

## Sync groups

Clips of different lengths (a walk and a run) must keep their feet in step while blended. Each update picks a group's leader ([`src/graph_clip_step.c`](../../src/graph_clip_step.c), which moves every clip node): the first member declared one, else the member weighed most. The leader moves by its own speed; each follower starts from the leader's phase and moves as far through its own clip as the leader through its own. Only the leader's events fire unless the group asks for every member's.

A group whose members all repeat and share marker names syncs by markers instead: a position is the last common marker passed, the next one and the fraction between them. A follower is placed where its markers carry the leader's names at the leader's fraction, passes as many common markers as the leader, and lands at the leader's final fraction, so a clip that starts on the other foot stays in step without passing a loop of events.

## Blend spaces

Blend spaces weigh their inputs by a position among points, one point per input (at most 256); the position is a vec2 parameter or two numbers.

- **1D** blends the two points either side of x, held at the ends; with a period it wraps, so a direction can turn past the seam.
- **Triangles** weighs the corners of the first triangle holding the position barycentrically. Outside every triangle, the nearest point on any edge weighs that edge's ends, which is continuous with the inside and needs no tolerance where rounding leaves a position between two triangles. Without authored triangles the builder makes the Delaunay triangulation with exact predicates: each coordinate times 2^149 is an integer, and the orientation and in-circle determinants are computed on those integers ([`src/exact.c`](../../src/exact.c), [`src/triangulate.c`](../../src/triangulate.c)), so the file is the same on every platform even for a grid, where every cell is a tie. Ties stay as the sweep made them; Lawson's flips turn only edges whose far point lies strictly inside.
- **Bands** weighs each point by Johansen's gradient bands: the least, over the other points, of one minus the position's projection onto the band between the two, then normalized. Polar bands measure magnitudes and angles, so a space of speeds and directions blends by speed along one direction and by angle around it. Bands cost the square of the points; lines and triangles cost linearly.

The update and the evaluation use the same binary32 weights, so a space plays bit for bit as a blend of those weights.

## State machines and one-shots

A machine ([`src/graph_machine.c`](../../src/graph_machine.c)) takes its states as inputs and outputs a normalized blend of them by per-state weights the instance keeps. It decides at the start of an update from what the updates before it left, as Unreal's does: its clocks advance, a fade whose time is up completes, then the first candidate transition (the current state's and the any-state ones, by priority) whose conditions all hold fires. Conditions compare parameters, read triggers (consumed on firing), ask for time in the state, or listen to the state's leading clip: a phase passed, a finished state, an event heard. Those read the update before, as Unreal's rules read the players' last tick.

A cross-fade keeps each state's weight where it started and moves the destination's to 1 and every other to 0 along a linear or smoothstep curve. An interrupted fade goes on from the weights it reached: continuous, with no pose stored and no stack of transitions. Cooldowns hold a transition back; a transition of no duration completes at once and the machine looks again, at most its chain limit per update. A state at weight 0 neither advances nor evaluates, which is why a machine of 256 states costs about its weighted states.

Priorities among one state's candidates must differ, and the file keeps transitions sorted, so even a crafted tie plays the same everywhere.

A one-shot plays a shot over a base: fire starts the shot over and fades it in from the weight it has (firing again never pops), abort stops it, fade-out fades it, and a shot whose leading clip nears its end fades out by itself, as Unreal's montages blend out by position. Its weight blends as a mask node of those weights would, bit for bit.

## Inertialization and dead blending

A transition marked inertial switches at once and asks the inertialize node above it to ease the jump; so do motion matching and learned nodes when they jump. The builder refuses a graph where such a source reaches the output without passing an inertialize node, which Unreal can only log at run time.

Poses exist only during evaluation, so the node captures on its first evaluation after a request: per bone, the offset from its last output to the new input (direction and length for translation and scale, axis and angle for rotation) and its speed from its last two outputs, when they came from consecutive updates. The offset then decays by Bollo's quintic of zero jerk at its end. Dead blending, the node's other method, extrapolates each bone's last output by its velocity with a decaying rate and blends into the input by smoothstep. Each node keeps its last two outputs, so a frame with nothing to ease costs one copy.

## Other node kinds

| Kind | Page |
|---|---|
| Motion matching, learned | [motion-matching.md](motion-matching.md) |
| Two-bone, aim, CCD, FABRIK, full-body XPBD and SDLS | [ik.md](ik.md) |
| Spring, pose input | [physics-seam.md](physics-seam.md) |
| Mirror; clip slots with a retargeting map | [retargeting-and-mirroring.md](retargeting-and-mirroring.md) |

The **turn limit** node turns each bone's local rotation from its last output toward its input by at most a speed times the seconds since its last evaluation, a bone's budget divided by its mask weight. It bounds any jump above the sources it distrusts: an IK pole crossing its goal line, a cut transition, a teleported goal. Positions are not limited (a root's travel is root motion's), and limits are local, so a parent's turn never spends its children's budget. IK nodes themselves keep no history; the limit is a node of its own so they stay functions of their input.

## Subgraphs and the JSON form

A subgraph is a source-only node: at build its output takes the node's place, its other nodes follow, its parameters are read through the node's map, and its assets join the graph's by identity. A subgraph's lists are held to the rules of the root's, each of its sync groups has a name to join by, and a slot or sync group past the subgraph's own list is refused at its node's place, never read as the root's. In the same way a slot or sync group of the root's nodes past the root's own lists is refused at its node's place, never read as a subgraph's. A graph built from subgraphs plays bit for bit as the same graph written out. Nesting deeper than 16 is refused.

`maniWriteGraphJson` writes a source as text after building it, so only a source that builds is written. Parameters, slots, sync groups and bones are named, never indexed, so the text survives a reimport that changes identities or bone order. Each list item takes one line, so a change to one item is a one-line diff, and numbers are the shortest decimal that reads back to the same binary32, found with integers only, so one source has one text on every platform. `maniBuildGraphJson` reads it strictly (an unknown or repeated member is refused), names resolve through sorted hash tables so a hostile text costs n log n, and a refusal gives the byte offset and its kind. The retargeting and mirror maps share the reader and writer ([retargeting-and-mirroring.md](retargeting-and-mirroring.md#the-json-form)).

## Introspection and subsets

`maniGraphInstance_InspectNode` reads what the update already keeps (a node's share of the output, a playhead, a machine's state and transition, a fade's progress), so a debugger costs nothing when unused and the runtime keeps no names.

`maniPlanGraphSubset` plans an evaluation of some bones only: the listed bones, their ancestors (so the model pass is right), and closed under what nodes read to write them (a mirrored bone's partner, an IK chain, a spring set's joints and colliders, a full-body part). `maniGraphInstance_EvaluateSubset` then blends only the groups holding those bones and samples only their channels, writing them bit for bit as a full evaluation and leaving the other bones as they were. A server's hit-test bones or a distant crowd cost by the groups they touch.

## Saved state

`maniGraphInstance_Save` writes a header (the state prefix described in [overview.md](overview.md), then the graph's identity and the size) and the cells, machines' and stateful nodes' blocks and pose parameters as little-endian words, without pointers. `maniGraphInstance_Restore` checks every value as external bytes and starts the cursors over, which cache nothing a pose depends on.

## Traps

- Machines and one-shots decide at the start of an update from the previous one's results, so a condition on a clip event takes effect one update after the event.
- A machine no parent reached in an update does not step, and its triggers wait.
- The learned node's cache of hidden values lives in the instance outside the saved state; it is valid only when the state's bytes equal the kept state's (a byte compare: `-0` and NaN differ), and only the update writes it.
- A seek above a stateful node (inertialize, spring, turn limit, one-shot) clears that node's history. A spring or turn limit node's own reset trigger clears it without moving any playhead, for cuts the host makes (a teleport, a camera cut).
