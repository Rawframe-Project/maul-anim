# mani-0035. Introspection and subsets

Status: Accepted

## Context

A host building a debugger for its animation graphs needs to see what a
running graph does: which nodes contribute to the pose and by how much,
where each clip's playhead is, which state each machine is in and how
far its transition has come. Unity answers per layer with its state and
transition infos; Unreal has each node gather its final weight and a
line of text; Morpheme made live debugging of a running network its
selling point. A host also often needs only some bones: a server's
hit-test bones, a distant crowd's level of detail.

## Decision

- **Structure:** an opened graph answers each node's kind
  (`maniGraphNodeKindOf`), its inputs (`maniGraphNodeInputCount`,
  `maniGraphNodeInput`), the output node (`maniGraphOutputNode`) and
  the node's place in the source it was built from
  (`maniGraphNodeSourcePlace`): the builder orders nodes after their
  inputs, so a file's node may sit elsewhere than its source's or JSON
  text's place. The place is kept in each node record (graph format
  version 4), checked below the node count on open.
- **Inspection:** `maniInspectGraphNode` fills a `maniGraphNodeView`
  from the state the update already keeps, adding nothing to the
  update: the kind; the node's share of the output after the last
  update (0 when not reached and before the first update; under a mask
  the largest of the mask's weights over the bones, or of one minus
  them for its second input; an additive layer its layer's weight times
  the node's; the shares of several paths summed, so it may pass 1),
  read between updates; a clip's or
  matching node's playhead; a select's chosen input, a machine's
  current state (during a transition the state it leads to; the states
  it fades out keep their shares as nodes), a one-shot's phase or a
  matching node's entry; a machine's transition under way; how far a
  machine's fade or an inertialization has come (elapsed over
  duration) or a one-shot's weight. Fields a kind has no use for read
  as `MANI_GRAPH_NONE` or 0.
- **Subsets:** `maniPlanGraphSubset` turns a strictly increasing bone
  list into a plan in caller memory (`maniGraphSubsetSize` bytes): the
  bones to compute and write (`maniGraphSubsetBones`: the listed ones
  and their ancestors, so the model pass of the listed bones is right,
  closed to a fixed point under what the nodes read to write them:
  under a mirror map each bone's partner; for an IK node writing a
  planned bone its chain; for a spring node its set's joints, tails,
  colliders' bones and centers; for a full-body node its paths from its
  top to its effectors), the pose groups holding them, and whether each
  node runs (a node of those kinds writing no planned bone passes its
  input on).
  `maniEvaluateGraphSubset` evaluates as `maniEvaluateGraph` but blends
  and adds only the plan's groups and samples clips only at the plan's
  bones (by random access, the cursor's bits), writing the plan's bones
  bit for bit as a full evaluation does and leaving every other bone of
  the pose as it was. Retargeted clips are transferred whole (their
  plans read source bones of another skeleton). The update is
  unchanged.

## Consequences

- A debugger draws the tree from the file and colours it from the
  instance, mapping each node back to its authored place, without the
  library keeping names or bookkeeping at runtime.
- Introspection costs nothing when unused: it reads the instance.
- A spring set none of whose joints is planned is not stepped, and an
  inertialization keeps its offsets and output outside the plan's
  groups; bones joining a later plan start from what they last had, as
  a level of detail switch does elsewhere.
- A graph file of format version 3 is not opened; graphs are rebuilt
  from their sources, as before the first release.

Timings, by instruction count, GCC 14 at -O2: a 54-bone humanoid playing
a blend of two clips under an additive layer, an update and an
evaluation a step. A whole evaluation takes 53,198 instructions; a
plan of the head, the hands and the feet (22 bones with their
ancestors, in 5 of the pose's 7 groups) 38,444; a plan of the head (8
bones, one group) 17,981. The update alone takes 2,226. A plan costs
by the groups it touches, since blending and adding run a group of
eight bones at once, and sampling by the bones it lists.
