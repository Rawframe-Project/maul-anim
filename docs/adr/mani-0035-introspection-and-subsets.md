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
  update (0 when not reached and before the first update); a clip's or
  matching node's playhead; a select's chosen input, a machine's
  current state (during a transition the state it leads to; the states
  it fades out keep their shares as nodes), a one-shot's phase or a
  matching node's entry; a machine's transition under way; how far a
  machine's fade or an inertialization has come (elapsed over
  duration) or a one-shot's weight. Fields a kind has no use for read
  as `MANI_GRAPH_NONE` or 0.

## Consequences

- A debugger draws the tree from the file and colours it from the
  instance, mapping each node back to its authored place, without the
  library keeping names or bookkeeping at runtime.
- Introspection costs nothing when unused: it reads the instance.
- A graph file of format version 3 is not opened; graphs are rebuilt
  from their sources, as before the first release.
