# Animation graphs

A graph chooses and mixes a character's animation from named parameters the program sets each frame. It is a tree of nodes: clips, blends, masks, additive layers, blend spaces, state machines, one-shots, inertialization, IK, springs, motion matching and more. It is written as JSON or built from a source struct, refers to its clips and other assets by name, and is checked node by node when built.

## Building a graph

```c
static const char graphText[] =
    "{\"format\": \"maul-anim graph\", \"version\": 1,"
    " \"parameters\": [{\"name\": \"go\", \"type\": \"trigger\"}],"
    " \"clips\": [\"turn\"],"
    " \"nodes\": [{\"kind\": \"machine\", \"inputs\": [1, 2], \"transitions\":"
    " [{\"from\": 0, \"to\": 1, \"duration\": 0.25, \"conditions\": [{\"trigger\": \"go\"}]}]},"
    " {\"kind\": \"clip\", \"clip\": \"turn\"}, {\"kind\": \"clip\", \"clip\": \"turn\"}],"
    " \"output\": 0}";
const maniGraphClipSource graphClips[1] = {{&clip, "turn", 4, NULL}};
maniGraphNamedAssets named = {&skeleton, graphClips, NULL, NULL, NULL, 1, 0, 0, 0, NULL};
maniBuildResult graphBuilt = maniBuildGraphJson(builder, graphText, sizeof graphText - 1, &named);
size_t graphSize = 0;
maniResult graphCopied = maniCopyBuilt(builder, graphFile, graphCapacity, &graphSize);
maniGraph graph;
maniFileResult graphOpened = maniOpenGraph(graphFile, graphSize, &graph);
```

The JSON form names everything (parameters, clip slots, bones), so the text survives a reimport that reorders bones, and each list item is one line, so a change reviews as a one-line diff. `maniWriteGraphJson` writes a source as text. A text that is not JSON, or not the graph form, is refused with the byte offset and what was wrong.

## Playing a graph

An opened graph is bound to the assets it names once, and each character gets an instance: its parameters, playheads and state, in the program's memory. Copying an instance's bytes copies the character's state, and `maniSaveGraphInstance` writes it to bytes for saving or rollback.

```c
const maniClip* clips[1] = {&clip};
maniGraphAssets assets = {.skeleton = &skeleton, .clips = clips, .clipCount = 1};
maniGraphBinding* binding = NULL;
maniResult bound =
    maniBindGraph(&graph, &assets, bindingMemory, maniGraphBindingSize(&graph), &binding, NULL);
maniGraphInstance* instance = NULL;
maniResult started =
    maniInitGraphInstance(binding, instanceMemory, maniGraphInstanceSize(binding), &instance);
```

Each frame: set parameters, update by the frame's time (which fires transitions and events and gives the root motion), then evaluate the pose.

```c
uint32_t go = maniGraphFindParameter(&graph, "go", 2);
maniResult fired =
    maniSetGraphParameter(instance, go, maniGraphParameterTrigger, (maniGraphValue){.b = 1});
maniGraphEvent events[8];
uint32_t eventCount = 0;
maniRootMove move;
maniResult updated = maniUpdateGraph(instance, 1.0f / 60.0f, events, 8, &eventCount, &move);
maniResult evaluated = maniEvaluateGraph(instance, scratch, maniGraphScratchSize(binding), pose);
```

The update is cheap and gives what game logic needs (events, root motion, machine states); the evaluation writes the pose and may run on another thread or be skipped for a character off screen. One scratch per thread serves every instance.

Both return `mani_errorFormat` when a learned node's model gives a value past the float range, though every weight in it is finite (the update also when a root velocity's move over a long step passes it): the call still completes, and what it writes (events, root motion, the pose) is valid and finite. The failing node holds its last finite state, adds no root motion or events and gives the neutral pose, and `maniInspectGraphNode` names it with `failed`. The model is untrusted data, so this counts no misuse; a program can keep playing, replace the model, or report it.

## Node kinds

| Kind | What it does |
|---|---|
| clip | plays a clip slot, with a speed, a loop override and an optional sync group |
| blend, mask, additive | mixes its inputs by weights, per-bone weights, or as additive layers |
| select, time scale, seek | picks an input by a parameter, scales time below it, jumps playheads on a trigger |
| blend space (1D, triangles, bands) | weighs its inputs by a position among points: a speed, a direction, a 2D stick |
| machine | states with transitions on conditions (parameters, triggers, time in state, a clip's phase, end or event) and cross-fades |
| one-shot | plays a shot (an attack, an emote) over a base, fading in and out |
| inertialize | eases jumps from inertial transitions and matching (inertialization or dead blending) |
| match, learned | motion matching from a database, or a learned model ([Motion matching](motion-matching.md)) |
| two-bone, aim, CCD, FABRIK, full-body | IK on the pose ([Inverse kinematics and physics](ik-and-physics.md)) |
| spring, pose | spring bones; a pose the program sets, such as a ragdoll's |
| mirror, turn limit | mirrors the pose; bounds how fast bones may turn |
| subgraph | a graph inside a graph, inlined when built |

## Debugging and level of detail

`maniInspectGraphNode` reads what each node did in the last update (its share of the output, a playhead, a machine's state and how far its transition has come), for a debugger, without the runtime keeping any names; `maniGraphNodeSourcePlace` maps a node back to its place in the JSON. A subset plan (`maniPlanGraphSubset`, `maniEvaluateGraphSubset`) evaluates only some bones, bit for bit as a full evaluation, for hit tests on a server or a distant crowd.

## Not obvious

- Machines and one-shots decide at the start of an update from what the previous update left, so a transition on a clip's event fires on the update after the event.
- A node fed by several parents plays once; its playhead serves them all.
- A time scale that reads a parameter needs the parameter's bounds. The product of the time scales on any path from the output is at most `MANI_GRAPH_MAX_TIME_FACTOR` (1e6) in size, a parameter's counted at the larger size of its bounds; the builder and the opener refuse a graph past it. Scales on parallel branches do not multiply.
- Sync groups keep clips of different lengths in step by phase, or by marker names when every member has them, so a run that starts on the other foot stays in step with a walk.
- A pose parameter (`maniSetGraphPose`) copies the pose in; the program may reuse its pose afterwards.
