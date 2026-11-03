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
maniResult graphCopied = maniBuilder_CopyBuilt(builder, graphFile, graphCapacity, &graphSize);
maniGraph graph;
maniFileResult graphOpened = maniOpenGraph(graphFile, graphSize, &graph);
```

The JSON form names everything (parameters, clip slots, bones), so the text survives a reimport that reorders bones, and each list item is one line, so a change reviews as a one-line diff. `maniWriteGraphJson` writes a source as text. A text that is not JSON, or not the graph form, is refused with the byte offset and what was wrong.

### Keys left out

A key the graph form marks as optional may be left out, and reads as the value below. `maniWriteGraphJson` leaves out every key that holds that value, so a node at its defaults is written with only what it needs.

| Object | Key left out reads as |
|---|---|
| Graph | `parameters`, `clips`, `syncGroups`, `databases`, `models`, `springSets`: empty |
| Parameter | `default`: false, 0, zeros, or the identity rotation for a quaternion |
| Sync group | `allEvents`: false |
| `clip` | `syncGroup`: none; `speed`: 1; `loop`: `"clip"`; `leader`: false |
| `blend` | `direct`: false |
| `mask`, and the `mask` of a `oneShot` or `turnLimit` | `chains`: none; `defaultWeight`: 0; a chain's `subtree`: false |
| `seek` | `time`: 0 |
| `blendSpace1D`, `blendSpaceTriangles`, `blendSpaceBands` | the position's `y`: 0; `period`: 0; `triangles`: a Delaunay triangulation; `polar`: false |
| `machine` | `entry`, `chainLimit`: 0; `transitions`, `finals`: none |
| Transition | `priority`: 0; `interruption`: `"none"`; `entry`: `"restart"`; `fade`: `"linear"`; `duration`, `cooldown`: 0; `conditions`: none |
| `oneShot` | `fadeOutTrigger`, `abort`: none; `curve`: `"linear"`; `fadeIn`, `fadeOut`: 0; `mask`: the whole body |
| `inertialize` | `trigger`: none; `method`: `"inertialization"`; `duration`: 0; for dead blending, `halfLifeScale` 0.3, `halfLifeMin` 0.1, `halfLifeMax` 1 |
| `match` | `trigger`: none; `query`: empty; `period`: 0.1; `jumpThreshold`, `endExclusion`, `historyTime`, `continuingBias`, `transitionCost`, `historyCount`: 0; `requiredTags`, `excludedTags`: 0; `duration`: 0 |
| `learned` | `trigger`: none; `query`: empty; `period`: 0.1; `transitionCost`: 0; `requiredTags`, `excludedTags`: 0; `duration`: 0 |
| `spring` | `position`, `rotation`, `reset`, `force`: none |
| `twoBone` | `pole`: none; `hinge`: [0, 0, 0]; `twist`: 0; `soften`, `weight`: 1 |
| `aim` | `shares`: equal; `pole`: none; `offset`, `up`: [0, 0, 0]; `twist`, `cone`: 0; `weight`: 1 |
| `ccd`, `fabrik` | `step` (`ccd`): 0; `pole` (`fabrik`), `rotation`: none; `tolerance`: 0; `rotationWeight`, `weight`: 1; `limits`: none; a cone limit's `twist`: [-π, π] |
| `bodyXpbd`, `bodySdls` | `topMode`: `"pinned"`; `bones`: none; `stretch`: false; `tolerance`: 0; `weight`: 1 |
| Effector | `rotation`: none; `positionWeight`: 1; `rotationWeight`: 1 with a rotation; `strength`: 1; `pullChain`: 0 |
| Bone setting | `limit`: `{"kind": "none"}`; `preferred`: [0, 0, 0]; `rotationStiffness`, `positionStiffness`: 0 |
| `turnLimit` | `reset`: none; `mask`: every bone |
| `subgraph` | `parameters`: none, for a subgraph without parameters |

## Playing a graph

An opened graph is bound to the assets it names once, and each character gets an instance: its parameters, playheads and state, in the program's memory. Copying an instance's bytes copies the character's state, and `maniGraphInstance_Save` writes it to bytes for saving or rollback; called with no buffer, it reports the size it needs.

```c
const maniClip* clips[1] = {&clip};
maniGraphAssets assets = {.skeleton = &skeleton, .clips = clips, .clipCount = 1};
maniGraphBinding* binding = NULL;
maniResult bound =
    maniBindGraph(&graph, &assets, bindingMemory, maniGraph_GetBindingSize(&graph), &binding, NULL);
maniGraphInstance* instance = NULL;
maniResult started =
    maniInitGraphInstance(binding, instanceMemory, maniGraphBinding_GetInstanceSize(binding), &instance);
```

Each frame: set parameters, update by the frame's time (which fires transitions and events and gives the root motion), then evaluate the pose.

```c
uint32_t go = maniGraph_FindParameter(&graph, "go", 2);
maniResult fired =
    maniGraphInstance_SetParameter(instance, go, maniGraphParameterTrigger, (maniGraphValue){.b = 1});
maniGraphEvent events[8];
uint32_t eventCount = 0;
maniRootMove move;
maniResult updated = maniGraphInstance_Update(instance, 1.0f / 60.0f, events, 8, &eventCount, &move);
maniResult evaluated = maniGraphInstance_Evaluate(instance, scratch, maniGraphBinding_GetScratchSize(binding), pose);
```

The update is cheap and gives what game logic needs (events, root motion, machine states); the evaluation writes the pose and may run on another thread or be skipped for a character off screen. One scratch per thread serves every instance.

Both return `mani_errorFormat` when a learned node's model gives a value past the float range, though every weight in it is finite (the update also when a root velocity's move over a long step passes it): the call still completes, and what it writes (events, root motion, the pose) is valid and finite. The failing node holds its last finite state, adds no root motion or events and gives the neutral pose, and `maniGraphInstance_InspectNode` names it with `failed`. The model is untrusted data, so this counts no misuse; a program can keep playing, replace the model, or report it.

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

`maniGraphInstance_InspectNode` reads what each node did in the last update (its share of the output, a playhead, a machine's state and how far its transition has come), for a debugger, without the runtime keeping any names; `maniGraph_GetNodeSourcePlace` maps a node back to its place in the JSON. A subset plan (`maniPlanGraphSubset`, `maniGraphInstance_EvaluateSubset`) evaluates only some bones, bit for bit as a full evaluation, for hit tests on a server or a distant crowd.

## Not obvious

- Machines and one-shots decide at the start of an update from what the previous update left, so a transition on a clip's event fires on the update after the event.
- A node fed by several parents plays once; its playhead serves them all.
- A time scale that reads a parameter needs the parameter's bounds. The product of the time scales on any path from the output is at most `MANI_MAX_GRAPH_TIME_FACTOR` (1e6) in size, a parameter's counted at the larger size of its bounds; the builder refuses a graph past it with `mani_errorLimit`, and the opener a file past it with `mani_errorFormat`. Scales on parallel branches do not multiply.
- Sync groups keep clips of different lengths in step by phase, or by marker names when every member has them, so a run that starts on the other foot stays in step with a walk.
- A pose parameter (`maniGraphInstance_SetPose`) copies the pose in; the program may reuse its pose afterwards.
