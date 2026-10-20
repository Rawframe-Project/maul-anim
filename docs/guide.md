# Maul Anim guide

This guide walks through the library in the order a program uses it:
skeletons and clips built and opened, poses sampled, blended and taken
to the model, then skinning, the animation graph, inverse kinematics,
springs and ragdolls, retargeting and mirroring, motion matching, and
the offline side that gets data in. Every function named here is
described in full in [the API reference](api.md); the design records
behind each part are in `adr/`. Every C block below is built and run by
the tests (`test/test_guide*.c`), so it cannot drift from the API.

## The ground rules

- **Units and axes.** Meters and seconds. Rotations are unit
  quaternions `{x, y, z, w}`; a bone's local transform is translation,
  rotation and scale (`maniTransform`), applied scale first.
- **Results.** A fallible call returns `maniResult`: `mani_success` (0)
  or a negative error (`mani_errorInvalid`, `mani_errorCapacity`,
  `mani_errorFormat`, `mani_errorVersion`, `mani_errorState`), and
  `maniResultName` names any of them. Opening a file returns a
  `maniFileResult` whose `offset` names the first byte found wrong;
  building returns a `maniBuildResult` whose `reason`, `item` and
  `other` say what in the source was refused.
- **Data is files.** Skeletons, clips, skins, graphs, maps, motion
  databases and the rest are built offline into files by a builder and
  opened in place at runtime: opening checks every byte, allocates
  nothing and keeps a view over your bytes, which must outlive it.
- **Memory.** The runtime allocates nothing. A pose, a graph instance
  or a solver's scratch lives in memory you give it, its size from a
  `...Size` query. Only builders and importers allocate, through the
  allocator in their def (zeroed, the C library's).
- **Versions.** A def from its `maniDefault...Def` function carries the
  version of the headers you built with (`MANI_ABI_VERSION`); a library
  of another major or minor refuses it with `mani_errorVersion`. Files
  carry their format's version and are refused the same way.
- **Threads.** The library starts none. Opened files are read-only and
  shared freely; a pose, an instance, a builder is used by one thread at
  a time. Each function's documentation says what may run at once.
- **Determinism.** The same inputs give the same bits on every platform
  and compiler the library supports, so a pose can be hashed and
  compared across machines.

## Skeletons

A skeleton is built from its bones: a name, a parent (-1 for a root)
and a rest transform. The builder orders bones so that parents come
first; `maniCopySkeletonOrder` says where each source bone went.

```c
maniBuilderDef def = maniDefaultBuilderDef();   // the C library's allocator
maniBuilder* builder = NULL;
maniResult made = maniCreateBuilder(&def, &builder);

const maniBoneSource bones[3] = {
    {"hips", 4, -1, {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"spine", 5, 0, {{0.0f, 0.2f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"head", 4, 1, {{0.0f, 0.4f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}}};
maniSkeletonSource source = {bones, 3, 0};
maniBuildResult built = maniBuildSkeleton(builder, &source);
// built.result; when refused, built.reason, built.item and built.other.

size_t size = 0;
maniResult copied = maniCopyBuilt(builder, file, capacity, &size);   // file 16-byte aligned
maniSkeleton skeleton;
maniFileResult opened = maniOpenSkeleton(file, size, &skeleton);
// opened.result; on mani_errorFormat, opened.offset is the first bad byte.
```

`maniSkeletonBoneCount`, `maniSkeletonParent`, `maniSkeletonRest` and
`maniSkeletonFindBone` read an opened skeleton; its identity
(`maniSkeletonIdentity`) is a hash of its bytes that every asset built
for it records, so a clip of another skeleton is refused, not played
wrongly.

## Clips

A clip holds bone tracks (each a translation, rotation and scale
channel of keys), float and discrete tracks, events and markers, and
root motion, sampled on a uniform grid. Keys may be at any times; the
builder resamples them at the clip's rate.

```c
static const float times[2] = {0.0f, 1.0f};
static const float turns[8] = {0.0f, 0.0f, 0.0f, 1.0f,             // at 0 s
                               0.0f, 0.38268343f, 0.0f, 0.92387953f}; // 45 degrees about y
const maniBoneTrackSource tracks[1] = {
    {1, 0, {NULL, NULL, 0, 0}, {times, turns, 2, maniInterpolationLinear}, {NULL, NULL, 0, 0}}};
maniClipSource clipSource = {.skeleton = &skeleton,
                             .boneTracks = tracks,
                             .boneTrackCount = 1,
                             .loop = maniLoopRepeat,
                             .sampleRate = 30.0f,
                             .duration = 1.0f};
maniBuildResult clipBuilt = maniBuildClip(builder, &clipSource);
size_t clipSize = 0;
maniResult clipCopied = maniCopyBuilt(builder, clipFile, clipCapacity, &clipSize);
maniClip clip;
maniFileResult clipOpened = maniOpenClip(clipFile, clipSize, &clip);
```

A channel without keys stays at the rest. `maniCompressClip` makes a
compressed clip of an opened one, within an error you choose; it is
opened and sampled by the same calls.

## Poses, sampling and blending

A pose holds a skeleton's local transforms and, once composed, its
model matrices. Its memory is yours: `maniPoseSize` bytes aligned to
`MANI_POSE_ALIGNMENT`.

```c
maniPose* pose = NULL;
maniResult ready = maniInitPose(&skeleton, poseMemory, maniPoseSize(&skeleton), &pose);
maniResult sampled = maniSampleClip(&clip, &skeleton, 0.5f, pose);   // half a second in
maniTransform spine = maniPoseLocal(pose, 1);
```

Sampling at a time is random access: any clip, any time, the same cost.
To play forward, a cursor (`maniInitClipCursor`, `maniSampleClipCursor`)
keeps the samples it decoded and gives the same bits. Blending takes
layers, each a pose, a weight and optional per-bone weights:

```c
const maniBlendLayer layers[2] = {{walking, 0.25f, NULL}, {running, 0.75f, NULL}};
maniResult blended = maniBlendPoses(&skeleton, layers, 2, 0.1f, pose);
```

A bone whose layers' weights add to less than the threshold (here 0.1)
takes the rest for the remainder. Additive layers (`maniPoseDifference`,
`maniAddPoses`) and masks of per-bone weights are described in the
reference.

## The model pass

Model matrices are composed on demand:

```c
maniResult composed = maniComputeModel(pose, &skeleton);
maniAffine head = maniPoseModel(pose, 2);   // the head in model space
```

Setting a local marks the model stale; `maniPoseModelCurrent` says
whether it is current.

## Skinning

A skin binds a mesh to some of the skeleton's bones (its joints), each
with an inverse bind matrix; a mesh primitive's influences say which
joints move each vertex and by how much.

```c
const uint32_t joints[2] = {1, 2};   // the spine and the head
const maniAffine inverseBinds[2] = {
    {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0.0f, -1.2f, 0.0f}},   // undoing each joint's rest
    {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0.0f, -1.6f, 0.0f}}};
maniSkinSource skinSource = {&skeleton, joints, inverseBinds, 2, 0};
maniBuildResult skinBuilt = maniBuildSkin(builder, &skinSource);
size_t skinSize = 0;
maniResult skinCopied = maniCopyBuilt(builder, skinFile, skinCapacity, &skinSize);
maniSkin skin;
maniFileResult skinOpened = maniOpenSkin(skinFile, skinSize, &skin);

// Two vertices: one on the spine, one shared by the spine and the head.
const uint32_t offsets[3] = {0, 1, 3};
const uint32_t vertexJoints[3] = {0, 0, 1};
const float weights[3] = {1.0f, 0.5f, 0.5f};
maniInfluenceSource influenceSource = {&skin, offsets, vertexJoints, weights, NULL, 2, 0};
maniBuildResult influencesBuilt = maniBuildInfluences(builder, &influenceSource);
size_t influencesSize = 0;
maniResult influencesCopied =
    maniCopyBuilt(builder, influencesFile, influencesCapacity, &influencesSize);
maniInfluences influences;
maniFileResult influencesOpened = maniOpenInfluences(influencesFile, influencesSize, &influences);
```

Each frame, the pose's model gives the palette, and a job moves the
vertices: linear blend skinning here, dual quaternion
(`maniSkinDualQuaternion`, its palette from `maniPaletteDualQuaternions`)
or a blend of both per vertex (`maniSkinBlended`) alike.

```c
maniAffine palette[2];
maniResult paletted = maniComputePalette(pose, &skeleton, &skin, palette, 2, NULL);
const float positions[6] = {0.1f, 1.3f, 0.0f, 0.1f, 1.5f, 0.0f};
float moved[6];
const maniSkinStream stream = {positions, moved, 12, 12, maniStreamPoint, 0};
const maniSkinJob job = {&influences, palette, NULL, 2, 0, 2, 1, &stream};
maniResult skinned = maniSkinLinear(&job);
```

Jobs over disjoint ranges of vertices may run on several threads at
once. Normals, directions and tangents are further streams of the same
job.

## The animation graph

A graph is a tree of nodes (clips, blends, blend spaces, state
machines, one-shots, inertialization, IK, springs, motion matching and
more) with named parameters a host sets each frame. It is written as
JSON or built from a source struct, and refers to its clips, databases
and other assets by name; building it checks every node.

```c
static const char graphText[] =
    "{\"format\": \"maul-anim graph\", \"version\": 1,"
    " \"parameters\": [{\"name\": \"go\", \"type\": \"trigger\"}],"
    " \"clips\": [\"turn\"],"
    " \"nodes\": [{\"kind\": \"machine\", \"inputs\": [1, 2], \"transitions\":"
    " [{\"from\": 0, \"to\": 1, \"duration\": 0.25, \"conditions\": [{\"trigger\": \"go\"}]}]},"
    " {\"kind\": \"clip\", \"clip\": \"turn\"}, {\"kind\": \"clip\", \"clip\": \"turn\"}],"
    " \"output\": 0}";
const maniGraphClipSource graphClips[1] = {{&clip, "turn", 4, 0, NULL}};
maniGraphNamedAssets named = {&skeleton, graphClips, NULL, NULL, NULL, 1, 0, 0, 0, NULL};
maniBuildResult graphBuilt = maniBuildGraphJson(builder, graphText, sizeof graphText - 1, &named);
size_t graphSize = 0;
maniResult graphCopied = maniCopyBuilt(builder, graphFile, graphCapacity, &graphSize);
maniGraph graph;
maniFileResult graphOpened = maniOpenGraph(graphFile, graphSize, &graph);
```

At runtime an opened graph is bound to the assets it names, once, and
each character gets an instance: its parameters, playheads and state,
in memory you give it (a `memcpy` of an instance is a copy, and
`maniSaveGraphInstance` writes one to bytes).

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

Each frame: set parameters, update by the frame's time (which fires
transitions and events and gives the root motion), then evaluate the
pose.

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

`maniInspectGraphNode` reads what each node did in the last update, for
a debugger; a subset plan (`maniPlanGraphSubset`) evaluates only some
bones, for hit tests or a distant crowd.

## Inverse kinematics

The solvers work on a pose's locals: two-bone chains, aims and
look-ats, CCD and FABRIK chains with joint limits, and full-body IK by
position-based dynamics or damped least squares. Each is also a graph
node, its goal from a parameter.

```c
// The hips, spine and head as a chain bending about the spine's z axis,
// its end onto a goal in model space.
const maniTwoBoneSolve reach = {.start = 0,
                                .middle = 1,
                                .end = 2,
                                .goal = {0.3f, 1.5f, 0.0f},
                                .hinge = {0.0f, 0.0f, 1.0f},
                                .soften = 1.0f,
                                .weight = 1.0f};
bool reached = false;
maniResult solved = maniSolveTwoBone(pose, &skeleton, &reach, &reached);
```

`maniSolveAim` turns a bone (or spreads the turn over a chain) to look
at a goal; `maniSolveCcd` and `maniSolveFabrik` bend longer chains;
`maniSolveBodyXpbd` and `maniSolveBodySdls` move a whole body toward
several effectors at once, from a description built once.

## Retargeting and mirroring

A retargeting map carries poses from one skeleton to another: bones
paired by name or explicitly, chains of different lengths, the pelvis
scaled by the rigs' heights, each target bone keeping its own lengths.

```c
maniRetargetMapSource retargetSource = {.source = &skeleton, .target = &tall, .matchNames = true};
maniBuildResult mapBuilt = maniBuildRetargetMap(builder, &retargetSource);
size_t mapSize = 0;
maniResult mapCopied = maniCopyBuilt(builder, mapFile, mapCapacity, &mapSize);
maniRetargetMap map;
maniFileResult mapOpened = maniOpenRetargetMap(mapFile, mapSize, &map);
maniResult carried = maniRetargetPose(&map, &skeleton, pose, &tall, tallPose, retargetScratch,
                                      maniRetargetScratchSize(&map));
```

`maniBuildRetargetedClip` bakes a whole clip for the target, and a
graph's clip node may play another skeleton's clip through a map. A
mirror map pairs left and right bones (by rules on their names or
explicitly) and folds in the rest pose's corrections:

```c
const maniMirrorRuleSource sides[2] = {{"L", 1, maniMirrorSuffix, "R", 1, 0},
                                       {"R", 1, maniMirrorSuffix, "L", 1, 0}};
maniMirrorMapSource mirrorSource = {
    .skeleton = &skeleton, .rules = sides, .ruleCount = 2, .axis = maniMirrorAxisX};
maniBuildResult mirrorBuilt = maniBuildMirrorMap(builder, &mirrorSource);
size_t mirrorSize = 0;
maniResult mirrorCopied = maniCopyBuilt(builder, mirrorFile, mirrorCapacity, &mirrorSize);
maniMirrorMap mirror;
maniFileResult mirrorOpened = maniOpenMirrorMap(mirrorFile, mirrorSize, &mirror);
maniResult mirrored = maniMirrorPose(&mirror, &skeleton, pose, pose);   // in place
```

Mirrored clips (`maniBuildMirroredClip`), a graph's mirror node and
mirrored motion database entries use the same map.

## Springs and ragdolls

Spring bones (hair, tails, cloth strips, VRM's `VRMC_springBone`) are a
spring set built for a skeleton (`maniBuildSpringSet`, or read from a
glTF file with `maniGltfReadSprings`) and a state in your memory
(`maniSpringStateSize`, `maniInitSpringState`). Each frame
`maniStepSprings` moves them at the set's fixed rate against the pose's
model and `maniApplySprings` writes them into the pose; a graph's
spring node does both. The state saves and restores
(`maniSaveSpringState`, `maniRestoreSpringState`).

A ragdoll file maps bones to physics bodies and joints; the library
does no physics. `maniRagdollBodiesFromPose` gives the bodies'
placements for a pose, your physics engine moves them, and
`maniRagdollPoseFromBodies` reads the pose back;
`maniRagdollJointTargets` gives powered joints an animation to follow
and `maniRagdollGetUpPlacement` places a get-up animation where the
body fell. A graph's pose parameter (`maniSetGraphPose`) blends a
physics pose with animation.

## Motion matching

A motion database (`maniBuildMotionDatabase`) holds features of its
clips' frames: bone positions and velocities, trajectory positions and
facings ahead, float tracks, each normalized and weighted.
`maniSearchMotionDatabase` finds the frame nearest a query, exactly,
pruned by bounds over runs of frames; a graph's matching node builds the
query from parameters (a trajectory goal) and plays what it finds,
blended by inertialization. Mirrored entries double a database without
a second clip. Learned motion matching (`maniOpenLearnedModel`,
`maniLearnedStep`, `maniLearnedProject`, `maniLearnedDecompress`) runs
a trained model of the same database in a fixed memory budget, its
training on the offline side.

## Getting data in

The offline side (built with `MAUL_ANIM_OFFLINE`, on by default) holds
the builders, compression and the importers. glTF 2.0 files (JSON or
GLB, meshopt-compressed as gltfpack writes them or not) give skeletons,
skins, influences, morph targets, clips and spring bones; FBX files (binary or ASCII, `maniOpenFbx`) the same through the
same calls. How the importers read or refuse each case of a file is
listed in records mani-0007 to mani-0011 (glTF: JSON and numbers, the
document, skeletons and skins, animations, spring bones) and mani-0012
to mani-0018 (FBX).

```c
maniGltf* gltf = NULL;
maniImportResult read = maniOpenGltf(builder, bytes, size, &gltf);   // bytes kept alive
maniGltfOptions options = maniDefaultGltfOptions();
maniImportResult skeletonImported = maniGltfBuildSkeleton(builder, gltf, 0, &options);
size_t skeletonSize = 0;
maniResult skeletonCopied = maniCopyBuilt(builder, skeletonFile, skeletonCapacity, &skeletonSize);
maniSkeleton skeleton;
maniFileResult skeletonOpened = maniOpenSkeleton(skeletonFile, skeletonSize, &skeleton);

maniImportClipOptions clipOptions = maniDefaultImportClipOptions();   // 30 samples a second
maniImportResult clipImported =
    maniGltfBuildClip(builder, gltf, 0, &skeleton, &options, &clipOptions, NULL);
size_t clipSize = 0;
maniResult clipCopied = maniCopyBuilt(builder, clipFile, clipCapacity, &clipSize);
maniClip clip;
maniFileResult clipOpened = maniOpenClip(clipFile, clipSize, &clip);
maniCloseGltf(gltf);
```

A refused file says why: `read.reason` and `read.offset` name the
problem and the byte. A file cannot make an importer take more memory
than the counts it declares: a compressed view at most 1,024 times its
stream, a Draco stream's points and faces at most its primitive's
accessors'. Counts themselves are the file's, so a host reading files it
does not trust sets its ceiling in the builder's allocator, which is
asked for every block with its size: a block it refuses fails the call
with `mani_errorCapacity`, nothing kept.

The C library's allocator is no ceiling on every system. Linux
overcommits memory by default: `malloc` can succeed for a block larger
than the machine holds, and the process is killed when the block is
first written rather than the call refused. An accessor without a view
(zeros, as Draco's primitives' are) costs its declared count however
small the file: each is held to 16,777,216 elements, but one primitive
may name many, so a file of two kilobytes can ask for gigabytes. Give
such a host's builder an allocator with a budget:

```c
typedef struct Budget
{
    size_t used;
    size_t limit;
} Budget;

static void* BudgetAlloc(size_t size, size_t alignment, void* context)
{
    Budget* budget = context;
    if (alignment > alignof(max_align_t) || size > budget->limit - budget->used)
    {
        return NULL;
    }
    void* memory = malloc(size);
    budget->used += memory != NULL ? size : 0;
    return memory;
}

static void BudgetFree(void* memory, size_t size, size_t alignment, void* context)
{
    Budget* budget = context;
    (void)alignment;
    if (memory != NULL)
    {
        budget->used -= size;
        free(memory);
    }
}
```

```c
Budget budget = {0, (size_t)256 << 20};   // 256 MiB for one file's import
maniBuilderDef guarded = maniDefaultBuilderDef();
guarded.allocator = (maniAllocator){BudgetAlloc, BudgetFree, &budget};
maniBuilder* careful = NULL;
maniResult created = maniCreateBuilder(&guarded, &careful);
```

A full-precision clip compresses within an error you choose, measured on
a shell of points around each bone:

```c
maniCompressOptions tolerance = maniDefaultCompressOptions();   // 0.1 mm at 3 cm
maniCompressReport report;
maniResult compressed = maniCompressClip(builder, &clip, &skeleton, &tolerance, &report);
size_t smallSize = 0;
maniResult smallCopied = maniCopyBuilt(builder, smallFile, smallCapacity, &smallSize);
maniClip small;
maniFileResult smallOpened = maniOpenClip(smallFile, smallSize, &small);
// report.shellError is the largest error found at any sample.
```

Graphs and the retargeting and mirror maps also have a JSON form, read
and written (`maniBuildGraphJson`, `maniWriteRetargetMapJson`,
`maniBuildRetargetMapJson`, `maniWriteMirrorMapJson`,
`maniBuildMirrorMapJson`), so a tool can edit them as text.

