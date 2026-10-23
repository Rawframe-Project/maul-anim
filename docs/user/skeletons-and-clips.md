# Skeletons, clips and poses

A program builds its skeletons and clips offline (or imports them, see [Importing](importing.md)), opens the files at run time, and samples clips into poses, which it blends and takes to model space for rendering.

## Contents

- [Skeletons](#skeletons)
- [Clips](#clips)
- [Poses and sampling](#poses-and-sampling)
- [Blending](#blending)
- [The model pass](#the-model-pass)
- [Root motion](#root-motion)
- [Not obvious](#not-obvious)

## Skeletons

A skeleton is built from its bones: a name, a parent (-1 for a root) and a rest transform. The builder orders bones so that parents come first; `maniCopySkeletonOrder` says where each source bone went.

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

`maniSkeletonBoneCount`, `maniSkeletonParent`, `maniSkeletonRest` and `maniSkeletonFindBone` read an opened skeleton. Its identity (`maniSkeletonIdentity`) is a hash of its bones that every asset built for it records, so a clip of another skeleton is refused, not played wrongly.

## Clips

A clip holds bone tracks (translation, rotation and scale channels of keys), float and discrete tracks, events and markers, and root motion, sampled on a uniform grid. Keys may be at any times, with step, linear or cubic spline interpolation; the builder resamples them at the clip's rate.

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

A channel without keys stays at the rest. The rate decides where step keys land (each on the first sample at or after it) and how closely curves are followed; `maniMeasureClip` reports both against the source. A compressed clip plays through the same calls ([Importing](importing.md#compression)).

## Poses and sampling

A pose holds a skeleton's local transforms and, once composed, its model matrices. Its memory is the program's: `maniPoseSize` bytes aligned to `MANI_POSE_ALIGNMENT`.

```c
maniPose* pose = NULL;
maniResult ready = maniInitPose(&skeleton, poseMemory, maniPoseSize(&skeleton), &pose);
maniResult sampled = maniSampleClip(&clip, &skeleton, 0.5f, pose);   // half a second in
maniTransform spine = maniPoseLocal(pose, 1);
```

Sampling at a time is random access: any clip, any time, the same cost. To play forward, a cursor (`maniInitClipCursor`, `maniSampleClipCursor`) keeps the samples it decoded and gives the same bits for less work. A cursor given `maniClipCursorSize` bytes also keeps its samples laid out for eight-lane playback, which is faster; given `maniClipCursorCompactSize` bytes it skips that layout, which costs about 92 bytes a bone. A crowd of large rigs picks the compact size. Both give random access's bits.

A pose holds no pointers: copying its bytes copies it.

## Blending

Blending takes layers, each a pose, a weight and optional per-bone weights:

```c
const maniBlendLayer layers[2] = {{walking, 0.25f, NULL}, {running, 0.75f, NULL}};
maniResult blended = maniBlendPoses(&skeleton, layers, 2, 0.1f, pose);
```

A bone whose layers' weights add to less than the threshold (here 0.1) takes the rest for the remainder. Layers are summed in order. Per-bone weights come from `maniBuildMask`, which turns chains (a bone, or a bone and its subtree) into a weight per bone.

Additive animation works on differences: `maniPoseDifference` takes one pose from another, and `maniAddPoses` adds weighted differences onto a pose. An additive clip, built with a basis (the rest or its first frame), samples straight into a difference. A pose remembers which kind it holds, and calls refuse to mix them.

## The model pass

Model matrices are composed on demand:

```c
maniResult composed = maniComputeModel(pose, &skeleton);
maniAffine head = maniPoseModel(pose, 2);   // the head in model space
```

Setting a local marks the model stale; `maniPoseModelCurrent` says whether it is current, and calls that need the model refuse a stale one. `maniPoseModelTransform` and `maniDecomposeAffine` split a model matrix back into translation, rotation and scale.

## Root motion

A clip can declare one bone whose motion moves the character: its translation along chosen axes and its rotation (none, the turn about one axis, or all of it), measured from the rest, the first sample or the origin. The builder takes that motion out of the bone, so the clip plays in place, and keeps it beside the clip. Each frame the program reads the move over its step and moves the character:

- `maniClipRootMotion` gives the move from a time over a signed step: across any number of loops, backward too, and a step split into parts composes to the whole.
- `maniComposeRootMotion` and `maniBlendRootMotion` combine moves as poses combine.
- `maniRestoreRootMotion` puts the motion back into a sampled pose, for a program that keeps it there.

Importers take the root motion source per clip in `maniImportClipOptions`.

## Not obvious

- The file bytes must outlive every view opened over them and stay where they were opened.
- Opening checks everything, so it is the slow call; open once at load, never per frame.
- A loop whose last sample does not match its first pops at the seam; the clip's loop policy and the root motion source's loop option decide how the seam is closed.
