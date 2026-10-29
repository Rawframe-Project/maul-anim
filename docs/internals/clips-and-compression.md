# Skeletons, poses, clips and compression

The core of the library: skeletons and their poses, clips sampled at any time, compressed clips, blending and root motion. Everything else (graphs, IK, retargeting, skinning) reads and writes the poses this page describes.

## Transforms and skeletons

A bone's local transform (`maniTransform`) is a translation, a unit quaternion (x, y, z, w) and a scale per axis, applied scale first, then rotation, then translation. Its model matrix is its parent's times its local matrix, a 3x4 affine matrix (`maniAffine`): exactly how glTF composes nodes. So a non-uniform scale on a parent shears its children, mirrors are negative scales, and any glTF hierarchy plays as authored with no flag per bone. FBX's inherit modes, which no single composition rule reproduces, are baked into this one rule at import ([fbx-import.md](fbx-import.md)).

A skeleton ([`src/skeleton.c`](../../src/skeleton.c)) stores bones in depth-first order: every subtree is a contiguous range, children and roots in their authored order, at most 65,535 bones with `MANI_NO_BONE` as no parent. Every range a part needs (an IK chain, a subtree mask, a subset) is then one span, and a parent always precedes its children, so one forward pass composes the model. Names are unique, non-empty UTF-8, found by binary search over their hashes, which the file keeps sorted. The builder accepts bones in any order and reports where each went (`maniBuilder_CopySkeletonOrder`).

## Poses

A pose ([`src/pose.c`](../../src/pose.c)) is caller memory of `maniSkeleton_GetPoseSize` bytes: a 64-byte header (the bone count, the skeleton's identity, whether the model is current, whether it holds a difference), then local transforms per component in groups of `MANI_POSE_GROUP_BONES` (eight) bones, then the model matrices. The layout is the library's and may change between releases; hosts read and write through functions (`maniPose_GetLocal`, `maniPose_SetLocal`, `maniPose_ReadLocals`, `maniPose_GetModel`). Eight bones a group is one eight-lane vector a component, which is what blending and the forward cursor compute on.

Setting a local marks the model stale; `maniPose_ComputeModel` composes it in bone order, and calls that need the model (the palette, a ragdoll's bodies) refuse a stale one instead of reading old matrices. A pose given the skeleton of another identity is refused by one comparison.

`maniDecomposeAffine` splits a matrix into translation, rotation and scale with the rotation of the polar decomposition, computed in binary64 by Higham's scaled Newton iteration: exact up to rounding without shear, the nearest rotation with it, a mirror as a negative x scale, and a zero or flat axis completed from the others rather than refused. When the columns are already orthogonal (most rigs), the rotation is the columns divided by their lengths and the iteration is skipped.

## Clips on one grid

A clip holds samples on one uniform grid: each multiple of 1 / rate up to the duration (30 a second by default, at most `MANI_MAX_SAMPLES`). The builder ([`src/clip_build.c`](../../src/clip_build.c)) evaluates every channel of the source's keys (step, linear or cubic spline, glTF's three interpolations) at each sample time in binary64, with the library's own trigonometry, and rounds once. Rotations are normalized and each put in the hemisphere of the sample before.

One grid makes any time cost the same: a division gives the two samples and the factor between them, translation and scale interpolate linearly and rotations by normalized lerp. Seeking, backward play and phase sync, which graphs do constantly, cost what forward play costs. The price is that a step key lands on the first sample at or after it and is spread over one sample period by interpolation; the sample rate is the host's lever, and `maniBuilder_MeasureClip` reports how late each step plays.

Besides bone tracks a clip holds float tracks (morph weights and host curves, sampled on the same grid; a step track holds its sample), discrete tracks (whole numbers held from key to key), events (a time, a name hash and a 32-bit payload) and sync markers. Events and markers are kept in time order, so the events a playhead passes between two times, forward or backward, are one range of indices (`maniClip_GetEventRange`).

## The forward cursor

`maniClip_Sample` is random access. A cursor (`maniInitClipCursor`, `maniClipCursor_Sample`) in caller memory keeps the two decoded samples a time falls between, so forward play decodes a sample only when the playhead passes it, and gives exactly random access's bits. A clip of another identity or at another address rebinds it, dropping what it held; a jump in time decodes the samples it lands between, so seeking through a cursor costs what random access costs.

On a vector backend the cursor can also hold its two samples in the pose's group layout, with masks for the channels it interpolates, so each pose is one eight-lane step a group instead of a walk over every channel. That costs about 92 bytes a bone against about 18 for the decoded samples alone, which matters for crowds of large rigs, so the host chooses by the memory it gives: `maniClip_GetCursorSize` includes the groups, `maniClip_GetCursorCompactSize` does not, and a cursor plays by groups only when its memory holds them. On the scalar backend the two sizes are equal and the cursor walks. Building lanes from the walk's values each pose was tried and cost more than the lanes saved; keeping the samples laid out once, when decoded, is what pays.

## Compressed clips

`maniCompressClip` makes a compressed clip from a full-precision one, within an error the caller chooses (`maniCompressOptions`: 0.1 mm at 3 cm by default). The method is ACL's ([`src/clip_compress.c`](../../src/clip_compress.c), [`src/clip_compressed.c`](../../src/clip_compressed.c)):

- Each channel of each bone and each float track is a sub-track: default (the rest, nothing stored), constant (one value) or animated.
- Samples fall into segments of about 16. Animated sub-tracks keep a range over the clip in binary32 and, with several segments, a range per segment on 8 bits a component rounded outward, and per segment a bit rate: 1 to 23 bits a component, raw binary32, or 0 for a value constant in the segment.
- The file keeps each kind of sub-track together, as ACL does: rotations, translations and scales over the tracks that store them (two bitsets a kind, constant and animated), then the float tracks; the constants, the clip ranges, the rates, the segment ranges and each sample's bits follow that order. Clip ranges come in blocks of eight, a load a component on eight lanes.
- Rotations store three components and rebuild the fourth, made positive. ACL always drops w; here a rotation drops the component whose smallest magnitude over the clip is largest, because rebuilding a component near 0 loses precision whatever the bits, and the choice costs two spare bits of a byte already stored.
- The error metric moves points at a shell distance on each bone's axes through the hierarchy in binary64; the error is the farthest one moves. The rate search visits bones leaves first, tries each bone's rate combinations in increasing size and keeps the first that holds the bone and all its descendants within their precision, which is why every bone stays within its precision at every sample whenever binary32 can hold it. `maniCompressReport` says when it cannot.

The compressor measures through the sampler's own arithmetic, so its choices, and therefore its bytes, are the same on every machine. Random access decodes the two samples a time falls between, a kind eight sub-tracks at a time on the lanes, each lane with the scalar arithmetic in its order, so every backend gives the same bits; two samples in one segment share their rates and segment ranges, decoded once. A subset decodes only its bones; the cursor works on either form. Against ACL on ozz-animation's Quaternius clips (`acl_differential`) the compressed size is 1.071 of ACL's at a third of its largest error, and compression takes 0.15 of its time. Random access takes 1.30 times ACL's instructions ([#39](https://github.com/Rawframe-Project/maul-anim/issues/39)): the bits are read one sub-track at a time, as the SIMD header has no integer lanes to read eight at once.

## Blending

`maniBlendPoses` ([`src/blend.c`](../../src/blend.c)) takes layers (a pose, a weight, optional per-bone weights) in order, a threshold and an output. Per bone, each weight is divided by the bone's total, or by the threshold where the total is below it, the rest taking the shortfall's share, so poses slide into the rest as weights fall. Rotations are summed on the arc that agrees with the sum so far, then normalized; translations and scales are convex combinations. Order matters, as in ozz-animation and Unreal. Graphs use `MANI_BLEND_THRESHOLD` (1e-5).

Weights outside [0, 1], a threshold outside (0, 1] and values that are not finite are refused before the output is touched, where ozz silently zeroes negative weights: the mistake surfaces at the call that made it.

A pose records whether it holds a difference. `maniComputePoseDifference` takes one pose from another, `maniPose_AddLayers` adds weighted differences (`t + w d_t`, `r nlerp(1, d_r, w)`, `s (1 + w (d_s - 1))`), and blends take layers of one kind and write that kind. The model pass, the palette and subset sampling refuse differences. An additive clip names its basis (the skeleton's rest or its first frame) and stores each channel's difference from it; sampling it writes a difference. Keeping the kind in the pose makes mixing a pose into a difference a refused call rather than a silent artifact, as graphs refuse it when built.

`maniBuildMask` turns chains (a bone alone, or its subtree) into per-bone weights in skeleton order: a chain on the bone first, else a subtree chain on its nearest ancestor, else a default.

## Root motion

A clip may name one bone whose motion is its root motion: translation along chosen axes and rotation (none, the twist about an axis, or all of it), each from a reference (the rest, the first sample or the origin), in the bone's parent's frame. The builder takes it out of the bone at every sample, in binary64, and stores it beside the clip, so the clip plays in place and the host moves the character by moves it reads ([`src/root_motion.c`](../../src/root_motion.c)). There is no extraction at run time.

A move is a translation and a unit rotation in the character's frame where it begins. Over a signed step from a playhead, a clamped clip gives the move between the two clamped times; a repeating clip composes the partial periods at either end with the whole period's move (or its inverse, backward) raised to the number of wraps by squaring. So a step split anywhere composes to the whole step, backward undoes forward, and any number of wraps costs their logarithm. Moves blend by the weighted mean of translations and the normalized weighted sum of rotations, each turned into the first one's hemisphere, as poses blend. `maniClip_RestoreRootMotion` puts the motion back into a pose for hosts that keep it there.

## Traps

- A clip's rate decides where step keys land; a change of rate changes events' and steps' timing, not just smoothness.
- The pose layout is internal. A host reading pose memory directly breaks on the next release.
- A repeating clip's period move is from its first sample to its last; a loop whose end does not match its start drifts unless its root motion source asks for the loop option, which spreads the difference over the clip.
