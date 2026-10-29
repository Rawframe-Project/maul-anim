# Changelog

All notable changes to this project are recorded here, grouped as
Added, Changed, Fixed, Removed and Security. The project uses
[Semantic Versioning](https://semver.org/). Before 1.0.0, any release
may change the API, the ABI and every data format.

## [0.6.0](https://github.com/Rawframe-Project/maul-anim/compare/v0.5.0...v0.6.0) (2026-10-29)


### ⚠ BREAKING CHANGES

* every name length a public struct carries is size_t in place of uint32_t: the nameLength of maniBoneSource, maniFloatTrackSource, maniDiscreteTrackSource, maniEventSource, maniColliderGroupSource, maniSpringSource, maniRagdollBodySource, maniMorphTargetSource, maniMorphChannelSource, the graph's parameter, clip, database, model, spring set and sync group sources, maniMirrorNameSource, maniClipEvent and maniRagdollBody; maniGraphConditionSource.eventLength, maniLearnedNameSource.length, maniMatchChannelSource.trackLength, maniMirrorRuleSource.findLength and replaceLength, and maniMirrorNamePairSource.firstLength and secondLength. A builder refuses a name past the new MANI_MAX_NAME_BYTES (2^31 - 1) with mani_errorLimit.
* a saved graph instance and a saved spring state start with the family's state prefix (magic, MANI_ABI_VERSION, format version, header size), so their headers are 32 bytes and every saved state's bytes change. maniRestoreGraphInstance, maniRestoreSpringState and maniRestoreLearnedCheckpoint refuse a state written by a release of another ABI version with mani_errorVersion. Saved states are not kept across releases before 1.0.0, nor across minor releases from 1.0.0.
* every name getter (maniSkeletonBoneName, maniClipFloatTrackName, maniClipDiscreteTrackName, the graph's slot and parameter names, maniMotionDatabaseChannelName, maniLearnedModelFloatTrackName and EventName, maniMorphChannelName and TargetName, maniSpringSetSpringName, and the glTF and FBX importers' names and buffer URI) reports the length through size_t* lengthOut in place of uint32_t*, the type names are taken with.
* **graph:** maniGraphMapCount, maniGraphMapIdentity, maniGraphMapSourceIdentity and maniGraphClipMap are maniGraphRetargetMapCount, maniGraphRetargetMapIdentity, maniGraphRetargetMapSourceIdentity and maniGraphClipRetargetMap; MANI_MAX_GRAPH_MAPS is MANI_MAX_GRAPH_RETARGET_MAPS; maniGraphClipSource.map is retargetMap, and maniGraphAssets.maps and mapCount are retargetMaps and retargetMapCount.
* maniSaveGraphInstance, maniSaveSpringState and maniSaveLearnedCheckpoint take a sizeOut after the capacity and measure with a NULL buffer and capacity 0. maniGraphStateSize, maniSpringSavedSize and maniLearnedCheckpointSize are removed.
* maniMotionDatabaseFeatures, maniMotionDatabaseNormalization, maniLearnedModelNormalization, maniMorphTargetWeights and maniLearnedProject take a capacity and a countOut after the array they fill; maniGltfMorphWeights and maniFbxMorphWeights take a capacity in place of the exact count, and a countOut; maniRagdollBodiesFromPose, maniRagdollJointTargets and maniBuildMask take a countOut after the capacity. maniLearnedOutput gains floatTrackCapacity and eventCapacity after its arrays. A NULL array with capacity 0 measures.

### Fixed

* **graph:** refuse names past the 32-bit offsets ([#144](https://github.com/Rawframe-Project/maul-anim/issues/144)) ([6cbfad3](https://github.com/Rawframe-Project/maul-anim/commit/6cbfad3ea09473a3f1d4154cc31b96a3b6dd660b)), closes [#141](https://github.com/Rawframe-Project/maul-anim/issues/141)
* **import:** refuse names past 4 GiB instead of cutting them short ([#153](https://github.com/Rawframe-Project/maul-anim/issues/153)) ([9ee7b44](https://github.com/Rawframe-Project/maul-anim/commit/9ee7b446b737962edea46630cda41803f8372c62)), closes [#150](https://github.com/Rawframe-Project/maul-anim/issues/150) [#154](https://github.com/Rawframe-Project/maul-anim/issues/154)
* **match:** sum track names in 64 bits and refuse past 4 GiB ([#143](https://github.com/Rawframe-Project/maul-anim/issues/143)) ([074e3db](https://github.com/Rawframe-Project/maul-anim/commit/074e3db198029497c9b63468fff83c0ba8a8cd8d)), closes [#140](https://github.com/Rawframe-Project/maul-anim/issues/140)
* **mirror:** refuse partners past the 32-bit offsets ([#151](https://github.com/Rawframe-Project/maul-anim/issues/151)) ([7ea9184](https://github.com/Rawframe-Project/maul-anim/commit/7ea9184ae4dadadd438a4ef62866fc5d1e8b82e8)), closes [#149](https://github.com/Rawframe-Project/maul-anim/issues/149)
* **mirror:** size the name room in 64 bits ([#142](https://github.com/Rawframe-Project/maul-anim/issues/142)) ([111014c](https://github.com/Rawframe-Project/maul-anim/commit/111014c1aa20cacc0342d55a7ab46e33116aeb9c)), closes [#139](https://github.com/Rawframe-Project/maul-anim/issues/139)


### Changed

* **graph:** name the graph's retargeting maps as such ([#136](https://github.com/Rawframe-Project/maul-anim/issues/136)) ([4f8f25c](https://github.com/Rawframe-Project/maul-anim/commit/4f8f25c864189b099d7c00a0d97830f4c56e19f0))
* record the writer's ABI version in every saved state ([#134](https://github.com/Rawframe-Project/maul-anim/issues/134)) ([4a48a13](https://github.com/Rawframe-Project/maul-anim/commit/4a48a13887a4acc304eecfaeac9d5e508553c308))
* report name lengths as size_t ([#135](https://github.com/Rawframe-Project/maul-anim/issues/135)) ([81d7dbe](https://github.com/Rawframe-Project/maul-anim/commit/81d7dbed3c0d9929ce61de97f6dab580bcd06bff))
* report the saved size from each save call ([#129](https://github.com/Rawframe-Project/maul-anim/issues/129)) ([09a1ff9](https://github.com/Rawframe-Project/maul-anim/commit/09a1ff95e9d20a11dc00e9b6d54d3691de89673a))
* take a capacity for every array a call fills ([#133](https://github.com/Rawframe-Project/maul-anim/issues/133)) ([961b221](https://github.com/Rawframe-Project/maul-anim/commit/961b2211e9bb8c6e9171c9481f89c987ee15f7a2))
* take name lengths in public structs as size_t ([#138](https://github.com/Rawframe-Project/maul-anim/issues/138)) ([58694b7](https://github.com/Rawframe-Project/maul-anim/commit/58694b7803f53e94e5aad8e6791fcd8495a5e766))

## [0.5.0](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.5.0) (2026-10-28)

### Changed

- **Breaking: every file kind's layout and format version change, and
  an opener reads only its own version.** The reserved words are gone
  from the files: the container header is 48 bytes, not 64, and the
  clip, retargeting map, influences, learned model, learned checkpoint,
  spring set, ragdoll profile and mirror map lose their zero words.
  Every kind's format version goes up by one: skeleton 2, clip 4, graph
  7, retargeting map 3, skin 2, influences 2, morph targets 2, motion
  database 4, learned model 2, learned checkpoint 2, spring set 3,
  ragdoll profile 2, mirror map 3. An opener now refuses any format
  version but its own with `mani_errorVersion`, an earlier one included;
  it used to accept an earlier version and read it with the current
  layout. Build every stored file again from its source. Poses, samples
  and other outputs do not change; only hashes of file bytes move
  ([#113](https://github.com/Rawframe-Project/maul-anim/pull/113)).
- **Breaking: results take the Maul family's values.**
  `mani_errorVersion` is -7 (was -4) and `mani_errorFormat` is -8 (was
  -3). `mani_errorState` is removed: a call from an object's own host
  code (its allocator or task hooks) is refused with the new
  `mani_errorBusy` (-10) in its place, and still counts one misuse. Code
  that compares results by name only renames `mani_errorState` to
  `mani_errorBusy`
  ([#115](https://github.com/Rawframe-Project/maul-anim/pull/115)).
- **Breaking: a fixed bound passed returns `mani_errorLimit` (-9).** An
  argument, or a builder's or the trainer's source, past a bound the
  library publishes returns `mani_errorLimit` in place of
  `mani_errorInvalid`, with the same reason and item. This covers the
  aim, CCD, FABRIK and full-body IK solvers, the motion database search,
  and the skeleton, skin, influences, clip, morph targets, spring set,
  retargeting map, motion database, learned model, trainer and graph
  builders. A bound checked while opening a file is still
  `mani_errorFormat`, and a range of a parameter's meaning (a weight in
  [0, 1], a cone up to pi) is still `mani_errorInvalid`. Full-body IK
  scratch that is too small returns `mani_errorCapacity`, not
  `mani_errorInvalid`
  ([#117](https://github.com/Rawframe-Project/maul-anim/pull/117)).
- **Breaking: the bound constants are named `MANI_MAX_*`.**
  `MANI_AIM_MAX_BONES` is `MANI_MAX_AIM_BONES`, `MANI_IK_CHAIN_MAX_BONES`
  is `MANI_MAX_IK_CHAIN_BONES`, `MANI_IK_MAX_ITERATIONS` is
  `MANI_MAX_IK_ITERATIONS`, `MANI_SPRING_MAX_RATE` is
  `MANI_MAX_SPRING_RATE`, `MANI_MORPH_MAX_ATTRIBUTES` is
  `MANI_MAX_MORPH_ATTRIBUTES`, and the `MANI_BODY_IK_MAX_*`,
  `MANI_MATCH_MAX_*`, `MANI_LEARNED_MAX_*` and `MANI_GRAPH_MAX_*` names
  are `MANI_MAX_BODY_IK_*`, `MANI_MAX_MATCH_*`, `MANI_MAX_LEARNED_*` and
  `MANI_MAX_GRAPH_*`. A name that keeps the old form (`MANI_AIM_MAX_CONE`,
  `MANI_SPRING_MAX_STEPS`, `MANI_GRAPH_LEARNED_MAX_STEPS`) is a range or
  an internal cap, not a bound refused with `mani_errorLimit`
  ([#117](https://github.com/Rawframe-Project/maul-anim/pull/117)).
- **Breaking: public structs have no `reserved` field.** The field is
  removed from every public struct, and builders no longer refuse a
  source for a nonzero one. Remove it from initializers.
  `maniFileResult` is `{result, tag, offset}`: `tag` names the section
  the first wrong byte lies in, 0 in the header and table, so positional
  initializers take the tag second
  ([#109](https://github.com/Rawframe-Project/maul-anim/pull/109)).
- **Breaking: a root graph's slots are checked against the root's own
  lists.** `maniBuildGraph` refuses, with `maniBuildReasonNode` at the
  node's place, a root node that names a clip, sync group, database,
  model or spring set slot past the root's own list. In a graph with a
  subgraph such a node used to build with the subgraph's slot of that
  number: with 1 clip in the root and 1 in the subgraph, a root clip
  node naming slot 1 played the subgraph's clip. Only a host's own
  `maniGraphSource` structs reach this; the JSON form resolves each name
  within its own graph. Every other graph builds to the same bytes. To
  migrate, list in the root every asset and sync group its nodes name
  ([#93](https://github.com/Rawframe-Project/maul-anim/pull/93)).
- `MANI_MAX_INFLUENCES` (64 weighted influences on a vertex) and
  `MANI_MAX_GRAPH_DEPTH` (16 levels of subgraphs) are public
  ([#117](https://github.com/Rawframe-Project/maul-anim/pull/117)).
- On WebAssembly, the public calls no longer go through the
  floating-point environment's out-of-line entries, which have nothing
  to switch there, so the static library is about 14.7 KB smaller.
  Results do not change
  ([#101](https://github.com/Rawframe-Project/maul-anim/pull/101)).
- The user guide's getting started page explains `mani_errorLimit`,
  `mani_errorBusy` and the one format version a release reads, and its
  examples drop the `reserved` field and use the new constant names.

### Fixed

- The FBX importer bounds a file's property count. A file with 2^32 - 1
  properties or more wrapped the count, and the importer wrote past the
  property array it had allocated from it. Such a file is now refused
  at the property past the bound; every file under it reads as before
  ([#105](https://github.com/Rawframe-Project/maul-anim/pull/105)).
- A motion database file whose entry is too long for a frame count and
  stores `0xFFFFFFFF` as its count is refused at that entry's count. It
  was still refused, but at a later, wrong place. Valid databases open
  and build as before
  ([#107](https://github.com/Rawframe-Project/maul-anim/pull/107)).

## [0.4.0](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.4.0) (2026-10-25)

### Changed

- **Breaking: a subgraph's lists and slots are checked like the
  root's.** `maniBuildGraph` refuses with `maniBuildReasonNode`, at the
  node's place, a node inside a subgraph that names a clip, sync group,
  database, model or spring set slot past the subgraph's own list. Such
  a graph used to build when the root had a slot of that number, which
  the node then named: with 2 clips in the subgraph and 9 in the root,
  a clip node naming slot 3 played the root's clip 3. It also refuses
  with `maniBuildReasonCount` a subgraph with a list whose count is not
  zero and whose pointer is NULL; its clip, sync group, database, model
  and spring set lists used to be read through that pointer when a node
  named one of their slots. Only a host's own `maniGraphSource` structs
  reach either case; the JSON form resolves and checks its lists. Every
  other graph builds to the same bytes. To migrate, list in each
  subgraph every asset and sync group its nodes name
  ([#77](https://github.com/Rawframe-Project/maul-anim/pull/77)).
- On 32-bit x86 the library is built with SSE2 math
  (`-msse2 -mfpmath=sse`), so it needs a processor with SSE2, and every
  build now leaves trapping math at the compiler's default, from the
  family's compiler flags. 64-bit builds' tests and pinned hashes pass
  unchanged
  ([#73](https://github.com/Rawframe-Project/maul-anim/pull/73)).

### Fixed

- `maniBuildGraph` refuses a subgraph's sync group whose name is NULL
  with `maniBuildReasonCount` at item 0, before anything is laid out, as
  it refuses a subgraph's list that is not given. A root's own nameless
  group is still refused by the check with the sync group list's Name
  reason. Only a host's own `maniGraphSource` structs reach this; the
  JSON form always gives a name. Every graph that built before builds to
  the same bytes
  ([#81](https://github.com/Rawframe-Project/maul-anim/pull/81)).

## [0.3.0](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.3.0) (2026-10-25)

### Changed

- **Breaking: a graph's time scales are bounded.** On any path from the
  output, the product of the time scales' sizes must be at most
  `MANI_GRAPH_MAX_TIME_FACTOR` (1e6), which is far past any playback
  rate a graph needs. A scale that reads a parameter counts at the
  larger size of the parameter's bounds, and a time scale may read only
  a parameter with bounds. Scales on parallel branches never multiply.
  `maniBuildGraph` refuses a source past either rule with
  `maniBuildReasonScalar`, and `maniOpenGraph` refuses such a file with
  `mani_errorFormat`. Each node record now holds its time factor, so
  the graph format goes from 5 to 6, and `maniOpenGraph` refuses an
  earlier format with `mani_errorVersion`. To migrate, give bounds to
  each parameter a time scale reads, and build every graph file again.
  Results do not change for any graph still taken.
  ([#66](https://github.com/Rawframe-Project/maul-anim/pull/66))

### Fixed

- A clip node's step that is not finite moves nothing, and the marker
  walk's count is bounded by two loops of markers. Results do not
  change for steps shorter than about 2^53 clip lengths.
  ([#64](https://github.com/Rawframe-Project/maul-anim/pull/64))
- A graph's binary32 timers saturate at `FLT_MAX`: the fade time of a
  machine and of a one-shot, and an inertialize node's step and elapsed
  time. Every timer value below `FLT_MAX` gives the same bits as
  before.
  ([#67](https://github.com/Rawframe-Project/maul-anim/pull/67))

## [0.2.1](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.2.1) (2026-10-24)

### Changed

- The runtime's core (sampling, blending, the model pass, skeletons and
  root motion) is about 1.0 KB smaller at `-O2` with GCC: 68,562 bytes,
  against ozz-animation's 69,698. The skeleton's opener and the file
  checks are compiled for size under GCC and Clang. With GCC, opening a
  skeleton takes about 7% more instructions and opening a compressed
  clip about 9% more. Opening a full clip or a motion database does not
  change, and neither does any call made every frame. Results do not
  change.
  ([#29](https://github.com/Rawframe-Project/maul-anim/pull/29))

### Fixed

- The host's allocator and task hooks run in the host's own
  floating-point setting, like the rest of the host's code. In 0.2.0
  they ran in the library's setting while a public call was in
  progress.
  ([#27](https://github.com/Rawframe-Project/maul-anim/pull/27))

## [0.2.0](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.2.0) (2026-10-24)

### Changed

- **Breaking:** motion matching searches a motion database through an
  exact k-d tree, in place of bounds over runs of frames. Every search
  returns the same frame and cost as before. On large databases a search
  takes far fewer instructions: about 6 times fewer at 100,000 frames
  and up to 16 times fewer at 300,000 frames with the default leaf. On
  small databases with many features (around 5,000 frames and 45
  features) it can be slightly slower. The index takes about 49 bytes a
  frame at the default leaf, up from about 17.
  The motion database format version is 3, and
  `maniOpenMotionDatabase` refuses any other format version with
  `mani_errorVersion`. Build databases again from their sources, along
  with the graphs that name them, since a database's identity changes.
  A learned trainer's checkpoint does not resume against a rebuilt
  database.
  `maniMotionDatabaseSource` gains `leafFrames` (8, 16 or 32 frames a
  leaf, 0 for 16) and a reserved word that must be 0, so positional
  initializers must name them.
  ([#24](https://github.com/Rawframe-Project/maul-anim/pull/24))

### Fixed

- A host that runs its threads with flush-to-zero, denormals-are-zero
  or another rounding mode now gets the same bits as every other host.
  Every public call that computes switches to the library's
  floating-point setting and restores the host's on return, and so do
  the learned trainer's tasks on the host's worker threads. Results in
  the default setting do not change.
  ([#26](https://github.com/Rawframe-Project/maul-anim/pull/26))

## [0.1.0](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.1.0) (2026-10-24)

### Changed

- Compressed clips keep each kind of sub-track together and decode
  several sub-tracks at once: sampling at a random time takes about 28%
  fewer instructions and a forward cursor about 15% fewer, and poses and
  float tracks are the same bits as before.
  The clip format version is 3, and `maniOpenClip` refuses a clip of
  any other format version, compressed or full precision, with
  `mani_errorVersion`. Build stored clips again from their sources, or
  compress them again from full-precision clips made with this release.

### Fixed

- A learned motion matching node whose model gives a value past the
  float range, or whose root velocity moves past it over the step, no
  longer passes it on: `maniUpdateGraph`, `maniEvaluateGraph` and
  `maniEvaluateGraphSubset` complete the call with valid, finite
  outputs and return `mani_errorFormat`, and `maniLearnedStep`,
  `maniLearnedProject` and `maniLearnedDecompress` return
  `mani_errorFormat` instead of success. `maniGraphNodeView`'s reserved
  word is now `failed`.

## [0.0.1](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.0.1) (2026-10-24)

The first release: skeletal animation from import to the skinned
vertex, with the same results on every platform, compiler and SIMD
backend, and no allocation at run time.

### Added

- Skeletons, poses in local and model space composed as glTF composes
  nodes, and the polar decomposition of model matrices.
- Clips on a uniform grid sampled at the same cost at any time, with
  float and discrete tracks, events and sync markers; a forward cursor
  with the same results, in a faster form and a compact one.
- Clip compression within an error measured on a shell around each
  bone, close to ACL's sizes at a third of its error.
- Blending with per-bone masks, pose differences and additive layers
  and clips, and root motion taken out at build and read as moves
  across any number of loops, forward or backward.
- Animation graphs: clip, blend, mask, additive, select, time scale
  and seek nodes; 1D, triangulated and banded blend spaces; state
  machines with conditions on parameters, time, a clip's phase, end or
  events; one-shots; inertialization and dead blending; sync groups by
  phase or markers; subgraphs; a JSON form; saved and restored
  instances; node inspection for debuggers; evaluation of a subset of
  bones.
- Motion matching: motion databases with weighted feature channels, an
  exact pruned search, a trajectory predictor and a graph node;
  learned motion matching with a model format, a graph node and an
  in-library trainer whose results are the same on every platform and
  with any number of threads.
- Inverse kinematics: two-bone, aim and look-at over a chain, CCD and
  FABRIK chains with cone, twist and hinge limits, full-body XPBD and
  SDLS solvers, a turn limit node, each also a graph node.
- Spring bones as `VRMC_springBone` 1.0 defines them, with its extended
  colliders and limits, the same motion at any frame rate.
- Ragdoll profiles mapping bones to a physics engine's bodies and
  joints, bodies and motor targets from poses, poses from bodies, and
  get-up placement; a pose input for graphs.
- Retargeting between skeletons with different names, chain lengths,
  rests and proportions, and mirroring on rigs whose sides do not have
  mirrored axes, for poses, baked clips, graphs and motion databases;
  JSON forms for both maps.
- Skinning: skins, palettes, linear, dual quaternion and blended
  skinning and centers of rotation over one influence format, as jobs
  over strided vertex streams; morph targets with in-betweens.
- glTF 2.0 import (JSON and GLB, `EXT_meshopt_compression`,
  `KHR_meshopt_compression`, `KHR_draco_mesh_compression`,
  `KHR_animation_pointer`, `VRMC_springBone` and the `OMI_physics_*`
  extensions) and FBX import (binary and ASCII, versions 3000 to 7700):
  skeletons, skins, influences, morph targets and clips.
- Validation of every file, JSON text and saved state as it is opened,
  in every build: bad data is refused with a status and the offset of
  the first wrong byte, and counts no misuse on the builder; a misuse
  count reports only calls a correct program never makes.
- A CPU check for the AVX2 build, a portable scalar build with the same
  results, a CMake package and a pkg-config module.
- Support for Linux on x86-64 and arm64, macOS, Windows with
  `clang-cl`, and WebAssembly.
