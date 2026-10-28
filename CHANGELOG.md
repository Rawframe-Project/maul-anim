# Changelog

All notable changes to this project are recorded here, grouped as
Added, Changed, Fixed, Removed and Security. The project uses
[Semantic Versioning](https://semver.org/). Before 1.0.0, any release
may change the API, the ABI and every data format.

## [0.5.0](https://github.com/Rawframe-Project/maul-anim/compare/v0.4.0...v0.5.0) (2026-10-28)


### ⚠ BREAKING CHANGES

* **family:** a bound of the library passed by an argument or a builder's or trainer's source returns mani_errorLimit (-9) in place of mani_errorInvalid, with the same reason and item; full-body IK scratch too small returns mani_errorCapacity. The bound constants are renamed MANI_MAX_*: MANI_AIM_MAX_BONES is MANI_MAX_AIM_BONES, MANI_IK_CHAIN_MAX_BONES is MANI_MAX_IK_CHAIN_BONES, MANI_IK_MAX_ITERATIONS is MANI_MAX_IK_ITERATIONS, MANI_BODY_IK_MAX_* is MANI_MAX_BODY_IK_*, MANI_MATCH_MAX_* is MANI_MAX_MATCH_*, MANI_LEARNED_MAX_* is MANI_MAX_LEARNED_*, MANI_MORPH_MAX_ATTRIBUTES is MANI_MAX_MORPH_ATTRIBUTES, MANI_SPRING_MAX_RATE is MANI_MAX_SPRING_RATE and MANI_GRAPH_MAX_* is MANI_MAX_GRAPH_*. MANI_MAX_INFLUENCES and MANI_MAX_GRAPH_DEPTH are new.
* **family:** `mani_errorVersion` is -7 and `mani_errorFormat` is -8, the Maul family's values. `mani_errorState` is removed. A call from an object's own host code is refused with the new `mani_errorBusy` (-10).
* **format:** every file kind's layout and format version change, and an opener refuses any format version but its own with `mani_errorVersion`, an earlier one included. Files are built again from their sources.
* **family:** the `reserved` field is removed from every public struct, and builders no longer refuse sources by it. `maniFileResult` is `{result, tag, offset}`; positional initializers of it take the tag second.
* **graph:** maniBuildGraph refuses a graph whose root nodes name a clip, sync group, database, model or spring set slot past the root's own list, with maniBuildReasonNode at the node's place. Such a graph with a subgraph used to build with the subgraph's slot of that number. To migrate, list in the root every asset and sync group its nodes name.

### Fixed

* **fbx:** bound a file's property count ([#105](https://github.com/Rawframe-Project/maul-anim/issues/105)) ([7fce170](https://github.com/Rawframe-Project/maul-anim/commit/7fce1701c7292dbcac497ce9f243109145c516ff)), closes [#104](https://github.com/Rawframe-Project/maul-anim/issues/104)
* **format:** drop the files' reserved words and read one format version ([#113](https://github.com/Rawframe-Project/maul-anim/issues/113)) ([2d5181e](https://github.com/Rawframe-Project/maul-anim/commit/2d5181e409d554f0569847d4dce24a15c3d31c60))
* **graph:** refuse a root's slot past the root's own lists ([#93](https://github.com/Rawframe-Project/maul-anim/issues/93)) ([520ef9a](https://github.com/Rawframe-Project/maul-anim/commit/520ef9a56afa5d91a12cf61aff5a21444b4db66c)), closes [#94](https://github.com/Rawframe-Project/maul-anim/issues/94)
* **match:** report an entry too long for a count apart from any count ([#107](https://github.com/Rawframe-Project/maul-anim/issues/107)) ([8dac8fe](https://github.com/Rawframe-Project/maul-anim/commit/8dac8fe8b82edf25411950b1ac7093f04f9ff026)), closes [#106](https://github.com/Rawframe-Project/maul-anim/issues/106)


### Changed

* **family:** remove reserved fields from public structs ([#109](https://github.com/Rawframe-Project/maul-anim/issues/109)) ([6a179e9](https://github.com/Rawframe-Project/maul-anim/commit/6a179e9418ab8c22a3d7ce90dd6b708e9c3559cb))
* **family:** return limit for a fixed bound passed ([#117](https://github.com/Rawframe-Project/maul-anim/issues/117)) ([eeb5136](https://github.com/Rawframe-Project/maul-anim/commit/eeb513625124c8630aad964a7d966d65eb900950))
* **family:** take the family's result values; busy for host code ([#115](https://github.com/Rawframe-Project/maul-anim/issues/115)) ([bd08379](https://github.com/Rawframe-Project/maul-anim/commit/bd08379887fc4b7d6f6addc9f7c1333497da36d8))

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
