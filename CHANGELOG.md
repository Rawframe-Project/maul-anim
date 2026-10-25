# Changelog

All notable changes to this project are recorded here, grouped as
Added, Changed, Fixed, Removed and Security. The project uses
[Semantic Versioning](https://semver.org/). Before 1.0.0, any release
may change the API, the ABI and every data format.

## [0.3.0](https://github.com/Rawframe-Project/maul-anim/compare/v0.2.1...v0.3.0) (2026-10-25)


### ⚠ BREAKING CHANGES

* **graph:** a graph's time scales are bounded. On any path from the output, the product of their sizes must be at most MANI_GRAPH_MAX_TIME_FACTOR (1e6), a scale read from a parameter counting at the larger size of the parameter's bounds, and a time scale may read only a parameter with bounds. maniBuildGraph refuses a source past either rule with maniBuildReasonScalar, and maniOpenGraph refuses such a file with mani_errorFormat. The graph format goes from 5 to 6, and maniOpenGraph refuses a file of an earlier format with mani_errorVersion. To migrate, give each parameter a time scale reads its bounds, and build every graph file again with the builder.

### Fixed

* **graph:** bound the marker walk's step ([#64](https://github.com/Rawframe-Project/maul-anim/issues/64)) ([057c47c](https://github.com/Rawframe-Project/maul-anim/commit/057c47c76e26200ae7b41788e57515070c72f3ab))
* **graph:** bound the product of a graph's time scales ([#66](https://github.com/Rawframe-Project/maul-anim/issues/66)) ([f9917f1](https://github.com/Rawframe-Project/maul-anim/commit/f9917f159a539be323086ebeb7d48bae2a9e7ccc))
* **graph:** saturate the graph's binary32 timers at FLT_MAX ([#67](https://github.com/Rawframe-Project/maul-anim/issues/67)) ([9583669](https://github.com/Rawframe-Project/maul-anim/commit/95836697c02c8032d05ed93849ad1389750241ba))

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
