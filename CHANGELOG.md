# Changelog

All notable changes to this project are recorded here, grouped as
Added, Changed, Fixed, Removed and Security. The project uses
[Semantic Versioning](https://semver.org/). Before 1.0.0, any release
may change the API, the ABI and every data format.

## [0.2.0](https://github.com/Rawframe-Project/maul-anim/compare/v0.1.0...v0.2.0) (2026-10-24)


### ⚠ BREAKING CHANGES

* **match:** the motion database format version is 3. A database keeps its frames in a k-d tree's order with each node's bounding box, in place of bounds over runs of frames, and maniOpenMotionDatabase refuses any other format version with mani_errorVersion. Build databases again from their sources, and the graphs that name them, since a database's identity changes; a learned trainer's checkpoint does not resume against the rebuilt database. maniMotionDatabaseSource gains leafFrames (8, 16 or 32 frames a leaf, 0 for 16) and a reserved word that must be 0; positional initializers name them. Searches return the same frames and costs as before.

### Fixed

* enter the floating-point environment in every computing call ([#26](https://github.com/Rawframe-Project/maul-anim/issues/26)) ([d19d608](https://github.com/Rawframe-Project/maul-anim/commit/d19d6086e0e4138139b2bb39c2203ae111cc9e8a)), closes [#23](https://github.com/Rawframe-Project/maul-anim/issues/23)


### Changed

* **match:** search motion databases through an exact k-d tree ([#24](https://github.com/Rawframe-Project/maul-anim/issues/24)) ([cd88f01](https://github.com/Rawframe-Project/maul-anim/commit/cd88f01427df5a5adb410e2359b15b6e255bef0c))

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
