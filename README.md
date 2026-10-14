# Maul Anim

Skeletal animation for games and tools: skeletons and poses, clip
sampling, blending, animation graphs and state machines, inertial
transitions, motion matching, root motion, inverse kinematics,
retargeting, skinning and compression, with glTF 2.0 and FBX import and
its own file formats. Written in
C23 with public headers any C17 or C++17 program can include, with no
dependency and an MIT license.

It covers:

- **Poses:** skeletons with skins and their skinning palettes, poses in
  local and model space, composed as glTF composes nodes.
- **Skinning:** linear blend, dual quaternion and optimized centers of
  rotation over one influence format, morph targets, palettes in GPU
  layouts.
- **Clips:** bone, float and discrete tracks, events and sync markers,
  sampled at the same cost forward, backward and at random (a cursor
  makes forward playback cheaper and gives the same bits), and
  compressed with per-track bit rates under an error measured where a
  skinned vertex would be.
- **Blending:** normalized and per-bone blends, additive layers, blend
  spaces in one and two dimensions, phase and marker sync.
- **Graphs:** typed parameters, hierarchical state machines,
  cross-fades, inertialization and dead blending, one-shots, events and
  root motion, built once and shared by any number of instances.
- **Motion matching:** weighted feature channels, a database file, an
  exact search, and learned motion matching.
- **IK:** two-bone, aim and look-at, CCD and FABRIK chains with joint
  limits, and full-body solvers.
- **Physics-driven poses:** spring bones, and the boundary to any
  physics engine for ragdolls and physical animation.
- **Retargeting** between skeletons with different rests and
  proportions.
- **Offline:** glTF 2.0 and FBX importers, builders, the compressor,
  and a text form for graphs, behind the `MAUL_ANIM_OFFLINE` option;
  the runtime builds without them.

Results are identical on every platform. The runtime allocates
nothing and starts no thread: assets are views over the caller's
bytes, and instances live in the caller's memory.

## Status

Not released. The design is decided; the library is built in slices.
This skeleton has 30 public functions.

## Building

Requirements: CMake 3.25 and GCC 14 or Clang 19 or newer; on Windows,
`clang-cl` (the Visual Studio component "C++ Clang tools for Windows").

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

## Design

The rules every Maul library follows are in `docs/conventions.md` and
`docs/adr/`; the records particular to this library are listed in
`docs/adr/mani.md`.

## License

MIT; see `LICENSE`.
