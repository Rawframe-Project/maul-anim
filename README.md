# Maul Anim

Skeletal animation in C: sampling, blending, graphs, motion matching, IK, retargeting and skinning, with glTF and FBX import.

[![CI](https://github.com/Rawframe-Project/maul-anim/actions/workflows/ci.yml/badge.svg)](https://github.com/Rawframe-Project/maul-anim/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/github/license/Rawframe-Project/maul-anim)](LICENSE)
[![Release](https://img.shields.io/github/v/release/Rawframe-Project/maul-anim)](https://github.com/Rawframe-Project/maul-anim/releases)
[![OpenSSF Scorecard](https://api.scorecard.dev/projects/github.com/Rawframe-Project/maul-anim/badge)](https://scorecard.dev/viewer/?uri=github.com/Rawframe-Project/maul-anim)

A game character needs more than playing a clip. Clips are blended by speed and direction, switched by state machines, eased across jumps, layered with additive motion, bent by IK so feet meet the ground, shared between rigs of other proportions, and imported from the files artists export. Maul Anim does all of that in one dependency-free library:

- **Clips and poses**: skeletons, poses in local and model space, clips sampled at the same cost at any time, a forward cursor, and compression to ACL's sizes within an error measured on the skeleton.
- **Blending and graphs**: per-bone masks, additive layers, blend spaces, state machines, one-shots, sync by phase or markers, inertialization and dead blending, events and root motion, built once and shared by any number of characters.
- **Motion matching**: an exact search over a motion database, and learned motion matching with an in-library trainer.
- **IK and secondary motion**: two-bone, aim and look-at, CCD and FABRIK chains with joint limits, full-body solvers, spring bones as VRM defines them, and the seam to any physics engine for ragdolls.
- **Retargeting and mirroring** between rigs with different rests and proportions.
- **Skinning**: linear, dual quaternion, blended and centers of rotation, morph targets, palettes for GPUs.
- **Import**: glTF 2.0 (meshopt and Draco compression, VRM spring bones, OMI physics) and FBX, binary and ASCII.

Results are the same bits on every platform, compiler and SIMD backend, so servers, clients and replays agree. The runtime allocates nothing and starts no thread: assets are views over the program's bytes, and per-character state lives in the program's memory. The offline side (builders, importers, the compressor) can be left out of a shipping build.

## Quick start

It builds with CMake 3.25 or newer and GCC 14 or newer, Clang 19 or newer, or Apple Clang. Ubuntu 24.04's default compiler is GCC 13, which the build refuses, so there install GCC 14, with Git and CMake, and tell CMake to use it:

```sh
sudo apt install git cmake gcc-14
export CC=gcc-14
```

Then, on Linux and macOS:

```sh
git clone https://github.com/Rawframe-Project/maul-anim.git
cd maul-anim
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

A `build/` that GCC 13 already configured keeps that compiler and fails again ("GCC 14 or newer is required"). Remove it and configure again, with `CC` exported as above, then carry on with the rest of the block from `cmake --build build`:

```sh
rm -rf build && cmake -B build -DCMAKE_BUILD_TYPE=Release
```

On Windows the library builds with `clang-cl`, and on the web with Emscripten; [Getting started](docs/user/getting-started.md) lists what each needs.

Install it with `cmake --install build --prefix <prefix>` and use it from CMake:

```cmake
find_package(maul-anim REQUIRED CONFIG)
target_link_libraries(game PRIVATE maul-anim::maul-anim)
```

or through pkg-config as `maul-anim`. On x86-64 the default build uses AVX2 and FMA; a program calls `maniCheckCpu` once before anything else, or the library is configured with `-DMAUL_ANIM_SIMD=scalar` to run on any CPU with the same results. [Getting started](docs/user/getting-started.md) covers the options and the model the API follows.

## Platforms

Linux on x86-64 and arm64 with GCC 14 or Clang 19 and newer, macOS with Apple Clang, Windows with `clang-cl`, and WebAssembly with Emscripten; CI builds and tests each. The public headers are C17 and C++17, so any C or C++ program can include them.

## Documentation

- [The documentation site](https://rawframe-project.github.io/maul-anim/), with the API reference generated from the headers.
- [User documentation](docs/README.md): one page per part of the library.
- [How it works](docs/internals/overview.md) and [how to develop it](docs/operations/development.md).
- [Changelog](CHANGELOG.md).

Maul Anim belongs to the Maul family of standalone C libraries, which share one set of conventions.

## Contributing

Issues are welcome. Pull requests are accepted for issues a maintainer has approved; see [CONTRIBUTING.md](https://github.com/Rawframe-Project/.github/blob/main/CONTRIBUTING.md). The project is developed with AI coding agents, directed and reviewed by its maintainers; outside contributions follow the [AI policy](https://github.com/Rawframe-Project/.github/blob/main/AI_POLICY.md).

## License

MIT; see [LICENSE](LICENSE). The test fixtures under `test/data/` carry their own licenses, named in each folder's `README.md`.
