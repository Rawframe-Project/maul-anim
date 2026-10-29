# Getting started

Maul Anim is a C library. It builds with CMake, installs as a CMake package and a pkg-config module, and needs nothing but a C compiler and the C standard library.

## Requirements

- CMake 3.25 or newer.
- GCC 14 or Clang 19 or newer. On Windows, `clang-cl`: the Visual Studio component "C++ Clang tools for Windows". MSVC's own `cl` compiles the public headers but not the library.
- Emscripten for WebAssembly builds.

The public headers are plain C17 and C++17, so programs that include them need neither a C23 compiler nor C at all.

## Build and install

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
cmake --install build --prefix /path/to/prefix
```

Then, from CMake:

```cmake
find_package(maul-anim REQUIRED CONFIG)
target_link_libraries(game PRIVATE maul-anim::maul-anim)
```

or through pkg-config, `pkg-config --cflags --libs maul-anim`. A project can also add the source tree with `add_subdirectory`; tests and benchmarks are then off by default. [`test/consumer/`](../../test/consumer) is a complete program that uses only the installed package.

## Build options

| Option | Default | Effect |
|---|---|---|
| `MAUL_ANIM_BUILD_SHARED` | `OFF` | A shared library instead of a static one. |
| `MAUL_ANIM_OFFLINE` | `ON` | The offline side: builders, compression, the JSON forms, training, the glTF and FBX importers. A game that ships prebuilt files turns it off. |
| `MAUL_ANIM_SIMD` | `auto` | `auto` builds x86-64 for AVX2 and FMA and arm64 for NEON; `scalar` builds portable code that runs on any CPU. Both give the same results. |

## The CPU check

An AVX2 build stops on its first vector instruction on a CPU without AVX2 and FMA (before about 2013). A program checks once, before any other call, that the CPU runs the build:

```c
const char* backend = NULL;
int32_t runs = maniCheckCpu(&backend);   // before any other call
// runs == 0: this CPU cannot run the build's backend (backend names it,
// "avx2" say): refuse to start, or load a build made with
// -DMAUL_ANIM_SIMD=scalar, which runs on any CPU with the same results.
```

Configure prints the backend, and the installed package names it (`maul-anim_SIMD_BACKEND` in CMake, the description in pkg-config), so a package meant for older machines is built with `-DMAUL_ANIM_SIMD=scalar`.

## The model

- **Units and axes.** Meters and seconds by convention; the library does not care which units a program uses as long as it is consistent. Rotations are unit quaternions `{x, y, z, w}`. A bone's local transform is translation, rotation and scale (`maniTransform`), applied scale first, as glTF composes nodes.
- **Data is files.** Skeletons, clips, skins, graphs, maps, motion databases and the rest are built offline into files and opened in place at run time. Opening checks every byte, allocates nothing and keeps a view over the program's bytes, which must stay alive and 16-byte aligned while the view is used. Files built against a skeleton are refused with another skeleton, so a mismatch is an error, not a wrong pose.
- **No allocation at run time.** A pose, a graph instance or a solver's scratch lives in memory the program gives, its size from a `...Size` query and its alignment from a `..._ALIGNMENT` constant. Only builders, importers and the trainer allocate, through the allocator in their def (zeroed means the C library's).
- **Results.** A fallible call returns `maniResult`: `mani_success` (0) or a negative error (`mani_errorInvalid`, `mani_errorCapacity`, `mani_errorFormat`, `mani_errorVersion`, `mani_errorLimit`, `mani_errorBusy`); `maniResultName` names any of them. `mani_errorLimit` means an argument or a source passes one of the library's fixed bounds, each a `MANI_MAX_*` constant in the headers, which more memory would not move, such as `MANI_MAX_NAME_BYTES` (2^31 - 1) for every name a source carries (a `MANI_*_MAX_*` name, such as `MANI_AIM_MAX_CONE`, is a range or an internal cap, not such a bound); a file past a bound is malformed and gives `mani_errorFormat`. Opening a file returns a `maniFileResult` whose `offset` names the first byte found wrong; building returns a `maniBuildResult` whose `reason`, `item` and `other` say what in the source was refused; importing returns a `maniImportResult` with the reason and byte.
- **Versions.** A def from its `maniDefault...Def` function carries the version of the headers the program was built with; a library of another version refuses it with `mani_errorVersion`. Files carry their format's version and are refused the same way when the library does not read it: before 1.0 a library reads its own format version of each kind only, so a file of an earlier or a later release is built again from its source.
- **Threads.** The library starts none. Opened files are read-only and may be shared by every thread; a pose, an instance or a builder is used by one thread at a time. Each function's documentation says what may run at once.
- **Determinism.** The same inputs give the same bits on every platform, compiler and backend, so poses can be hashed and compared across machines and replays repeat exactly. A thread's flush-to-zero, denormals-are-zero or rounding mode changes nothing: each call that computes runs in the library's own floating-point setting and leaves the thread's as it found it.

## Where to go next

- [Skeletons, clips and poses](skeletons-and-clips.md): building data, sampling, blending, root motion.
- [Importing](importing.md): glTF and FBX files, compression, a memory budget for untrusted files.
- [Graphs](graphs.md), [Inverse kinematics and physics](ik-and-physics.md), [Skinning](skinning.md), [Retargeting and mirroring](retargeting-and-mirroring.md), [Motion matching](motion-matching.md).
- The [API reference](https://rawframe-project.github.io/maul-anim/api.html), generated from the headers.
