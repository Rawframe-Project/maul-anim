# Development

How to build, debug and change Maul Anim as a maintainer. Building and installing as a user is in [getting-started.md](../user/getting-started.md).

## Contents

- [Toolchain](#toolchain)
- [Build options for development](#build-options-for-development)
- [Running one test](#running-one-test)
- [Generated sources and test data](#generated-sources-and-test-data)
- [Differentials](#differentials)
- [Code size](#code-size)
- [Reproducing a fuzz finding](#reproducing-a-fuzz-finding)

## Toolchain

The organization's CI image, `ghcr.io/rawframe-project/ci-cpp`, has everything below. On another machine:

- CMake 3.25 or newer and Ninja.
- GCC 14 and Clang 20 (`clang-20`, `clang-tidy-20`), with the sanitizer and libFuzzer runtimes. A system GCC older than 14 is refused at configure.
- `clang-format` at the version CI pins: `pip install clang-format==22.1.5`.
- Python 3 for the checks, the generators and the documentation site (`pip install markdown==3.6` for the site).
- For the differentials only: a C++17 compiler and network access (CMake fetches ufbx, ACL, RTM and ozz-animation at pinned commits), and Godot 4.7.2 for the retargeting comparison.

## Build options for development

| Option | Effect |
|---|---|
| `MAUL_ANIM_WERROR` | Warnings as errors, as CI builds. |
| `MAUL_ANIM_SANITIZE` | AddressSanitizer and UBSan. |
| `MAUL_ANIM_TSAN` | ThreadSanitizer. |
| `MAUL_ANIM_COVERAGE` | gcov instrumentation. |
| `MAUL_ANIM_FUZZ` | The libFuzzer targets, `fuzz_<area>` (Clang only). |
| `MAUL_ANIM_DIFFERENTIAL` | The differential programs against ufbx, ACL, ozz-animation and Godot; needs `MAUL_ANIM_OFFLINE`. |
| `MAUL_ANIM_SIMD=scalar` | The portable backend, to compare with a vector build. |
| `MAUL_ANIM_BUILD_TESTS`, `_BENCH` | On when the project is built on its own. |

## Running one test

Each suite is `test/test_<name>.c`, built as `test_<name>` at the top of the build tree and registered with CTest as `<name>`, labeled with its name:

```sh
cmake -B out/asan -G Ninja -DCMAKE_C_COMPILER=clang-20 -DCMAKE_BUILD_TYPE=Debug \
  -DMAUL_ANIM_SANITIZE=ON -DMAUL_ANIM_WERROR=ON
cmake --build out/asan
ctest --test-dir out/asan -L clip
./out/asan/test_clip
```

A suite prints one line per failed check with its file and line. Suites marked `WHITEBOX` in `CMakeLists.txt` include `src/` and are skipped in shared builds, where internal symbols are hidden. The glTF and FBX suites read `test/data/`.

Build and test through the workspace's heavy-job wrapper on the shared development machine; the full suite under the sanitizers takes about a minute.

## Generated sources and test data

Never edit these by hand; change the generator and run it.

| Files | Generator | Command |
|---|---|---|
| `src/generated/decimal_powers.h` | `tools/gen_decimal_powers.py` | `python3 tools/gen_decimal_powers.py` (`--check` compares) |
| `src/generated/inflate_tables.h` (family) | `tools/gen_inflate_tables.py` | `python3 tools/gen_inflate_tables.py` |
| `src/generated/math_constants*.h` (family) | `tools/gen_math_constants.py` | `python3 tools/gen_math_constants.py` |
| `test/json_number_cases.h` | `tools/gen_number_cases.py`, an exact rational oracle | `python3 tools/gen_number_cases.py > test/json_number_cases.h` |
| `test/json_format_cases.h` | `tools/gen_format_cases.py` | `python3 tools/gen_format_cases.py > test/json_format_cases.h` |
| `test/gltf_fixture_expect.h` | `tools/gen_gltf_expect.py`, glTF's interpolation in binary64 | `python3 tools/gen_gltf_expect.py > test/gltf_fixture_expect.h` |
| `test/fbx_fixture_expect.h` | `tools/fbx_differential.c --expect`, ufbx's values | see [Differentials](#differentials) |
| binary FBX fixtures in `test/data/fbx/` | `tools/fbx_fixtures.py` (with `tools/fbx_binary.py`) | `python3 tools/fbx_fixtures.py` |
| `test/gltf_meshopt_fixtures.h` | `tools/meshopt_fixtures.cc`, meshoptimizer's encoders at commit f0a684f1 | build against meshoptimizer, then `meshopt_fixtures > test/gltf_meshopt_fixtures.h` and `clang-format -i` it |
| `test/data/draco/*.drc` and their hashes | `tools/draco_fixtures.cc`, Google's Draco at commit d429de8 | build against Draco, then `draco_fixtures test/data/draco` |
| expected values in `test_gltf_draco_predict.c` and `test_gltf_draco_traverse.c` | `tools/draco_predict_oracle.cc`, Google's Draco | build against Draco and run; values are pasted into the tests |
| meshopt file hashes in `test_gltf_compression.c` | `tools/meshopt_files.py`, meshoptimizer's decoder | `python3 tools/meshopt_files.py libmeshoptimizer.so <files>` |
| `test/data/gltf/SimpleSkinSeam-draco.glb` | `tools/draco_skin_seam.py`, then `gltf-transform draco` | see `test/data/gltf/README.md` |

The C++ generators need the third-party libraries named at the top of each file; they are never built by CMake. The glTF sample models in `test/data/gltf/` are kept exactly as published, with their source and license in that folder's README.

## Differentials

Programs built with `-DMAUL_ANIM_DIFFERENTIAL=ON` compare the library with other implementations on the same data; `.github/workflows/differential.yml` runs them on a schedule and by hand, never in the pull request check.

| Program | Compares with | Data |
|---|---|---|
| `fbx_differential` | ufbx: skeletons, skins, clips and weights; known deviations in `tools/fbx-deviations.txt` | ufbx's corpus at the pinned commit (`build/_deps/ufbx-src/data`) |
| `acl_differential` | ACL: compressed size, error and time | ozz-animation's Quaternius `ual.fbx`, checked by hash |
| `ozz_differential` | ozz-animation's blending job | the same clips |
| `ik_differential` | ozz-animation's two-bone and aim jobs | a million random chains |
| `retarget_godot` with `tools/retarget_godot.gd` | Godot 4.7.2's RetargetModifier3D, run headless | 500 random poses of two humanoids |

To refresh the FBX fixtures' expected values after a change ufbx agrees with:

```sh
cmake -B out/diff -G Ninja -DCMAKE_C_COMPILER=clang-20 -DCMAKE_CXX_COMPILER=clang++-20 -DMAUL_ANIM_DIFFERENTIAL=ON
cmake --build out/diff --target fbx_differential
./out/diff/fbx_differential --deviations tools/fbx-deviations.txt --expect test/fbx_fixture_expect.h test/data/fbx/*.fbx
```

A disagreement is a defect in the library, or a deviation decided and listed with its reason in `tools/fbx-deviations.txt`.

## Code size

```sh
cmake -B out/size -G Ninja -DCMAKE_C_COMPILER=gcc-14 -DCMAKE_BUILD_TYPE=Release \
  -DMAUL_ANIM_OFFLINE=OFF -DMAUL_ANIM_SIMD=scalar -DMAUL_ANIM_BUILD_TESTS=OFF -DMAUL_ANIM_BUILD_BENCH=OFF \
  "-DCMAKE_C_FLAGS_RELEASE=-O2 -DNDEBUG"
cmake --build out/size
python3 tools/size_report.py out/size/libmaul-anim.a --check O2
```

prints each part's text against its ceiling in `tools/size-budget.txt` and fails on any allocator symbol. [size-and-performance.md](../internals/size-and-performance.md) explains the parts and the ceilings.

## Reproducing a fuzz finding

A fuzz target writes the input that failed as `crash-<hash>` in its working directory. To reproduce:

```sh
cmake -B out/fuzz -G Ninja -DCMAKE_C_COMPILER=clang-20 -DMAUL_ANIM_FUZZ=ON -DMAUL_ANIM_BUILD_TESTS=OFF
cmake --build out/fuzz
./out/fuzz/fuzz_clip -max_total_time=300 corpus-clip
./out/fuzz/fuzz_clip crash-<hash>
```

Always pass `-max_total_time`: a fuzz target otherwise runs until it finds something. Every crash becomes a test in the suite of its module, with the smallest input that still fails (`-minimize_crash=1`). The test suites can write a fuzz target's first seed: `./out/asan/test_clip --seed corpus-clip/sample.clip`.
