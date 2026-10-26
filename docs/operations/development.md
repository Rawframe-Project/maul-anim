# Development

How to build, debug and change Maul Anim as a maintainer. Building and installing as a user is in [getting-started.md](../user/getting-started.md).

## Contents

- [Toolchain](#toolchain)
- [Presets](#presets)
- [Build options for development](#build-options-for-development)
- [Running one test](#running-one-test)
- [Generated sources and test data](#generated-sources-and-test-data)
- [Differentials](#differentials)
- [Benchmarks](#benchmarks)
- [Code size](#code-size)
- [Debugging a failure](#debugging-a-failure)
- [Reproducing a fuzz finding](#reproducing-a-fuzz-finding)

## Toolchain

The organization's CI image, `ghcr.io/rawframe-project/ci-cpp`, has everything below. On another machine:

- CMake 3.25 or newer and Ninja.
- GCC 14 and Clang 20 (`clang-20`, `clang-tidy-20`), with the sanitizer and libFuzzer runtimes. A system GCC older than 14 is refused at configure.
- `clang-format` at the version CI pins: `pip install clang-format==22.1.5`.
- Python 3 for the checks, the generators and the documentation site (`pip install markdown==3.6` for the site).
- shellcheck for the shell scripts in `tools/`; without it the source check says so and skips them, except in CI.
- For the differentials only: a C++17 compiler and network access (CMake fetches ufbx, ACL, RTM and ozz-animation at pinned commits), and Godot 4.7.2 for the retargeting comparison.

## Presets

`CMakePresets.json` holds every CI cell as a configure, build and test preset of the same name, built in `out/<preset>`: `linux-gcc-14`, `linux-arm64-gcc-14`, `linux-clang-20`, `linux-clang-20-asan`, `linux-clang-20-tsan`, `linux-shared`, `macos-clang`, `windows-clang-cl`, `windows-shared`, `windows-runtime` and `web-emscripten` (with emsdk active), plus `linux-fuzz` and `linux-differential` for the fuzz targets and the differentials. `tools/check.sh` runs them by tier ([checks.md](checks.md)). ThreadSanitizer inside a container needs `--security-opt seccomp=unconfined`.

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
| `MAUL_ANIM_TEST_PACKAGE` | The `package` test: installs the build and builds `test/consumer` against it. |
| `MAUL_ANIM_BUILD_TESTS`, `_BENCH` | On when the project is built on its own. |

## Running one test

Each suite is `test/test_<name>.c`, built as `test_<name>` at the top of the build tree and registered with CTest as `<name>`, labeled with its name:

```sh
cmake --preset linux-clang-20-asan
cmake --build --preset linux-clang-20-asan
ctest --preset linux-clang-20-asan -L '^clip$'
./out/linux-clang-20-asan/test_clip
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

Programs built with `-DMAUL_ANIM_DIFFERENTIAL=ON` compare the library with other implementations on the same data; `.github/workflows/nightly.yml` runs them every night and by hand, never in the pull request check ([checks.md](checks.md)).

| Program | Compares with | Data |
|---|---|---|
| `fbx_differential` | ufbx: skeletons, skins, clips and weights; known deviations in `tools/fbx-deviations.txt` | ufbx's corpus at the pinned commit (`_deps/ufbx-src/data` in the build tree) |
| `acl_differential` | ACL: compressed size, error and time | ozz-animation's Quaternius `ual.fbx`, checked by hash |
| `ozz_differential` | ozz-animation's blending and model pass jobs: results, instructions and time | the same clips |
| `ik_differential` | ozz-animation's two-bone and aim jobs: results, then instructions and time against the jobs alone and their whole cost to a host; Maul Anim's CCD and FABRIK timed alone | 100,000 random chains |
| `retarget_godot` with `tools/retarget_godot.gd` | Godot 4.7.2's RetargetModifier3D, run headless | 500 random poses of two humanoids |

Each timing program takes a last argument that runs one side alone (`acl_differential`'s loop, `ozz_differential`'s side, `ik_differential`'s only), so a side's user instructions a call are the difference of two runs with different repetitions under `perf stat -e instructions:u`, which the machine's load does not change; [size-and-performance.md](../internals/size-and-performance.md#speed) gives the counts and how they were taken. `tools/ozz_runtime_size.sh <ozz source>` measures ozz-animation's runtime at the commit the differentials pin, the size the core is held to.

To refresh the FBX fixtures' expected values after a change ufbx agrees with:

```sh
cmake --preset linux-differential
cmake --build --preset linux-differential --target fbx_differential
./out/linux-differential/fbx_differential --deviations tools/fbx-deviations.txt \
  --expect test/fbx_fixture_expect.h test/data/fbx/*.fbx
```

A disagreement is a defect in the library, or a deviation decided and listed with its reason in `tools/fbx-deviations.txt`.

## Benchmarks

`bench/` times the calls a host makes every frame on fixed scenes built at its start (`bench/scenes.c`): a rig of 64 bones; 16 clips of 10 s at 30 samples a second, full and compressed; a graph blending two clips; a motion database of their 4,800 frames; spring chains on the hair and a tail; the IK solvers; a skin of 20,000 vertices of four influences; and 32 morph targets over the same mesh. Beside those calls it counts a frame of the graph against the same samples and blend made by hand (`graph.frame`, `graph.byhand`), building a mask, and opening the skeleton, a clip, a compressed clip and the motion database. It needs `MAUL_ANIM_OFFLINE`, as the scenes are built by the builders.

```sh
cmake --preset linux-clang-20 && cmake --build --preset linux-clang-20 --target maul-anim_bench
tools/bench.sh out/linux-clang-20
```

`tools/bench.sh` prints the machine and its load average before and after, each case's nanoseconds a call (the best of five runs) and, where `perf` counts user instructions, its instructions a call. Name cases to run only those; `maul-anim_bench --list` prints them. Wall time on the shared machine moves with its load: quote instruction counts for a change's effect, and timings only from a quiet machine with the load beside them. Run it through the workspace's heavy-job wrapper on the shared development machine. It is never a test: timings are not repeatable enough for a gate.

## Code size

```sh
cmake -B out/size -G Ninja -DCMAKE_C_COMPILER=gcc-14 -DCMAKE_BUILD_TYPE=Release \
  -DMAUL_ANIM_OFFLINE=OFF -DMAUL_ANIM_SIMD=scalar -DMAUL_ANIM_BUILD_TESTS=OFF -DMAUL_ANIM_BUILD_BENCH=OFF \
  "-DCMAKE_C_FLAGS_RELEASE=-O2 -DNDEBUG"
cmake --build out/size
python3 tools/size_report.py out/size/libmaul-anim.a --check O2
```

prints each part's text against its ceiling in `tools/size-budget.txt` and fails on any allocator symbol. [size-and-performance.md](../internals/size-and-performance.md) explains the parts and the ceilings.

## Debugging a failure

**A CI cell.** A failed build cell uploads its ctest logs as `logs-<preset>` for a week, `windows` and `web` as `logs-windows` and `logs-web-emscripten`. `fuzz` uploads nothing: it prints each finding's report ([Reproducing a fuzz finding](#reproducing-a-fuzz-finding)). To run a Linux cell as CI does, run its preset in the image `ci.yml` pins, through the heavy-job wrapper on the shared machine:

```sh
docker run --rm -v "$PWD":/w -w /w ghcr.io/rawframe-project/ci-cpp:<tag> bash -c \
  'git config --global --add safe.directory /w && cmake --preset linux-gcc-14 &&
   cmake --build --preset linux-gcc-14 && ctest --preset linux-gcc-14'
```

ThreadSanitizer inside the container needs `--security-opt seccomp=unconfined`. A tree configured there records `/w` paths, so the host cannot build in it again: keep a preset in one place, the container or the host.

**A moved hash.** A golden scene ([determinism.md](../internals/determinism.md)) hashes several outputs together, and most print the hash they computed under the failed check. If the change was meant (an arithmetic order, a constant, a quaternion's hemisphere), pin the new value and name the change of results in the pull request ([release.md](release.md)). If it was not:

- Configure the same preset again with `-DMAUL_ANIM_SIMD=scalar` and run the suite. When the scalar build gives the pinned hash and the vector build does not, a vector kernel no longer matches its scalar code. `test_blend_lanes` compares the blend's lanes with their scalar loops; for the other kernels, print the hash of each output the scene folds in, in both builds, to find the first that differs.
- When one compiler alone differs, look for contraction or a fast-math flag in the target's flags (`cmake/CompilerFlags.cmake`), or a `<math.h>` call outside `tools/libm-allowed.txt`.
- When every build differs, `git bisect run` with the suite finds the commit.

**A part over its ceiling.** `size_report.py` names the part. Build `main` and the branch into fresh trees with the same flags (a reused tree can hold stale objects), then compare `size` of the two archives object by object and `nm -S --size-sort` on the object that grew.

**The floating-point environment check.** `tools/check_float_env.py` names each public function that computes floats outside the library's environment. Make it enter the environment and do its work in a `MANI_NOINLINE` worker, or, when it does no floating-point arithmetic and no comparison, list it in `tools/float-env-allowed.txt` under the reason it needs none ([determinism.md](../internals/determinism.md#the-floating-point-environment)).

**A differential.** Run `fbx_differential` on the one file it names (`MISMATCH`, then the aspect). A disagreement is fixed in the library, or decided and listed with its reason in `tools/fbx-deviations.txt`. The other differentials print their worst difference against their bound and exit non-zero past it; [Differentials](#differentials) gives their arguments.

**`guide_import_memory_limit`.** It runs `test_guide_import --default-huge` under `ulimit -v 524288`, outside the sanitizer builds. It prints `default allocator: other` and fails when importing the morph targets of a 2 KB file that asks for 8.6 GB returns anything but `mani_errorCapacity`: the C library's allocator must refuse under the limit, and the importer must pass that refusal on as Capacity. It prints `default allocator: the build not reached` and fails when the limit is too tight to create the builder or open the file, so a limit that leaves no room never passes.

## Reproducing a fuzz finding

`tools/fuzz.sh` runs each target through `tools/fuzz_target.sh`, which writes the input that failed as `crash-<hash>` in `out/fuzz-crashes/<target>/`, the target's log in `out/fuzz-logs/<target>.log` and, on a finding, a report in `out/fuzz-reports/<target>.txt`. The report gives the target, the kind of each finding (crash, leak, timeout, oom) and the sanitizers' report. It never gives the input's bytes, its file name (the SHA-1 of the input) or the seed, which is the log's first line. CI prints the report and keeps the rest on the runner, so a CI finding is found again by fuzzing its target locally:

```sh
cmake --preset linux-fuzz && cmake --build --preset linux-fuzz
./out/linux-fuzz/fuzz_clip out/fuzz-crashes/fuzz_clip/crash-<hash>
tools/fuzz.sh run out/linux-fuzz 300  # every target for five minutes, four at a time
```

Always pass `-max_total_time`: a fuzz target otherwise runs until it finds something. Every crash becomes a test in the suite of its module, with the smallest input that still fails (`-minimize_crash=1`). The seeds and the corpus are described in [checks.md](checks.md#fuzzing).
