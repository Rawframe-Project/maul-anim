# Maul Anim

Maul Anim is a C library of skeletal animation: skeletons and poses, clip sampling and compression, blending, animation graphs, motion matching, IK, spring bones, the ragdoll seam, retargeting, mirroring and skinning, with glTF and FBX import. It is part of the Maul family; the family rules are in [docs/internals/maul-conventions.md](docs/internals/maul-conventions.md), and [docs/internals/overview.md](docs/internals/overview.md) explains how the parts fit.

It never compromises on:

1. **Same bits everywhere.** Every output is identical on every platform, compiler and SIMD backend ([determinism.md](docs/internals/determinism.md)). Hosts pin results by hash and replay from parameters.
2. **No allocation at run time.** Assets are views over the caller's bytes; poses, instances, cursors and scratch are caller memory. Only builders, importers and the trainer allocate, through the host's allocator.
3. **Hostile input.** Every file, text, saved state and argument is validated and refused with a status and a location, in every build. Opening a file checks every byte once.
4. **Measured against the best open rivals.** Size and speed targets against ACL, ozz-animation, ufbx, Godot and Holden's references are measured, never asserted ([size-and-performance.md](docs/internals/size-and-performance.md)).

## Commands

```sh
cmake -B out/asan -G Ninja -DCMAKE_C_COMPILER=clang-20 -DCMAKE_BUILD_TYPE=Debug -DMAUL_ANIM_SANITIZE=ON -DMAUL_ANIM_WERROR=ON
cmake --build out/asan && ctest --test-dir out/asan -j 8      # the whole suite, about a minute
ctest --test-dir out/asan -L clip                              # one suite, by label
python3 tools/check_source.py && python3 tools/check_modules.py && python3 tools/check_lengths.py \
  && python3 tools/check_docs.py && python3 tools/check_guide.py   # the family's rules
clang-format --dry-run -Werror <changed .c and .h files>       # clang-format 22.1.5, pinned
```

Use `gcc-14` or `clang-20`; the machine's default `gcc` is 13, which the build refuses. Run full builds and fuzzing through the workspace's heavy-job wrapper on the shared machine. Each suite is `test/test_<label>.c`, registered in `CMakeLists.txt` with `maul_add_test`.

## Ways to hurt yourself

- **Killing by pattern.** Never `pkill -f`, `pgrep | kill` or `kill` a PID found by matching a name or path: other sessions build and fuzz on the same machine with the same binary names. Kill only a PID you started.
- **Endless fuzzing.** A libFuzzer target runs until it finds something. Always pass `-max_total_time`.
- **Editing generated files.** `src/generated/` and the generated test headers and fixtures are written by the generators in `tools/` ([development.md](docs/operations/development.md#generated-sources-and-test-data)). Change the generator and run it.
- **Editing shared family files.** These are synced from the canonical copies in the organization's `.github` repository (`maul/`) and fail CI when they differ: `docs/internals/maul-conventions.md`; `.clang-format`, `.clang-tidy`, `.editorconfig`, `.gitattributes`, `.gitignore`; every file in `cmake/`; `bench/CMakeLists.txt`; `test/test_harness.h`, `test/test_inflate.c`, `test/test_math_float.c`; `tools/build_docs.py`, `check_docs.py`, `check_guide.py`, `check_lengths.py`, `check_modules.py`, `check_source.py`, `gen_api.py`, `gen_inflate_tables.py`, `gen_math_constants.py`; and `src/cpu.c`, `cpu.h`, `hash.c`, `simd.h`, `inflate.c`, `inflate.h`, `elementary_float.c`, `elementary_float.h` with their `src/generated/` tables. Change the canonical copy first, through the manager.
- **Changing results silently.** A change to arithmetic order, a constant or a quaternion's hemisphere choice changes bits every host may have pinned. The pinned hashes in the tests catch it; a deliberate change names its effect in the pull request.

## Invariants

- **Determinism:** binary32 in a fixed order, no contraction, no fast math, no estimate instructions; only the `<math.h>` functions in `tools/libm-allowed.txt`; vector kernels use the SIMD header's separate multiply and add, and are tested against their scalar code bit for bit. A public function that computes floats enters the library's floating-point environment and does its work in a `MANI_NOINLINE` worker ([determinism.md](docs/internals/determinism.md#the-floating-point-environment)).
- **Files:** every kind uses the container in `src/container.h`. A change to a kind's bytes bumps its format version. Opening checks every byte; later calls do not check again.
- **Identities:** a file built for a skeleton records its identity, and every pairing compares it.
- **Version:** `MANI_VERSION_*` between the release-please markers in `include/maul-anim/base.h`; CMake reads it from there and only release-please changes it. `MANI_ABI_VERSION` is stamped into the builder's and the trainer's defs, which check the cookie first and the version second.
- **Public headers** are the common subset of C17 and C++17; every public function has a `///` block with `@param`, `@return` and a thread safety paragraph (`tools/check_docs.py`).
- **Modules:** `tools/modules.txt` lists which module may include which; a new include needs a line there. The runtime never includes a builder. Every runtime module is placed in a part of `tools/size-budget.txt`.
- **Offline split:** builders, importers, the compressor, the JSON forms and the trainer build only with `MAUL_ANIM_OFFLINE`; runtime code must build and link without them.
- **No allocation at run time:** `tools/size_report.py` fails on any allocator symbol in a runtime-only archive.
- **Calls back in:** the host's allocator and task hooks are called only through `src/host_code.h`, which marks the object for the call.

## Testing

Follow the [repository standard's Tests section](https://github.com/Rawframe-Project/.github/blob/main/docs/repository-standard.md#tests); in this repository:

- Test through the public headers. `WHITEBOX` suites include `src/` for internal readers that have a specification of their own (JSON, numbers, inflate, Draco, meshopt, the elementary functions); do not test private state.
- References are written in the test, in binary64, from the method's description; compare the library's result with them, not with its own earlier output. Golden hashes stay a few scenes.
- Every opener has tests that change a file at the byte each check reads, and a fuzz target in `test/fuzz_<area>.c`; a fuzz finding becomes a test in its module's suite.
- Inputs that could be read past their end go in a heap allocation of exactly their length, so AddressSanitizer sees an overread.
- Every C snippet of `docs/user/` appears unchanged in `test/test_guide.c` or `test/test_guide_import.c`; `tools/check_guide.py` enforces it.
- Fixtures in `test/data/` keep their sources' READMEs (origin, license, how they were made). Expected values from other libraries come from the generators in `tools/`.

## Documentation

Follow the [repository standard's Documentation section](https://github.com/Rawframe-Project/.github/blob/main/docs/repository-standard.md#documentation). A change in what a caller must do or can rely on updates the page of `docs/user/` for that part and, if it is in a snippet, the matching code in `test/test_guide*.c`. The API reference is generated from the headers into the documentation site; never commit it.

## Pull requests

Titles are conventional commits. Scopes: `math`, `skeleton`, `pose`, `clip`, `compress`, `blend`, `rootmotion`, `graph`, `match`, `learned`, `ik`, `spring`, `ragdoll`, `retarget`, `mirror`, `skin`, `morph`, `format`, `import`, `gltf`, `fbx`, `build`, `bench`, `tools`, `family`. Example: `fix(gltf): read a sampler's stride equal to its element size`.

A change of results (a pose's bits, a compressed clip's bytes, a format version) says so in the pull request and the changelog, since hosts pin hashes and store built files. A change meant to affect speed or size gives instruction counts before and after on the same machine.
