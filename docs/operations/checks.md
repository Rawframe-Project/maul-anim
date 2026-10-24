# Checks

What verifies a change, locally and in CI, and the time each may take.

## Locally

`tools/check.sh` runs the checks in three tiers; each includes the one before it. Its builds are the CI cells' presets from `CMakePresets.json`, and it runs in the organization's CI image as it is.

| Tier | Runs | Budget |
|---|---|---|
| `changed` | The source, module, length, API documentation and snippet rules, the organization's documentation check, `clang-format` on the changed C files, and the `linux-clang-20-asan` preset (assertions on): the suites of the changed test files, or every suite when library code, a header or the build changed. | under 2 minutes |
| `pr` | Every rule on every file, the generated files against their generators, the documentation site, clang-tidy, the `linux-clang-20-asan` and `linux-gcc-14` presets, and 10 seconds of fuzzing per target. | under 10 minutes |
| `full` | Also ThreadSanitizer, the shared library with the package test, Clang Release, the runtime's size against its budget and its allocator symbols, and 60 seconds per fuzz target. | under 20 minutes |

Run `changed` before every push. On the shared development machine, run `pr` and `full` through the workspace's heavy-job wrapper. Builds go to `out/<preset>` and the other trees to `out/check-*`, each with its log beside it.

## In CI

`.github/workflows/ci.yml` runs on every pull request, on every push to `main` and every night at 06:07 UTC, the repository's slot in the organization's schedule. The Linux jobs run in the organization's CI image, pinned by digest, and install nothing. The ruleset requires only its `required` job, which passes when every other job passed or was skipped, and the title check.

| Job | What it guards |
|---|---|
| `changes` | Whether a pull request changes anything beyond documentation; when it does not, the build jobs below are skipped. |
| `docs` | The organization's documentation check: allowed files, the em dash, record numbers, page sizes, links. `test/data/` is excluded as vendored: it holds other projects' sample files (Khronos glTF models, Draco streams, FBX scenes) kept as published, with their sources' READMEs. |
| `zizmor` | The workflows: injection, persisted credentials, permissions, unpinned actions. |
| `checks` | `clang-format`, the family's source rules, the module graph, lengths, API documentation, the floating-point environment rule (`tools/check_float_env.py`: every public function enters the environment or is listed, with its reason, in `tools/float-env-allowed.txt`), documentation snippets, the generated sources and test tables against their generators, that the documentation site builds, and clang-tidy. It runs on documentation changes too. |
| `build` | One call of the organization's `cpp.yml` per preset: GCC 14 (every part, as CodeQL builds it) on x86-64 and on arm64 with NEON, Clang 20 in Release, Clang with AddressSanitizer and UBSan (each test repeated up to three times on a pull request, so a flaky one fails before it merges), Clang with ThreadSanitizer, the shared library on the scalar backend with the package test, macOS, and `clang-cl` on Windows. All build with warnings as errors. The GCC and Clang Release cells also run an import under a 4 GB address space limit, which must fail with Capacity rather than be granted by overcommit. ccache is restored on every run and saved only from `main`. |
| `windows` | The public headers compiled by MSVC's `cl` as C17 and C++17 with `/W4 /WX`; the runtime alone under `clang-cl`, which must reference no allocator; and the DLL's package test. |
| `size` | The runtime's text by part at `-O2` and `-O3` with GCC 14, on the scalar backend and on AVX2, against `tools/size-budget.txt`, and no allocator symbol in the runtime under GCC 14 or Clang 20. |
| `web` | The library and its tests under Emscripten and Node, and the static library's size, reported. The image has no Emscripten, so this job uses the runner and the emsdk action. |
| `fuzz` | Every fuzz target, four at a time, from the committed seeds: 15 seconds each on a pull request, 60 on `main` and at night. A finding uploads the logs and the input. |

`nightly.yml` is the extended run, not a required check, starting at 06:22 UTC, fifteen minutes after the nightly `ci`. It runs the differentials against ufbx (under the sanitizers), ACL, ozz-animation and Godot, described in [development.md](development.md#differentials), and fuzzes each target for ten minutes in a job of its own, from its corpus of the nights before. Each target's corpus is an Actions cache of its own, keyed by the target, the fuzz build's inputs and the compiler, falling back to the target's latest corpus of an earlier build; after the run, libFuzzer's merge minimizes the seeds and the corpus together and the result is saved, also after a finding. The run's summary gives each target's seed and corpus coverage, and the corpus is uploaded for a week.

`pr-title.yml` checks that the pull request title is a conventional commit with a scope from `AGENTS.md`, and `pr-gate.yml` closes a pull request from outside the organization unless it closes an issue labeled `accepted`. `codeql.yml` runs CodeQL on every pull request and on `main`, building every part with GCC 14 (an advanced setup, since the default one builds with the runner's GCC 13). `scorecard.yml` grades the repository's security practices weekly for the README badge, and Dependabot proposes updates of the pinned actions once a week in one pull request.

Only the `build` cells restore a ccache. The jobs outside `cpp.yml` do not: a cache would save seconds in jobs that never set the run's length, or nothing at all. Measured on three runs (a pull request and two pushes to `main`), with `cpp.yml`'s restore taking 0 to 1 second a step:

- `size` builds its five runtimes in 15 to 26 seconds in all and finishes well before `web` and `fuzz`. `fuzz` builds every target in 10 to 14 seconds and then fuzzes for 97 to 367.
- `windows` takes 15 to 41 seconds a step, configuring included, for the header check, the runtime and the DLL.
- `web` builds in 140 to 184 seconds, of which about 30 are compiling. The rest is linking the 126 executables (124 suites and the two benchmarks), which ccache does not cache.
- The nightly run builds the differentials in 36 seconds, the ufbx differential in 28 and each fuzz target in 6 to 11, within its two hours.

CodeQL must never build through ccache: a cache hit skips the compiler, so CodeQL's tracer would not see that code.

## Tests

Each suite in `test/` is one executable, registered with CTest under its name and labeled with it; `guide_import_memory_limit` runs `test_guide_import` under the address space limit, and `package` installs the build and builds `test/consumer` against it, by its CMake package and by pkg-config. The fixture suites read `test/data/`. The white-box suites include `src/` and are not built in shared builds. No test reports itself skipped; a test preset that finds no test fails, and so does a CI cell where a test is skipped or not run. `test_guide.c` and `test_guide_import.c` run the C snippets of `docs/user/` as written; `tools/check_guide.py` fails when a snippet and its copy differ.

## Fuzzing

Every reader of untrusted bytes has a fuzz target, `test/fuzz_<area>.c`: each file kind, the JSON forms, saved graph state, training checkpoints, and the glTF, meshopt, Draco and FBX importers. A target checks what must hold for any input: no crash, no broken assertion, and whatever opens can be used, sampled, evaluated or built, without either. Each target also retries the input with its header or hash made right, so mutations reach the checks behind them. `tools/fuzz.sh run` runs them and `tools/fuzz.sh merge` minimizes their corpora; a target reads the committed seeds in `test/seeds/<target>/`, and for the importers the sample files of `test/data/`.

The seeds are valid files the test suites write: `./out/linux-clang-20-asan/test_clip --seed test/seeds/fuzz_clip/sample.clip`. After a format change, write them again so they open. An input the nightly corpus found that reaches new code may be committed as a seed when it is worth keeping. A crash a fuzzer finds becomes a test in the suite of its module (see [development.md](development.md#reproducing-a-fuzz-finding)).

## Budgets

Over a budget, tests are pruned or merged before anything else is added. Golden hashes pin a few scenes; the large sweeps run in the nightly or not at all.
