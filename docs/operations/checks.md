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

`.github/workflows/ci.yml` runs on every pull request and on every push to `main`. The Linux jobs run in the organization's CI image, pinned by digest, and install nothing. The ruleset requires only its `required` job, which passes when every other job passed or was skipped, and the title check.

| Job | What it guards |
|---|---|
| `changes` | Whether a pull request changes anything beyond documentation; when it does not, the build jobs below are skipped. |
| `docs` | The organization's documentation check: allowed files, the em dash, record numbers, page sizes, links. `test/data/` is excluded: it holds other projects' sample files with their own READMEs. |
| `zizmor` | The workflows: injection, persisted credentials, permissions, unpinned actions. |
| `checks` | `clang-format`, the family's source rules, the module graph, lengths, API documentation, documentation snippets, the generated sources and test tables against their generators, that the documentation site builds, and clang-tidy. It runs on documentation changes too. |
| `build` | One call of the organization's `cpp.yml` per preset: GCC 14 (every part, as CodeQL builds it) on x86-64 and on arm64 with NEON, Clang 20 in Release, Clang with AddressSanitizer and UBSan (each test repeated up to three times on a pull request, so a flaky one fails before it merges), Clang with ThreadSanitizer, the shared library on the scalar backend with the package test, macOS, and `clang-cl` on Windows. All build with warnings as errors. The GCC and Clang Release cells also run an import under a 4 GB address space limit, which must fail with Capacity rather than be granted by overcommit. ccache is restored on every run and saved only from `main`. |
| `windows` | The public headers compiled by MSVC's `cl` as C17 and C++17 with `/W4 /WX`; the runtime alone under `clang-cl`, which must reference no allocator; and the DLL's package test. |
| `size` | The runtime's text by part at `-O2` and `-O3` with GCC 14, on the scalar backend and on AVX2, against `tools/size-budget.txt`, and no allocator symbol in the runtime under GCC 14 or Clang 20. |
| `web` | The library and its tests under Emscripten and Node, and the static library's size, reported. The image has no Emscripten, so this job uses the runner and the emsdk action. |
| `fuzz` | Every fuzz target, four at a time, from the corpus the nightly run grows and the committed seeds: 15 seconds each on a pull request, 60 on `main`. A finding uploads the logs and the input. |

`nightly.yml` is the extended run, not a required check. It runs the differentials against ufbx (under the sanitizers), ACL, ozz-animation and Godot, described in [development.md](development.md#differentials), and fuzzes each target for ten minutes from the corpus of the night before, then minimizes the corpus and saves it to the Actions cache for the next night and for the `fuzz` job.

`pr-title.yml` checks that the pull request title is a conventional commit with a scope from `AGENTS.md`, and `pr-gate.yml` closes a pull request from outside the organization unless it closes an issue labeled `accepted`. `codeql.yml` runs CodeQL on every pull request and on `main`, building every part with GCC 14 (an advanced setup, since the default one builds with the runner's GCC 13). `scorecard.yml` grades the repository's security practices weekly for the README badge, and Dependabot proposes updates of the pinned actions once a week in one pull request.

## Tests

Each suite in `test/` is one executable, registered with CTest under its name and labeled with it; `guide_import_memory_limit` runs `test_guide_import` under the address space limit, and `package` installs the build and builds `test/consumer` against it, by its CMake package and by pkg-config. The fixture suites read `test/data/`. The white-box suites include `src/` and are not built in shared builds. No test reports itself skipped; a test preset that finds no test fails, and so does a CI cell where a test is skipped or not run. `test_guide.c` and `test_guide_import.c` run the C snippets of `docs/user/` as written; `tools/check_guide.py` fails when a snippet and its copy differ.

## Fuzzing

Every reader of untrusted bytes has a fuzz target, `test/fuzz_<area>.c`: each file kind, the JSON forms, saved graph state, training checkpoints, and the glTF, meshopt, Draco and FBX importers. A target checks what must hold for any input: no crash, no broken assertion, and whatever opens can be used, sampled, evaluated or built, without either. Each target also retries the input with its header or hash made right, so mutations reach the checks behind them. `tools/fuzz.sh` runs them; it reads the committed seeds in `test/seeds/<target>/`, and for the importers the sample files of `test/data/`.

The seeds are valid files the test suites write: `./out/linux-clang-20-asan/test_clip --seed test/seeds/fuzz_clip/sample.clip`. After a format change, write them again so they open. An input the nightly corpus found that reaches new code may be committed as a seed when it is worth keeping. A crash a fuzzer finds becomes a test in the suite of its module (see [development.md](development.md#reproducing-a-fuzz-finding)).

## Budgets

Over a budget, tests are pruned or merged before anything else is added. Golden hashes pin a few scenes; the large sweeps run in the nightly or not at all.
