# Size and performance

The library is held to measured targets against the best open alternatives: ACL for compressed size, compression time and sampling, ozz-animation for blending, the model pass and code size, and no allocation at run time. Each is reported by a tool or a scheduled job, never asserted. This page lists how each is measured and where it stands.

## Code size

`tools/size_report.py` reads a runtime-only archive (`MAUL_ANIM_OFFLINE=OFF`) with `size(1)` and groups its objects into the parts `tools/size-budget.txt` names: core, skinning, graph, IK, full-body IK, springs, ragdoll, retargeting, mirroring, matching, learned matching, morph targets and the elementary functions. The core takes its listed modules and every module they use that no other part lists; a module no part takes is reported as unplaced and fails, so a new module is placed when it lands.

Each part has a ceiling at `-O2` and `-O3` (GCC 14, x86-64), for the scalar and the AVX2 archive: its measure rounded up to the next thousand bytes, which is the tolerance a change has before the ceiling must move. A ceiling is lowered when a part shrinks past a thousand bytes and raised only by a pull request that names what the bytes buy; the scalar and vector backends do not take different algorithms to fit one.

The core is the like-for-like measure against ozz-animation's runtime (its animation and base libraries, 69,698 bytes of text at `-O2` and 74,225 at `-O3`, from `tools/ozz_runtime_size.sh`): sampling, blending, the model pass, skeletons and root motion. Skinning is a part of its own, as ozz's skinning job lives in its geometry library. At `-O2` the core stays under ozz's size; at `-O3` the compiler unrolls and inlines the eight-lane loops and the core grows past it, buying fewer instructions in blending and the model pass. Both are held: a host that needs the smallest runtime builds with `-O2`.

## No allocation, by symbols

The same tool fails when the runtime archive references any function that returns memory to free: the C and POSIX allocators, functions that allocate a result (`strdup`, `asprintf`, `getline` and the rest), and the Windows CRT's, as ELF, COFF and Mach-O spell them. This covers every runtime path, where a counting allocator in a test covers only the calls a test makes. CI runs it on GCC and Clang archives on Linux and the clang-cl archive on Windows.

## Speed

Timings are never a CI gate: wall time is not repeatable on shared runners. A change meant to affect performance is measured before and after on one machine, by instruction count first (which the machine's load does not change) and by time in a quiet window where the two disagree; the pull request gives both numbers.

Where the targets stand, measured on main by the programs named, on an AMD Ryzen 9 5900X (24 logical CPUs, Linux 6.8) at a load average of about 6 to 9: the differentials built with the `linux-differential` preset (Clang 20, Release), on ozz-animation's Quaternius file (53 bones, 46 clips, 67.7 s). Instructions are user instructions a call from the kernel's counter, the difference of two runs, which the load does not change; the nanoseconds beside them are the best run's and move with the load. A target not met links the issue that pursues it.

| Target | Reference | Maul Anim | Reference's | Status | Program |
|---|---|---|---|---|---|
| Compressed size | ACL at its default settings, 0.1 mm | 284,288 bytes, largest error 0.0999 mm | 265,493 bytes, 0.3061 mm | met: 1.071 of ACL's bytes at a third of its largest error | `acl_differential` |
| Compression time | ACL | 0.18 s | 1.22 s | met: 0.15 of ACL's time | `acl_differential` |
| Random access sampling | ACL's decompression | 10,115 (680 ns) a pose | 7,790 (523 ns) | not met: 1.30 times ACL's instructions ([#39](https://github.com/Rawframe-Project/maul-anim/issues/39)) | `acl_differential`, loops 2 and 0 |
| Forward playback | ACL's forward decompression | 2,589 (193 ns) a pose with the group cursor; 5,809 (392 ns) at full precision | 7,586 (510 ns) | met: 0.34 times ACL's instructions | `acl_differential`, loops 3, 4 and 1 |
| Two-pose blend | ozz-animation's `BlendingJob` | 2,837 (210 ns) | 2,055 (130 ns) | not met: 1.38 times ozz's instructions ([#40](https://github.com/Rawframe-Project/maul-anim/issues/40)) | `ozz_differential`, sides 1 and 0 |
| Model pass | ozz-animation's `LocalToModelJob` | 6,502 (553 ns) | 5,239 (323 ns) | not met: 1.24 times ozz's instructions ([#41](https://github.com/Rawframe-Project/maul-anim/issues/41)) | `ozz_differential`, sides 3 and 2 |
| Two-bone IK | ozz-animation's `IKTwoBoneJob`, its whole cost to a host | 2,475 (365 ns) | 967 (169 ns) | not met: 2.6 times ozz's instructions ([#42](https://github.com/Rawframe-Project/maul-anim/issues/42)) | `ik_differential`, loops 0 and 2 |
| Aim IK | ozz-animation's `IKAimJob`, its whole cost to a host | 2,042 (293 ns) | 571 (105 ns) | not met: 3.6 times ozz's instructions ([#42](https://github.com/Rawframe-Project/maul-anim/issues/42)) | `ik_differential`, loops 3 and 5 |
| CCD and FABRIK | Godot's chain solvers | 17,533 and 17,363 (2,584 and 2,894 ns) a solve of 3 bones, 16 iterations at most | none run | not measured ([#35](https://github.com/Rawframe-Project/maul-anim/issues/35)) | `ik_differential`, loops 6 and 7 |
| Motion matching search | Holden's search | | | an exact k-d tree with his picks: 1.1 to 16.5 times fewer instructions than his bounds over runs from about 20,000 frames up, up to 27% more at 5,000 frames with 45 features ([motion-matching.md](motion-matching.md#the-search)) | `bench/bench_match.c` |
| Spring bones | Godot's spring bone simulator | | | not measured ([#34](https://github.com/Rawframe-Project/maul-anim/issues/34)) | none yet |
| Draco and meshopt decoding | Google's and meshoptimizer's scalar decoders | | | not measured ([#36](https://github.com/Rawframe-Project/maul-anim/issues/36)) | none yet |
| Opening glTF and FBX | cgltf, ufbx | | | not measured ([#37](https://github.com/Rawframe-Project/maul-anim/issues/37), [#38](https://github.com/Rawframe-Project/maul-anim/issues/38)) | none yet |

ozz-animation's IK jobs work on model matrices and return corrections, where `maniSolveTwoBone` and `maniSolveAim` start from the pose's locals and write them back; so ozz's whole cost to a host that holds the same locals is the chain's model pass with ozz's affine maths, the job, and the corrections multiplied into the locals. Its jobs alone take 702 and 309 instructions a solve (`ik_differential`, loops 1 and 4).

The differentials in `tools/` time both sides on the same data ([development.md](../operations/development.md#differentials)); the benchmark program in `bench/` times the library's own hot paths on fixed scenes, with their instruction counts ([development.md](../operations/development.md#benchmarks)).

## Graph overhead

A graph adds little to the work it orders: a frame of a graph that samples two compressed clips and blends them takes 27,497 instructions, 4.9% more than the same samples, root motion and blend made by hand at 26,213 (`tools/bench.sh`, `graph.frame` against `graph.byhand`, Clang 20); the target is within 10%. A node kind's cost is its work (samples, blends, solves) plus a small update; state machines cost about their weighted states.
