# Size and performance

The library is held to measured targets against the best open alternatives: ACL for compressed size, compression time and sampling, ozz-animation for blending, the model pass and code size, and no allocation at run time. Each is reported by a tool or a scheduled job, never asserted. This page lists how each is measured and where it stands.

## Code size

`tools/size_report.py` reads a runtime-only archive (`MAUL_ANIM_OFFLINE=OFF`) with `size(1)` and groups its objects into the parts `tools/size-budget.txt` names: core, skinning, graph, IK, full-body IK, springs, ragdoll, retargeting, mirroring, matching, learned matching, morph targets and the elementary functions. The core takes its listed modules and every module they use that no other part lists; a module no part takes is reported as unplaced and fails, so a new module is placed when it lands.

Each part has a ceiling at `-O2` and `-O3` (GCC 14, x86-64), for the scalar and the AVX2 archive: its measure rounded up to the next thousand bytes, which is the tolerance a change has before the ceiling must move. A ceiling is lowered when a part shrinks past a thousand bytes and raised only by a pull request that names what the bytes buy; the scalar and vector backends do not take different algorithms to fit one.

The core is the like-for-like measure against ozz-animation's runtime (its animation and base libraries, 69,698 bytes of text at `-O2`): sampling, blending, the model pass, skeletons and root motion. Skinning is a part of its own, as ozz's skinning job lives in its geometry library. At `-O2` the core stays under ozz's size; at `-O3` the compiler unrolls and inlines the eight-lane loops and the core grows past it, buying fewer instructions in blending and the model pass. Both are held: a host that needs the smallest runtime builds with `-O2`.

## No allocation, by symbols

The same tool fails when the runtime archive references any function that returns memory to free: the C and POSIX allocators, functions that allocate a result (`strdup`, `asprintf`, `getline` and the rest), and the Windows CRT's, as ELF, COFF and Mach-O spell them. This covers every runtime path, where a counting allocator in a test covers only the calls a test makes. CI runs it on GCC and Clang archives on Linux and the clang-cl archive on Windows.

## Speed

Timings are never a CI gate: wall time is not repeatable on shared runners. A change meant to affect performance is measured before and after on one machine, by instruction count first (which the machine's load does not change) and by time in a quiet window where the two disagree; the pull request gives both numbers.

Where the targets stand:

| Target | Reference | Status |
|---|---|---|
| Compressed size | ACL, Quaternius clips at its default precision | met: about 1.07 of ACL's bytes at a third of its largest error |
| Compression time | ACL | met: several times faster |
| Random access sampling | ACL's decompression | not met: about 1.3 times ACL's instructions (10,062 a pose against 7,790 on the Quaternius clips; 14,001 before sub-tracks were grouped by kind); the bits are read in scalar code, as the SIMD header has no integer lanes |
| Forward playback | ACL's forward decompression | met with the group cursor (about a third of ACL's instructions: 2,532 a pose against 7,586); not met by the compact cursor |
| Two-pose blend, model pass | ozz-animation | not met: the eight-lane kernels still take more instructions than ozz's four lanes with estimated reciprocals; timed runs pending |
| Two-bone and aim IK | ozz-animation's whole cost to a host | about 1.1 and 2.3 times |
| CCD and FABRIK | Godot's chain solvers | met at a fraction of its instructions |
| Motion matching search | Holden's search | within a few percent, with the same picks |
| Spring bones | Godot's spring bone simulator | met |
| Draco and meshopt decoding | Google's and meshoptimizer's scalar decoders | met, but for two meshopt filters where the specification's exact rounding costs a few percent |
| Opening glTF and FBX | cgltf, ufbx | met |

The differentials in `tools/` time both sides on the same data ([development.md](../operations/development.md#differentials)); the benchmark program in `bench/` times the library's own hot paths on fixed scenes, with their instruction counts ([development.md](../operations/development.md#benchmarks)).

## Graph overhead

A graph adds little to the work it orders: two clips and a blend through a graph take about 6% more instructions than the same samples and blend by hand, held within 10%. A node kind's cost is its work (samples, blends, solves) plus a small update; state machines cost about their weighted states.
