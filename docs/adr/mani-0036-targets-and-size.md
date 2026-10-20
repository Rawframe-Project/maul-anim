# mani-0036. The targets at release and the size budget

Status: Accepted

## Context

The library profile (mani-0001) holds Maul Anim to measured targets
against the best open rivals: ACL for compressed size, compression time
and sampling, ozz-animation for blending, the model pass and code size,
and no allocation at runtime. Each is reported by a benchmark or a
differential job; a missed one has to be closed or moved by a recorded
decision before a release. With every part of the scope built, the
targets were measured together.

ozz-animation 0.17.0's runtime (its animation and base libraries,
every job and the archive reader) is 69,698 bytes of text at `-O2` and
74,225 at `-O3`, its own Release (GCC 14, x86-64).

## Decision

- **Where the targets stand.** Compressed clips: 4,197 bytes a second
  at a largest error of 0.0999 mm, against ACL's 3,920 at 0.306 mm on
  the same 67.7 s (1.071 of its bytes at a third of its error), met.
  Compression: 0.23 s against ACL's 1.73 s, met. No allocation at
  runtime: met (below). A pose at a random clip and time and forward
  playback (2.4 and 2.0 times ACL's instructions), the blend of two
  poses (268 ns against ozz's 128 on CI's runner) and the model pass
  (about 1,100 ns against ozz's 588) are not met: they wait on vector
  code through the family's eight-lane header, which keeps the scalar
  and vector results the same bits. They stay targets for 1.0.0; each
  release before it publishes their measured ratios in its notes.
- **The size budget.** `tools/size_report.py` reads a runtime-only
  archive (`MAUL_ANIM_OFFLINE` off) with `size(1)` and groups its
  objects into the parts `tools/size-budget.txt` lists: the core
  (sampling, blending, the model pass and palettes, with skeletons,
  skinning and root motion, and every module they use), the graph, IK,
  full-body IK, springs, the ragdoll seam, retargeting, mirroring,
  motion matching, learned matching, morph targets and the binary32
  elementary functions. A module no part takes is reported as unplaced.
  Each part has a ceiling at `-O2` and at `-O3` (GCC 14): its measure
  rounded up to the next thousand bytes, the core's at `-O2` ozz's own
  69,698. CI's size job reports both levels on every push; `--check`
  holds them at release. A ceiling is lowered when a part shrinks past
  a thousand bytes and raised only by a decision recorded here.
- **The core against ozz.** At `-O2` the core is 69,357 bytes, under
  ozz's 69,698. At `-O3` it is 91,414 against 74,225: the compiler
  unrolls and inlines the eight-lane loops of blending, skinning and
  compressed sampling, and that code buys 18% fewer instructions in a
  two-pose blend (8,826 against 10,749) and 12% in the model pass
  (11,957 against 13,563) on 53 bones. The trade is kept: a host that
  needs the smallest runtime builds with `-O2` and gets ozz's size, one
  that needs speed gets `-O3`'s; both are held by ceilings.
- **No allocation, by symbols.** The same tool fails when the runtime
  archive names any function that returns memory to free: the C and
  POSIX allocators (`malloc`, `calloc`, `realloc`, `reallocarray`,
  `free`, `aligned_alloc`, `posix_memalign`, `memalign`, `valloc`,
  `pvalloc`), those that allocate a result (`strdup`, `strndup`,
  `wcsdup`, `asprintf`, `vasprintf`, `getline`, `getdelim`,
  `open_memstream`), and the Windows CRT's (`_aligned_malloc`,
  `_aligned_realloc`, `_aligned_recalloc`, `_aligned_free`,
  `_aligned_offset_malloc`, `_recalloc`, `_expand`, `_malloca`,
  `_freea`, `_strdup`, `_wcsdup`, `_mbsdup`), read as ELF, COFF (with
  or without its `__imp_` prefix) and Mach-O spell them. CI runs it on
  the runtime archive of GCC 14 at `-O2` and `-O3` and of Clang 20 on
  Linux, and of clang-cl on Windows (`llvm-nm`); only the
  floating-point guard's register access differs by platform, and it
  calls nothing. This covers every runtime path, where a counting
  allocator in tests covers only the calls a test makes.

## Consequences

The release checklist (`docs/releasing.md`) runs the size check and the
differential jobs on the release commit and puts the table of targets
in the notes. Sizes at the time of this record (`-O2`, `-O3`): core
69,357 and 91,414; graph 100,842 and 124,321; IK 30,070 and 33,431;
full-body IK 25,565 and 31,983; springs 28,155 and 30,091; ragdoll
14,449 and 15,896; retargeting 11,637 and 12,764; mirroring 4,858 and
5,234; matching 15,910 and 18,294; learned matching 10,261 and 15,020;
morph targets 5,946 and 6,246; elementary functions 5,954 and 6,130;
in all 323,004 and 390,824.

Ceilings raised after 0.1.0, each for what it bought: matching's at
`-O2` to 17,000 (16,182: the search's exclusions as frame counts and
its end found by a guess settled by the same test, a tenth fewer
instructions a search, mani-0024); morph targets' at `-O2` to 7,000
(6,398: the three-float add written out and vertices copied through a
local, the job from 2.17 to 1.035 of a hand loop's instructions at
`-O2`, mani-0021); the graph's to 105,000 at `-O2` and 129,000 at `-O3`
(104,881 and 128,424: the turn limit node, mani-0037, about 2,600 bytes
at `-O2`, and the IK and full-body evaluation in a module of its own,
1,442, its small readers inlined there as well as in the evaluation),
then to 106,000 and 130,000 (105,844 and 129,578: the motion matching
node's continuing frame interpolated to its playhead and scored there,
mani-0024, about 850 bytes of the node's search at `-O2`, which moved
18% to 33% of settled picks off the pose on screen before), then to
107,000 and 131,000 (106,214 and 130,678: the learned node's
Decompressor hidden values kept from its update for its evaluation,
mani-0025, about 370 bytes at `-O2`, a tenth of a learned frame's
instructions).

Targets added after 0.1.0, for the importer's decoders, in instructions
at `-O2` and `-O3` (GCC 14) on the same streams with the same bits: the
Draco decoder (mani-0021) no more than Google's decoder (d429de8, its
Release build) on each skinned Khronos sample and each group of Google's
encoder's fixtures, met at 0.87 to 0.99 at `-O2` and 0.78 to 0.90 at
`-O3` (a symbol's search started from a coarse table of the precision,
the dequantization and the difference prediction's loops by kind); the
meshopt decoder (mani-0008) no more than meshoptimizer's scalar decoder
(f0a684f1, `MESHOPTIMIZER_NO_SIMD`, at its `-O2`) on each stream kind
gltfpack writes, met on attributes of both versions, index sequences and
the exponential filter (0.88 to 0.96 at `-O2`, 0.81 to 0.93 at `-O3`;
groups and channels decoded for their widths, a LEB128 value bounded
once), and over where the specification asks more: the octahedral
filter's stream at 1.13 and 1.02, the quaternion's at 1.05 and 0.98
(the formulas' divisions and rounding half away from zero, which keep
the bits the same everywhere, where meshoptimizer multiplies by a
reciprocal and adds a half), triangles at 1.04 and 0.99 (the check that
no FIFO entry is read before it is written, which meshoptimizer skips).
meshoptimizer's vector decoder is measured with the eight-lane work
above.

Targets added for IK (mani-0029), in instructions at `-O2` and `-O3`
(GCC 14) on the same solves with the same results: the two-bone and aim
solves no more than ozz's cost of the same solve to its host (its job
on model matrices, the corrections multiplied into its SoA locals, its
model pass again from the chain's start), and over: the two-bone solve
at 1.16 and 1.10 of it, the aim at 2.44 and 2.29. Composing the root
path's frames and reading the pose's locals inline was measured and not
kept: a twelfth fewer instructions cost a third more IK code with the
helpers inline wherever called, and 1,300 to 2,600 bytes past the
ceilings with them inline only in the root path, where the bytes buy
less than other parts' raises did. What remains is the binary64 arc and
normalizations that keep a turn exact near opposite directions (about a
fifth of the aim), the inputs' checks, and scalar against ozz's four
lanes and estimated reciprocals, measured again with the eight-lane work
above. The CCD and FABRIK chains no more than Godot 4.7.2's CCDIK3D and
FABRIK3D on the same chains and iterations, met at 0.20 and 0.40 of its
instructions.
