# Determinism

Every output of the library is the same bits on every platform, compiler, architecture and SIMD backend the library builds for. A server and a client compute the same hit-test pose and the same root motion from the same parameters, a replay repeats from parameters alone, a compressed clip is the same bytes on every build machine, and a test can pin a result by hash. This page explains what holds that promise and what breaks it.

## Arithmetic

Computation is IEEE binary32, with binary64 where a step needs the digits (builders, decompositions, arcs between rotations). What keeps it reproducible:

- **No fast math and no contraction.** `cmake/CompilerFlags.cmake` builds with `-ffp-contract=off -fno-fast-math -fno-unsafe-math-optimizations -fno-trapping-math` and stops at configure when the flags ask for fast math. A compiler may otherwise fuse a multiply and an add into one rounding on one target and not another.
- **A fixed order.** Every sum is written in the order its result depends on: layers in their order, bones in skeleton order, features in feature order. Nothing reassociates, and no estimate instruction (reciprocal or reciprocal square root approximations) is used.
- **Only exactly rounded `<math.h>`.** `tools/libm-allowed.txt` lists the C library functions the sources may call (square roots, rounding, `fmod`, `frexpf`, `ldexpf`, `copysignf`, absolute values); `tools/check_source.py` refuses any other. C libraries disagree in the last bit of `sinf`, `expf` and the rest.
- **The library's own elementary functions.** Sine, cosine, arc cosine and arc tangent in binary64 ([`src/scalar.c`](../../src/scalar.c)) for builders and decompositions; the family's binary32 exponential, logarithm, sine, cosine and arc functions ([`src/elementary_float.c`](../../src/elementary_float.c)) for runtime paths that vectorize. Each states its range and accuracy, held by `test_scalar` and `test_math_float` against the C library's binary64 functions.
- **`-fno-math-errno`.** Square roots may vectorize without a call to set `errno`; no value changes.

## The floating-point environment

A host may run its threads with flush-to-zero, denormals-are-zero or another rounding mode, which would change results. Each public call that computes floats enters the library's environment (round to nearest, denormals kept, no traps) and restores the host's on return ([`src/fpenv.h`](../../src/fpenv.h)).

The trap: compilers treat floating-point arithmetic as free of side effects, so they may move it across the writes to the control register. A guarded public call therefore does no arithmetic of its own between entering and leaving; it calls a worker marked `MANI_NOINLINE`, whose body cannot move out of the call. A new public function that computes floats follows the same shape. `test_fpenv` runs calls under a host's flush-to-zero and checks the bits.

## SIMD backends

`MAUL_ANIM_SIMD=auto` builds x86-64 for AVX2 and FMA and arm64 for NEON through the family's eight-lane header ([`src/simd.h`](../../src/simd.h)); other targets, the web among them, and `MAUL_ANIM_SIMD=scalar` build the portable code. Results do not depend on the backend:

- A vector kernel uses the header's separate multiply and add wherever its scalar code does, never the fused operation, so each lane rounds as the scalar code does.
- Kernels with a scalar twin are tested against it bit for bit (`test_blend_lanes`, the clip suites' cursor comparisons) on AVX2, NEON and the scalar build CI runs.
- A scalar build and an AVX2 build of one program replay each other's saved instances and hashes.

An AVX2 build stops on its first vector instruction on a CPU without AVX2 and FMA. The library has no call every program passes through, so the check is the host's: `maniCheckCpu` names the backend and says whether this CPU runs it. It and its two parts (`maniGetSimdBackend`, `maniCpuSupportsBackend`) are compiled without the backend's flags ([`src/cpu.c`](../../src/cpu.c)), so they run on any CPU of the architecture. No job checks again: a check per job would cost a call each time and turn jobs that cannot fail into calls that return a status. Configure prints the backend, and the installed CMake package (`maul-anim_SIMD_BACKEND`) and pkg-config description name it, so a packager sees that an AVX2 build is not portable.

## What is covered

Deterministic results are pinned by a few golden scenes rather than by every test. Each scene hashes several outputs together, so every output the overview promises has a pinned check: every vector and quaternion operation over a fixed sequence (`test_vecmath`); the elementary functions (`test_scalar`); affine decomposition (`test_decompose`); the skeleton file and identity (`test_skeleton`); a blend, the same blend added back as a difference, its model pass and its mirror (`test_blend`); a clip with root motion, sampled across its loop for poses, float tracks and root motion, and its file (`test_clip`); a compressed clip's bytes (`test_clip_compress`); a graph file and an instance of it stepped across its clips' wraps, with each step's pose, root motion and events and the saved state (`test_graph`); a pose's skinning palette, morph targets, linear, dual quaternion and blended skinning, and a centers file and job (`test_skinning`); the SDLS body solver in two settings and the two-bone, aim, FABRIK and CCD solvers (`test_body_sdls`); the XPBD body solver in two settings (`test_body_xpbd`); a retargeted pose (`test_retarget_pose`); every file built from the glTF and FBX fixtures (`test_gltf_fixtures`, `test_fbx_fixtures`); meshopt's filters (`test_gltf_meshopt`); and a small network trained to the library's sine (`test_train`). Inputs a scene draws from the C library would differ by platform, so the scenes use the library's own functions. A change that alters any of them changes results for every user and names that in its pull request.

## Traps

- A new `<math.h>` call fails `check_source.py` unless it is exactly rounded; add it to `tools/libm-allowed.txt` only if it is.
- A quaternion's sign is a choice: blends, cursors and builders turn each rotation into the hemisphere of the one before by a dot product in a fixed order. Changing which value is compared changes bits without changing the rotation.
- `-0` and `+0` compare equal but hash differently. Caches that compare state (the learned node's) compare bytes, not floats.
