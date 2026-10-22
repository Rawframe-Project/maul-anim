# mani-0038. The SIMD backend and the CPU check

Status: Accepted

## Context

Playback and the blend of two poses work on groups of eight bones, and
four-lane SSE takes two operations a row of such a group where eight
lanes take one. The family's `src/simd.h` gives eight
lanes of binary32 as AVX2 on x86-64, two NEON registers on arm64 and a
scalar fallback elsewhere, and its `src/cpu.c` tells a program which of
them a build uses and whether the CPU runs it, without printing and
without a world. An AVX2 build stops on its first vector instruction on
a CPU without AVX2 and FMA, so something must check before that.

Maul Anim has no world and no call that every use passes: a host opens
skeletons and clips, builds graphs and calls jobs on its own arrays,
in any order. Maul3D, which has a world, checks when the world is
created.

## Decision

- **The build option `MAUL_ANIM_SIMD`,** `auto` or `scalar`, as
  Maul3D's: `auto` builds x86-64 for AVX2 and FMA (`-mavx2 -mfma`,
  `/arch:AVX2` under MSVC), arm64 for NEON, which every arm64 CPU has,
  and other targets, the web among them, scalar; `scalar` builds the
  portable code everywhere. Floating-point contraction stays off under
  every backend, so the compiler fuses no multiply and add the source
  did not ask for.
- **The check is the host's, once.** The public `maniCheckCpu` names
  the backend and returns 1 when this CPU runs it, 0 when it does not;
  `maniGetSimdBackend` and `maniCpuSupportsBackend` give the two parts.
  The three are compiled without the backend's flags, so they run on
  any CPU of the architecture. A host calls `maniCheckCpu` before any
  other call and, on 0, refuses to run or loads a scalar build. No job
  checks again: a check on every job would cost a call each time and
  make jobs that cannot fail return a status.
- **Same bits.** A vector kernel uses the header's separate multiply
  and add wherever its scalar code does, never the fused one, and a
  test compares each vector kernel's output with its scalar code's bit
  for bit on every CI architecture and in a scalar build (CI's shared
  build).

## Consequences

A program built from source on x86-64 needs a CPU from 2013 or later
(Intel Haswell, AMD Excavator) unless it asks for `scalar`; a package
meant for older machines builds `scalar`. Results do not depend on the
backend, so a scalar build and an AVX2 build of one program replay
each other's saved states and hashes. The suites assert that the
backend named is the one the build chose and that the machines running
them run it.

The size budget (mani-0036) holds the scalar backend's archives, the
build its ceilings and the comparison with ozz-animation were measured
on; the AVX2 archives are reported beside them. Before any vector
kernel, `-mavx2` alone left the `-O3` runtime's text about the same
(400,163 bytes scalar, 400,011 AVX2) but moved it between parts, GCC
vectorizing some scalar loops and dropping others: matching grew by
4,046 bytes and learned matching by 1,592 while the elementary
functions shrank by 976. A part's AVX2 ceiling is set when its kernels
land.
