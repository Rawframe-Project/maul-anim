# mani-0001. Library profile

Status: Accepted

## Context

Every Maul library states in one record what its domain adds to the
family rulebook (family record 0005). Maul Anim samples, blends and
evaluates skeletal animation, solves inverse kinematics and compresses
and imports clips. Servers compute hit-test poses and root motion that
clients must agree with, replays and lockstep games repeat motion from
parameters alone, and tests pin results by hash, so its results must
not depend on where it runs; and hosts evaluate many characters at
once on their own threads.

## Decision

- **Determinism:** bit-exact. Identical input gives identical output on
  every platform, compiler and architecture, for every output: local
  and model poses, skinning palettes, float and discrete track values,
  events and their weights, root motion, instance state, IK results,
  retargeted poses, compressed bytes and imported files. Computation is
  IEEE binary32 under the family's rules (no fast math, no implicit
  contraction, fused multiply-add only through the family SIMD header),
  with no estimate instructions. `tools/libm-allowed.txt` lists the
  only `<math.h>` functions the sources may call, all exactly rounded;
  sine, cosine and the arc functions are the library's own.
- **Threads:** none. Every function runs on the caller's thread.
  Assets are immutable views and safe from any thread; an instance, a
  scratch block and an offline builder are used by one thread at a
  time, and different ones on as many threads as the host likes.
- **Memory:** the runtime has no owner objects and allocates nothing.
  Skeletons, clips, graphs and retargeting maps are files whose bytes
  the caller holds, validated once when opened into fixed-size views.
  Instances, poses and per-thread scratch live in caller memory of the
  size and alignment the library reports. The offline part (import,
  building, compression) has owner objects created with the family
  allocator, which write their results into caller buffers.
- **Operating system:** the library makes no operating system call: no
  file is opened and no clock is read. The host passes bytes and time.
- **Platform dependencies:** none beyond the C standard library's
  `<stddef.h>`, `<stdint.h>`, `<string.h>` and the exactly rounded
  functions of `<math.h>`.
- **Commit areas:** `api`, `bench`, `blend`, `build`, `ci`, `clip`,
  `compress`, `docs`, `format`, `graph`, `ik`, `import`, `machine`,
  `match`, `math`, `pose`, `retarget`, `rootmotion`, `sample`,
  `skeleton`, `skin`, `tests`, `tools`, `transition`. `skin` (skins,
  influences, skinning and morph targets, mani-0021) joined when that
  scope's work began; `match` (motion matching, mani-0024) likewise.

## Consequences

A server and a client compute the same pose and root motion from the
same parameters, tests pin results by hash on every platform, a
compressed clip is the same bytes on every build machine, and the host
decides where every byte lives and which thread does what.
