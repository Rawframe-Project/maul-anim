# Overview

Maul Anim samples, blends and evaluates skeletal animation, solves inverse kinematics, and builds, compresses and imports the data it plays. It follows the family rules in [maul-conventions.md](maul-conventions.md); this page states what the library's domain adds to them and how the parts fit.

## Library profile

- **Determinism:** bit-exact. Identical input gives identical output on every platform, compiler, architecture and SIMD backend, for every output: local and model poses, palettes, skinned vertices, track values, events, root motion, instance state, IK results, retargeted and mirrored poses, compressed bytes, imported files, trained networks. Servers compute hit-test poses and root motion that clients must agree with, replays repeat motion from parameters alone, and tests pin results by hash. [determinism.md](determinism.md) explains how.
- **Threads:** none. Every function runs on the caller's thread. Opened files are immutable views, safe from any thread; a pose, a graph instance, a scratch block, a builder or a trainer is used by one thread at a time. The learned trainer hands work to the host's task hooks, and its results do not depend on how the host splits it.
- **Memory:** the runtime has no owner objects and allocates nothing. Its data are files whose bytes the caller holds, checked once when opened into fixed-size views; poses, instances, cursors and scratch are caller memory of the size and alignment the library reports. The offline side (builders, importers, the compressor, the trainer) has owner objects created with the family allocator. [memory-and-threads.md](memory-and-threads.md) explains the split.
- **Operating system:** no call. No file is opened and no clock is read; the host passes bytes and time.
- **Platform dependencies:** none beyond `<stddef.h>`, `<stdint.h>`, `<string.h>` and the exactly rounded `<math.h>` functions in `tools/libm-allowed.txt`.
- **Misuse and stale ids:** the library has no ids. Its owner objects count one misuse for each call a correct program never makes: a builder for a call back in from the host's code, a NULL argument, a bad option or an index past a count (`maniBuilderMisuses`), a trainer for a call back in (`maniLearnedTrainerMisuses`). Bad untrusted data (a file, JSON text, a saved state, a checkpoint) is refused with its status and location and counts none, so whoever controls the data cannot move the counter.
- **Root defs:** the builder and the learned trainer are the library's two creations, and neither takes an object another creation made (the trainer reads opened files), so both are roots and both defs carry `MANI_ABI_VERSION`. Files are opened from bytes without a def and carry their own format versions.

## The parts

The runtime is one static or shared library. Its parts, as the size report groups them (`tools/size-budget.txt`):

| Part | What it does | Page |
|---|---|---|
| core | skeletons, poses, clips (full and compressed), cursors, sampling, blending, masks, the model pass, root motion | [clips-and-compression.md](clips-and-compression.md) |
| skinning | skins, palettes, influences, the skinning jobs | [skinning.md](skinning.md) |
| morph | morph targets | [skinning.md](skinning.md) |
| graph | graphs, instances, every node kind | [graphs.md](graphs.md) |
| ik, bodyik | two-bone, aim, CCD, FABRIK, joint limits, full-body XPBD and SDLS | [ik.md](ik.md) |
| springs, ragdoll | spring bones and the seam to a physics engine | [physics-seam.md](physics-seam.md) |
| retarget, mirror | poses carried between skeletons and across a plane | [retargeting-and-mirroring.md](retargeting-and-mirroring.md) |
| match, learned | motion databases and their search, learned motion matching | [motion-matching.md](motion-matching.md) |
| elementary | the family's binary32 sine, cosine and arc functions | [determinism.md](determinism.md) |

The offline side, built when `MAUL_ANIM_OFFLINE` is on (the default), is part of the same library: the builder of every file kind, the JSON forms of graphs and maps, the clip compressor, the bakers of retargeted and mirrored clips, the database builder, the learned trainer, and the glTF and FBX importers ([gltf-import.md](gltf-import.md), [fbx-import.md](fbx-import.md)). A runtime-only build (`MAUL_ANIM_OFFLINE=OFF`) leaves all of it out.

## How data flows

```text
glTF / FBX bytes ──importer──┐
source structs ──────────────┼──builder──> file bytes (caller memory)
JSON text (graphs, maps) ────┘                  │
                                          maniOpen... (checked once)
                                                │
                                              views ──bind──> graph binding
                                                │                  │
                                    pose, cursor, scratch      instances
                                    (caller memory)            (caller memory)
```

Every asset kind shares one container ([file-formats.md](file-formats.md)). Files built against a skeleton record its identity, so a clip, skin, map or graph is refused with the wrong skeleton rather than played wrongly.

Within `src/`, each module is one `.c` file and its header; `tools/modules.txt` declares which module may include which and `tools/check_modules.py` enforces it. Builders sit beside the views they build (`clip_build.c` beside `clip.c`), and nothing in the runtime includes a builder.

## Hostile input and invariants

Every file, JSON text, glTF or FBX document, saved instance and argument is hostile: it is validated and refused with a status in every build. Opening a file checks every byte once, so later calls read without checks; a file that passes but was not made by the builder can give a wrong pose, never an unsafe access. A refusal names where: `maniFileResult.offset` for a file, `maniBuildResult` (`reason`, `item`, `other`) for a source, `maniImportResult` (reason, offset, item) for an import. One kind of bad data passes every check: a learned model's finite weights can carry a value past the float range at run time. The learned calls then return `mani_errorFormat` and write nothing; a graph's update and evaluation return it after completing, the node held at its last finite state and named by `maniInspectGraphNode`, every output finite. It is the one failure after which a call's outputs are valid.

Internal invariants are asserted with `MANI_ASSERT` ([`src/invariant.h`](../../src/invariant.h)): states that cannot occur unless the library is wrong, such as a parent composed after its child or a graph slot past the file's count. It traps in Debug, sanitizer and fuzz builds, where a broken invariant stops the run at the fault, and compiles out of Release builds, whose per-bone loops must not pay for it.

## Testing

Each suite in `test/` is one executable registered with CTest and labeled with its name. What the tests rely on:

- **References in binary64.** Sampling, blending, IK, retargeting, mirroring, springs and root motion are tested against straightforward binary64 implementations of the same rule, written in the test from the method's description.
- **Pinned hashes.** Sixteen golden values in sixteen suites hash scenes that together give every output promised above a check ([determinism.md](determinism.md) lists them); every platform and backend CI runs must give the same values.
- **Hostile files.** Every opener is tested with files changed at the byte each check reads, and fuzzed (`test/fuzz_*.c`).
- **Differentials.** Programs in `tools/` compare with ufbx, ACL, ozz-animation and Godot on their corpora; they run every night, not in the pull request check ([development.md](../operations/development.md#differentials)).
- **Guide snippets.** Every C block of `docs/user/` is built and run by `test/test_guide.c` or `test/test_guide_import.c`.
