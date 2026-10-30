# Memory and threads

The runtime allocates nothing and starts no thread; the offline side allocates through the host's allocator and may run work on the host's task system. This page explains where each byte lives, who may touch it from which thread, and the one guard against the host's code calling back in.

## Files are views

Every asset (skeleton, clip, skin, influences, morph targets, graph, retargeting and mirror maps, motion database, learned model, spring set, ragdoll profile) is a file whose bytes the host holds, at a 16-byte aligned address, for as long as anything reads it. `maniOpen...` checks every byte once ([file-formats.md](file-formats.md)) and fills a small fixed-size view struct with pointers into those bytes; the view then reads without checks and never copies. Nothing converts a file on load, so a file may be mapped from disk and shared read-only by every thread and every character.

## Working memory is the caller's

Everything that changes per character or per thread lives in caller memory, sized by a query and aligned to a stated constant:

| What | Size | Alignment |
|---|---|---|
| A pose | `maniSkeleton_GetPoseSize` | `MANI_POSE_ALIGNMENT` (32) |
| A clip cursor | `maniClip_GetCursorSize`, or `maniClip_GetCursorCompactSize` | `MANI_CLIP_CURSOR_ALIGNMENT` |
| A graph binding (one per graph and asset set) | `maniGraph_GetBindingSize` | `MANI_GRAPH_BINDING_ALIGNMENT` |
| A graph instance (one per character) | `maniGraphBinding_GetInstanceSize` | `MANI_GRAPH_INSTANCE_ALIGNMENT` |
| A graph's evaluation scratch (one per thread) | `maniGraphBinding_GetScratchSize` | `MANI_GRAPH_SCRATCH_ALIGNMENT` (32) |
| A subset plan | `maniGraphBinding_GetSubsetSize` | `MANI_GRAPH_SUBSET_ALIGNMENT` |
| A spring state | `maniSpringSet_GetStateSize` | `MANI_SPRING_STATE_ALIGNMENT` |
| Scratch for retargeting and full-body IK | `maniRetargetMap_GetScratchSize`, `maniSkeleton_GetBodyIkScratchSize` | 16 |
| Scratch for a learned model's evaluation | `maniLearnedModel_GetScratchSize` | 4 |

Poses, instances, cursors and spring states hold no pointers: copying their bytes copies them, and they may move between threads. An instance is a block of 16-byte cells (one per parameter and per node: a weight, a word the node's kind keeps, a binary64 playhead), then a cursor per clip node, then each stateful node's block; the binding records where each block lies, so nothing points into the instance. That is what makes rollback a `memcpy`, and why `maniGraphInstance_Save` can write the state as plain little-endian words.

Scratch is the opposite: a pose per slot of the graph, room for one node's layers and, when the graph plays another skeleton's clips through a map, room to carry one clip's pose. It is never part of an instance's state, and one scratch per thread serves any number of instances, one at a time. A scratch remembers the binding it last served and redoes its setup when handed another; a test pins that a scratch shared by two graphs in turn gives each its own result.

A node that needs room only while it evaluates (full-body IK's effectors and solver scratch, a spring node's work area) takes it from the instance's room, sized at instancing from what the node's description needs, so evaluation never allocates.

## Saved state is external bytes

`maniGraphInstance_Save`, `maniSpringState_Save` and the learned trainer's checkpoints write state as bytes a host may store or send. Restoring treats them as hostile: every value is checked as setting it through the API would keep it (parameters in their bounds, quaternions unit within a stated tolerance, weights finite and not negative, playheads finite, a machine's state in range, a running transition one into that state). Cursors and caches are not saved; a restore starts them over, and nothing a pose depends on was cached in them.

## The offline side owns memory

A builder (`maniCreateBuilder`) and a learned trainer (`maniCreateLearnedTrainer`) are owner objects. Each takes an allocator in its def and uses it for every block, with the block's size and alignment, so the host's allocator is the import's memory budget: a block it refuses fails the call with `mani_errorCapacity` and nothing is kept. A builder keeps the last file or text it made until the next build; `maniBuilder_CopyBuilt` copies it into caller memory, which is where it is opened from. Documents opened with a builder (`maniOpenGltf`, `maniOpenFbx`) share its allocator.

The learned trainer splits each batch into fixed chunks and hands them to the host's task hooks (`maniTaskHooks`); gradients are summed in chunk order, so the result does not depend on the thread count or on which thread ran which chunk. Without hooks every chunk runs on the calling thread.

## Calls back in

The host's code runs inside library calls: a builder's allocator during every build, copy and close; a trainer's allocator and its task hooks during training. If that code calls a function of the object it serves (destroys the builder mid-build, trains the trainer it is training), the object would be corrupted without a sign.

So the object is marked while the host's code runs. The library calls the host's allocator and hooks only through its own wrappers ([`src/host_code.h`](../../src/host_code.h)), which set the mark before the call and restore it after. Every function that takes a builder, closes a document opened with one, or trains, restores or destroys a trainer refuses with `mani_errorBusy` while its object is marked, and counts one misuse (`maniBuilder_GetMisuse`, `maniLearnedTrainer_GetMisuse`). A destroy or close refused from there leaves the object as it was, and any call refused from there leaves its outputs as they were: they may be the ones the call in progress is filling, as a free hook that measures again into the report the outer measure has just written, and clearing them would wipe it. Every other failure of such a call still clears its outputs. Functions that read an object through a const pointer are not refused.

The mark sits exactly around the host's code, not around the whole public call, because the library's own functions call each other (an importer builds through `maniBuildSkeleton`, a JSON graph through `maniBuildGraph`); a mark for the whole call would refuse those. The mark and the count are relaxed atomics, since a task hook's worker may call back in while the training thread waits. `test_reentry` calls every builder function from the builder's allocator inside a build and every trainer entry from its hooks, and checks each is refused, counted once, and that the build's bytes are unchanged.

## Threads

| Object | Rule |
|---|---|
| Opened files and their views | read-only; any thread, any number at once |
| A graph binding | read-only after binding; shared by every instance |
| A pose, cursor, instance, spring state, subset plan | one thread at a time |
| A graph scratch | one thread at a time; any instance |
| A builder, a trainer | one thread at a time; the trainer's task hooks run its chunks on any thread |

Jobs over disjoint ranges (skinning over vertex ranges, graph evaluation of different instances with different scratch) may run on as many threads as the host likes. Each public function's documentation states its thread safety in the form `tools/check_docs.py` checks.
