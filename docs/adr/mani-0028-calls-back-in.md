# mani-0028. Calls back in from the host's code

Status: Accepted

## Context

Two owner objects run the host's code: a builder its allocator, in
every build, copy and clear and in the glTF and FBX documents opened
with it, which share it; a learned trainer its allocator and its task
hooks, whose `enqueueTask` and `finishTask` run inside training.
Nothing said whether that code may call a function of the object it
serves, and nothing stopped it: an allocator destroying its builder in
the middle of a build, or a task hook training the trainer it is
training, corrupts the object without a sign. The family's rule
(conventions section 10) is that such code calls no function of the
object it serves, and that the object refuses one that does.

Box2D v3 locks its world for a step and refuses every change while
locked; GLFW documents on its allocator that it calls no GLFW function
and checks nothing; Tween (mtwn-0068) marks its context busy for each
of its calls.

## Decision

- **Marked while the host's code runs:** an object keeps the host's
  allocator and task hooks behind the library's own functions, which
  mark the object for the host's call and restore the mark after. Every
  function taking a builder, closing a document opened with one, and
  training, restoring or destroying a trainer refuses with
  `mani_errorState` (a new result) while its object is marked, and
  counts one misuse; a destroy or close from there leaves the object as
  it was. Functions reading an object const are not refused.
- **Marked around the host's code, not the whole call:** the library's
  own functions call each other (an importer builds through
  `maniBuildSkeleton`, a JSON graph through `maniBuildGraph`), which a
  mark for the whole call would refuse; the mark sits exactly where the
  host's code runs, so no list of which calls reach it is kept.
- **Counted atomically:** the mark and the misuse count are atomic
  (relaxed), since a task hook's worker may call back in while the
  training thread waits. A trainer's count is read with
  `maniLearnedTrainerMisuses`; a builder's with `maniBuilderMisuses`.
- **Stated** on `maniAllocator`, `maniTaskHooks`, the builder and the
  trainer.

## Consequences

A call back in is a counted refusal, not a corruption. `test_reentry`
calls every function taking a builder from the builder's allocator as
it allocates and as it frees inside a build, each with arguments
refused otherwise as invalid so the state error is the call back in
alone, and destroys the builder and closes a glTF and an FBX document
from there: each refused and counted once, the build's bytes as
without it. A trainer's allocator, its `enqueueTask` and its
`finishTask` train, restore and destroy it: each refused and counted
once, the training going on. The cost is a load and two stores around
each call of the host's code, and an atomic load at each guarded
function's entry.
