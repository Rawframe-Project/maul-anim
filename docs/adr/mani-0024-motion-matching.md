# mani-0024. Motion matching

Status: Accepted

## Context

Motion matching plays a character from a database of its clips: every
so often it searches for the frame whose pose and root path best match
what the character does now and what the player asks for, and jumps
there, easing the jump. Holden's reference (`orangeduck/Motion-Matching`)
and Unreal's Pose Search agree on the parts: features per frame in the
character's root space, normalized per channel so weights compare;
a search over every frame, sped up by bounds; a node that searches,
plays and inertializes. The library's results are bit for bit the same
everywhere (mani-0001), its assets are views over the caller's bytes
(mani-0002), and its graphs ease jumps with an inertialize node
(mani-0023).

## Decision

- **Space:** a sampled pose's model space is already the root's, as
  root motion rides beside the clip (mani-0022). A bone's position is
  its model translation; its velocity the difference of its positions a
  database frame either side over their time apart, the root's own
  motion carrying the later one; its facing a chosen axis of the bone.
  The trajectory is the clip's root motion from the frame, on the ground
  plane of the database's up axis (+y by default); a facing is the
  forward axis (+z by default) turned by it and projected.
- **Channels:** bone position, bone velocity, bone facing, trajectory
  position, trajectory facing and float track, each at up to 16 times
  ahead of or behind the frame (60 s at most), each with a weight. Each
  channel is normalized on its own as Holden's reference does: the mean
  per feature taken off, divided by the mean of its features' standard
  deviations over the weight, in binary64 over the frames in order. A
  channel that never varies is refused with its index.
- **Entries:** a database holds spans of its clips, each with its own
  flags (looping: times past its end wrap to its start, the root's
  motion composed across; else clamped to its ends), a cost bias added
  to its frames, 32 tag bits for filters, and a flag for frames that
  play but are never jumped to. Frames sit at `start + k / rate` (in
  binary64, rounded to binary32), every `k` whose time is at most the
  end (before it when looping): counted in binary64, then one frame
  less or more once when the binary32 times of the last frame or the
  next say otherwise, so an end a hair above a whole frame in binary64
  adds no frame at the wrap, and a start so large that steps no longer
  move the time still gives a count at once (a fuzzed file found the
  unbounded form's hang). Opening checks every entry's frame count
  against it.
- **Search** (`maniSearchMotionDatabase`): exact, Holden's. A frame's
  cost is a constant (its entry's cost bias plus the transition cost;
  for the continuing frame the continuing bias) plus the squared
  differences of the normalized query and its features in feature
  order, in binary32. The continuing frame is scored first; then each
  candidate entry's frames in order, a run of 64 skipped when its box's
  bound (the entry's constant plus the squared distances to the box, in
  the same order) already reaches the best, then a run of 16, then each
  frame, every sum stopped once it reaches the best. Box terms are at
  most their frames' and binary32 rounding is monotone, so pruning
  never changes the result: the plain scan's frame and cost bit for
  bit, which a test checks over thousands of random databases and
  searches. Ties go to the continuing frame, then the lowest frame. A
  frame is not a candidate when its entry's tags lack a required tag or
  hold an excluded one, its entry is never jumped to, or it lies nearer
  than given times to the continuing frame on its entry, to a clamped
  entry's end, or to one of up to eight earlier picks on its entry
  (Holden's ignored surroundings and range ends, Unreal's pose jump
  threshold and reselect history; looping entries measured around
  their span the shorter way).
- **The file** (container kind 8): the channels and their times, the
  clips by identity, the entries, the normalization, the normalized
  features frame by frame, and bounds over runs of 16 and 64 frames
  (Holden's runs, crossing entries). Opening recomputes every bound
  and refuses one that is not its run's exact minimum or maximum, as
  the search's exactness rests on them.

## Consequences

A database costs one pass over its clips at build and a check of every
byte when opened. Hosts tune by weights and entries, not code; a
database matches its skeleton and clips by identity, so a host cannot
pair it with others. The search, the graph node and the trajectory
predictor build on this record.

Opening is fuzzed; every check is tested against a file changed at its
byte.
