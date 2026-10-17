# mani-0025. Learned motion matching

Status: Accepted

## Context

Motion matching (mani-0024) keeps its database and the clips it plays
in memory, and both grow with the data. Learned motion matching
(Holden, Kanoun, Perepichka and Popa, SIGGRAPH 2020) replaces them by
three small networks: a Decompressor that gives a frame's pose from its
features and a few latent values, a Stepper that moves features and
latent values from one frame to the next, and a Projector that maps a
query to the features and latent values of its nearest frame. Their
paper measures ten to twenty times basic motion matching's cost a frame
and memory that stays at a few megabytes however large the data; their
reference trains the networks in PyTorch, whose results are not
reproducible across releases or platforms. The library's results are
bit for bit the same everywhere (mani-0001) and its assets are views
over the caller's bytes (mani-0002).

## Decision

- **Model file** (container kind 9): the networks (dense layers, ReLU
  on all but the last, input and output normalization), the schema,
  normalization and axes of the database the model learned, the step
  rate, which bones' scales the Decompressor gives and every other
  bone's constant scale, the names of its float tracks and events, one
  Projector for each tag filter it learned (required and excluded
  tags), and the identities of its skeleton and database. Weights are
  binary32, or 16-bit integers with a binary32 scale for each layer
  input (the input's weights over their largest magnitude / 32767,
  rounded to nearest, ties to even), chosen at build; the paper found
  16 bits lose no visible precision. Opening checks every record, count,
  value and name, and that each network's layers chain from its role's
  input to its role's output.
- **Evaluation:** Holden's order: inputs normalized, each layer's
  outputs started from its biases and added each input times its row of
  weights, input by input with the output innermost, inputs of exactly
  0 skipped; binary32 with no contraction, so the bits are the same on
  every target whether the compiler vectorizes the loops or not.
  `maniLearnedProject` gives a state from a normalized query,
  `maniLearnedStep` adds one step's seconds times the Stepper's rate of
  change, `maniLearnedDecompress` gives the pose and the rest.
- **Outputs** of the Decompressor, bone by bone: translation and two
  columns of the rotation matrix (the first normalized, the third the
  normalized cross of it and the second, the second their cross; a
  quaternion by Shepperd's method, w not negative); then the scaled
  bones' scales; the root's velocity and angular velocity in its own
  frame; the float tracks' values; and per event a state the event
  fires on as it rises past 0.5, as Holden's foot contacts. Velocities
  are not given: an inertialize node keeps its own history.
- **Still to come, decided now:** training in the library (each
  network's backward pass by hand, AdamW with amsgrad as Holden's code,
  Projector targets by the exact search; the same bits on every
  platform and with any number of threads, a batch split into fixed
  chunks summed in order on the host's task hooks; checkpoints that
  resume to the same bits), and a graph node beside the motion matching
  node that steps at the model's rate and decompresses the state
  between its last two steps.

## Consequences

A model costs its networks' sizes in memory, not its data's; a frame
costs a Decompressor evaluation and a share of a step, a projection
costs the Projector, the largest network. Hosts choose per character
between the database (cheaper a frame, exact) and the model (smaller
past a few megabytes of data); both stay. A model can be built from
networks trained anywhere, and the library's own trainer will make one
a host can rebuild from its data to the same bits.

Opening is fuzzed with an evaluation of each network; every check is
tested against a file changed at its byte; every evaluation is tested
bit for bit against a plain evaluation in the same order.
