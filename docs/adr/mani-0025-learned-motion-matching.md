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
- **Training in the library:** `maniCreateLearnedTrainer` takes a
  motion database, its clips and skeleton, the Projectors' tag filters
  and options (Holden's by default: latent 32, hidden 512, 500,000
  iterations a stage, batch 32); `maniTrainLearned` runs a bounded
  number of iterations at a time through the stages (the Compressor
  and Decompressor together, then every frame's latent values, then
  the Stepper and each Projector), each network's backward pass written
  by hand, AdamW with amsgrad and weight decay 0.001, the learning rate
  0.001 times 0.99 for each whole thousand iterations. The Decompressor
  learns in local and character space; the Stepper from windows of
  consecutive frames of one entry; each Projector from noisy queries
  against the exact search's picks under its filter. A batch is split
  into fixed chunks whose gradients are summed in chunk order, on the
  host's task hooks or one thread: the same bits on every platform and
  with any thread count. `maniBuildLearnedModelFromTrainer` builds the
  model.
- **Checkpoints** (container kind 10): the trainer's progress (stage,
  iteration, generator), every frame's latent values, the parameters
  and their moments, and the networks finished in earlier stages, under
  a signature of the source, options, filters and names they must be
  restored into; `maniSaveLearnedCheckpoint` and
  `maniRestoreLearnedCheckpoint` stop and resume to the same bits,
  every value checked as external bytes: a stage not begun has no loss
  or generator (both 0), one begun a generator not all 0, so a restored
  checkpoint saves back to the same sections.
- **Model slots in graphs:** a graph lists its models as it lists
  databases (an opened model of its skeleton, a name; section tag 12),
  bound by identity through `maniGraphAssets`; the JSON form names them
  in a "models" list.
- **The learned node** (kind 15): plays a model slot's state, the
  features and latent values. The state steps at the model's rate (the
  database's frame rate) in binary64 seconds, at most
  `MANI_GRAPH_LEARNED_MAX_STEPS` (20) steps an update, a longer update
  playing as 20 steps' seconds and losing the rest as fixed-step
  simulations do; between steps the node interpolates the last two
  states. It projects its query (the state's features with the
  trajectory and float channels taken from its parameters) when it has
  no state, its trigger is set or its period passed, through the
  Projector whose filter equals its tags; a projection farther than the
  transition cost from the state replaces it and requests the node's
  duration of inertialization above, as the motion matching node does.
  Its root moves by the Decompressor's velocities over the step, its
  events fire on their states' rising edges past 0.5, its pose is the
  Decompressor's at the interpolated state. Its block (the two states,
  the step and period clocks, the events' bits) is in the saved state;
  subgraphs carry their model slots; the JSON form writes it as
  "learned".

## Consequences

A model costs its networks' sizes in memory, not its data's; a frame
costs a Decompressor evaluation, a share of a step and a share of a
projection, whatever the data. Measured at Holden's sizes on a
four-bone character (a 60 frames a second graph frame, projecting every
0.1 s): about 607,000 instructions against the motion matching node's
53,000 on ten minutes of data (145,000 on an hour), 11 times; the
model 4.8 MB (2.4 MB with 16-bit weights, whose conversion costs 36%
more instructions) against 0.37 MB of clips and database a minute, so
it saves memory past about 13 minutes of such data, sooner as a
character's bones grow its clips. Hosts choose per character between
the database (cheaper a frame, exact) and the model (smaller with much
data); both stay. A model can be built from networks trained anywhere,
and the library's own trainer makes one a host can rebuild from its
data to the same bits.

Opening is fuzzed with an evaluation of each network, restoring
checkpoints with training on from them, and the graph fuzz targets'
hosts hold a model and a learned node; every check is
tested against a file changed at its byte; every evaluation is tested
bit for bit against a plain evaluation in the same order; training is
tested against its losses rolled out in binary64 and its gradients
against central differences; the node is tested update by update
against a reference built from the model's own functions.
