# Motion matching

Motion matching plays a character from a database of its clips: every so often it searches for the frame whose pose and root path best match what the character does now and what the player asks for, and jumps there, easing the jump. Learned motion matching replaces the database and its clips by three small networks whose size does not grow with the data. Both follow Holden's published work (the `orangeduck/Motion-Matching` reference and the SIGGRAPH 2020 paper), with Unreal's Pose Search as a second reference for the node's choices.

## Features

A sampled pose's model space is already the root's, since root motion rides beside the clip ([clips-and-compression.md](clips-and-compression.md#root-motion)). The database builder ([`src/match_build.c`](../../src/match_build.c)) samples each frame and computes channels of features:

- bone position (the model translation), bone velocity (positions a frame either side over their time apart, the root's own motion carrying the later one) and bone facing (a chosen axis);
- trajectory position and facing: the clip's root motion from the frame, projected on the ground plane of the database's up axis (+y by default);
- float track values.

Each channel is taken at up to 16 times ahead of or behind the frame, with a weight. Each channel is normalized on its own, as Holden's reference does: the mean per feature taken off, divided by the mean of its features' standard deviations over the weight, in binary64. A channel that never varies is refused rather than divided by zero.

Entries are spans of the database's clips, each with its own flags (looping: times past its end wrap and the root's motion composes across; else clamped), a cost bias, 32 tag bits for filters, and a flag for frames that play but are never jumped to. Frame counts are computed in binary64 and corrected once against the binary32 times, so an entry's end a hair above a whole frame adds no frame at the wrap. The correction is one step, never a loop: far past 2^24 seconds a frame's step no longer moves a binary32 time, so a loop over a hostile file's enormous start would never end.

## The search

`maniSearchMotionDatabase` ([`src/match_search.c`](../../src/match_search.c)) is exact. A frame's cost is a constant (its entry's bias plus the transition cost) plus the squared differences of the normalized query and its features, in feature order, in binary32. The continuing frame is scored first. Then each candidate entry's frames: a run of 64 is skipped when its bounding box's bound already reaches the best cost, then a run of 16, then each frame, every sum stopped once it reaches the best.

The bound of a box is never more than any of its frames' costs, and binary32 rounding is monotone, so pruning never changes the result: the search returns the plain scan's frame and cost bit for bit, which a test checks over thousands of random databases. Opening a database recomputes every bound and refuses one that is not its run's exact minimum or maximum, since exactness rests on them. Ties go to the continuing frame, then the lowest frame.

A frame is not a candidate when its entry's tags lack a required tag or hold an excluded one, its entry is never jumped to, or it lies within given times of the continuing frame, of a clamped entry's end, or of one of up to eight earlier picks (Holden's ignored surroundings and Unreal's reselect history).

On an hour of data with Holden's 27 features, a search visits about five and a half instructions' worth a frame against a plain scan's two hundred, about 2% more than Holden's own search on the same database and queries, which it picks the same frames as; the difference is tags, biases, no-jump flags and query checks his search does not have. An exact k-d tree with the same picks takes fewer instructions on steady queries and about the same after a sudden change of input; whether it replaces the bounds waits on timed runs, since instructions alone do not decide it.

## The node

A graph lists databases beside its clips; a matching node names its slot, a trigger, a parameter per trajectory and float channel, the search period and costs. It moves forward only while weighed, and searches when it has no entry, its clamped entry ended, its trigger is set or its period passed. A pick other than the continuing frame is jumped to and requests the node's duration of the inertialize node above, which the builder requires on every path ([graphs.md](graphs.md#inertialization-and-dead-blending)).

The query's bone channels are the continuing frame's features interpolated to the playhead toward the entry's next frame. The continuing frame alone lagged the displayed pose by up to a frame period, and that staleness moved the pick in a large share of settled searches; interpolated, the features come within 0.01 (squared, normalized) of the displayed pose's. Facings are directions, so they are renormalized after interpolation. The search scores the continuing frame by the same interpolated features, so playing on costs no bone difference. The displayed pose itself is not the query: while an inertialization eases, it carries the easing's offset, a frame late.

`maniAdvanceTrajectory` predicts the root with Holden's critically damped springs in closed form, using the library's exponential so the bits are the same everywhere; one advance equals many shorter ones up to rounding. `maniTrajectoryFeatures` turns a predicted sample into the node's trajectory parameters as the database measures its own. A host may feed the node from any predictor.

## Learned motion matching

A learned model (kind 9) holds three networks: a Decompressor (features and latent values to a pose, the root's velocities, float tracks and event states), a Stepper (the state's rate of change), and a Projector per tag filter (a query to the state of its nearest frame). Each is dense layers with ReLU on all but the last, weights in binary32 or 16-bit integers with a binary32 scale per layer input.

Evaluation ([`src/learned_eval.c`](../../src/learned_eval.c)) follows Holden's order: each layer's outputs start from its biases and add each input times its row of weights, input by input, skipping inputs of exactly 0, in binary32 without contraction, so the bits are the same whether the compiler vectorizes or not. Rotations come out as two matrix columns orthonormalized and turned into a quaternion by Shepperd's method.

The learned node steps the state at the model's rate in binary64 seconds (at most 20 steps an update) and interpolates the last two states, so its query, transition distance, motion and pose all read the same state at the playhead. Its root moves by the Decompressor's velocities; its events fire on their states rising past 0.5. The update keeps the Decompressor's hidden values for the state it decompressed, and the evaluation reuses them when the state's bytes match, a tenth of a learned frame.

A model's cost is its networks' size, not its data's: at Holden's sizes about ten times the matching node's instructions a frame, and smaller than the clips and database past roughly a quarter hour of data. Hosts choose per character; both stay.

### Training in the library

PyTorch's results are not reproducible across releases or platforms, so the library trains models itself ([`src/learned_train.c`](../../src/learned_train.c) and the `train_*` modules): Holden's stages (Compressor and Decompressor, then every frame's latent values, then Stepper and Projectors), backward passes written by hand, AdamW with amsgrad. A batch is split into fixed chunks whose gradients are summed in chunk order on the host's task hooks, so the result is the same with any thread count. `maniTrainLearned` runs a bounded number of iterations per call; checkpoints (kind 10) stop and resume to the same bits. A model can also be built from networks trained elsewhere (`maniBuildLearnedModel`).

## Traps

- The search's exactness depends on bounds being exact; a change to how features are stored must keep the bound computation and the opener's recomputation identical.
- Interpolating a rotation-like feature as a vector shortens it; a new channel kind that is a rotation needs its own interpolation rule before it joins the query.
- A learned node's interpolation and the matching node's continuing-frame interpolation exist for the same reason: everything the node reads must describe the displayed instant at any update rate.
