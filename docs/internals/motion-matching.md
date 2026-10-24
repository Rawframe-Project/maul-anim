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

`maniSearchMotionDatabase` ([`src/match_search.c`](../../src/match_search.c)) is exact. A frame's cost is a constant (its entry's bias plus the transition cost) plus the squared differences of the normalized query and its features, in feature order, in binary32. The continuing frame is scored first. Then the database's k-d tree is walked from its root, the nearer half of each node first: a node is skipped when the bound of its bounding box shows it holds no better frame, and a leaf's frames are scored, every sum stopped once it passes the best.

The builder ([`src/match_tree.c`](../../src/match_tree.c)) splits each node of more than a leaf's frames at the median of the feature its frames spread widest over, so the tree's shape follows from the frame count and the leaf size alone, and stores the features in the tree's order, so a leaf's frames are read in one run. Each node keeps its box, its lowest frame and the least cost bias of its frames' entries.

A box's bound starts from the transition cost plus that least bias and adds the squared distances from the query to the box in feature order; it is never more than any of its frames' costs, and binary32 rounding is monotone, so pruning never changes the result: the search returns the plain scan's frame and cost bit for bit, which a test checks over thousands of random databases, with every leaf size and with random orders as well as the builder's. Opening a database checks that the order holds every frame once with its entry and recomputes every box, lowest frame and least bias, refusing any that is not exact, since exactness rests on them. The tree yields frames out of order, so ties are settled as the scan settles them: the continuing frame first, then the lowest frame. A sum is stopped only once it passes the best, not when it reaches it, and a node whose bound equals the best is skipped only when it cannot hold a lower frame.

A frame is not a candidate when its entry's tags lack a required tag or hold an excluded one, its entry is never jumped to, or it lies within given times of the continuing frame, of a clamped entry's end, or of one of up to eight earlier picks (Holden's ignored surroundings and Unreal's reselect history).

Before format 3 the search pruned by Holden's bounds over runs of 16 and 64 frames, within about 2% of his own search's instructions; its cost grew with the frame count, 5.5 to 18 instructions a frame. `bench/bench_match.c` measures both on synthetic locomotion (an eight-bone rig walking, running, turning and standing at random; clamped entries; 1,000 queries of each shape: steady, the continuing frame's bone features with its path nudged; sudden, another frame's path; cold, nothing continuing) by user instructions, which do not depend on the machine's load. With Holden's 27 features, every pick the same:

| Search | instructions a query at 100,856 frames (steady / sudden / cold) | at 300,767 frames | index bytes a frame (100,856 / 300,767 frames) |
|---|---|---|---|
| run bounds (format 2) | 703,430 / 1,018,161 / 1,114,832 | 2,021,514 / 2,647,051 / 3,061,250 | 16.9 / 16.9 |
| tree, leaves of 8 | 103,444 / 175,473 / 189,700 | 157,156 / 235,783 / 320,788 | 86.1 / 111.4 |
| tree, leaves of 16 (the default) | 116,648 / 197,823 / 211,945 | 172,790 / 256,348 / 350,340 | 49.0 / 61.7 |
| tree, leaves of 32 | 145,434 / 249,914 / 259,620 | 210,311 / 308,577 / 417,728 | 30.5 / 36.8 |

The features themselves take 108 bytes a frame. The tree's advantage grows with the database and shrinks with the feature count: at leaves of 16 it takes 1.4 to 2.4 times fewer instructions than the bounds at 5,403 frames with 15 or 27 features and 8.7 to 16.5 times fewer at 300,767, but with 45 features at 5,403 frames it takes up to 27% more on steady and sudden queries (the bounds took 0.79 and 0.86 of its count). Leaves of 16 are the default as the middle of the trade; a host short of memory takes 32, one short of time 8.

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

- The search's exactness depends on the boxes being exact; a change to how features are stored must keep the builder's boxes (`src/match_tree.c`) and the opener's recomputation identical, down to which of -0 and +0 a minimum keeps. Exactness does not depend on the tree's order: any order the opener accepts gives the plain scan's result, which the search test checks with random orders.
- Interpolating a rotation-like feature as a vector shortens it; a new channel kind that is a rotation needs its own interpolation rule before it joins the query.
- A learned node's interpolation and the matching node's continuing-frame interpolation exist for the same reason: everything the node reads must describe the displayed instant at any update rate.
