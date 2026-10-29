# Motion matching

Motion matching plays a character from a database of its clips instead of a hand-made state machine: every so often it searches for the frame whose pose and predicted path best match what the character does and what the player asks for, and jumps there, easing the jump. Learned motion matching does the same with a small trained model whose memory does not grow with the data.

## Databases

A motion database (`maniBuildMotionDatabase`) is built from a skeleton's clips. It holds, for every frame, features: bone positions and velocities, bone facings, trajectory positions and facings ahead of (or behind) the frame, and float track values, each normalized and weighted. Entries say which spans of which clips the database covers, whether each loops, its cost bias, tags for filtering, and whether its frames may be jumped to. An entry flagged mirrored plays its clip mirrored through a mirror map, doubling the data without a second clip.

`maniMotionDatabase_Search` finds the frame nearest a query, exactly: the same frame and cost a scan of every frame would give, pruned by a k-d tree over the frames' features. It can exclude frames near the current one or near recent picks, and filter entries by tags. The tree's leaves hold at most 16 frames unless the source's `leafFrames` says 8 (fewer instructions a search, more bytes) or 32 (the reverse).

## In a graph

A graph's matching node does the whole loop: it builds the query from the parameters the program sets (trajectory goals as vec2 parameters, float channels as floats), searches every given period or when its trigger fires, plays what it finds, and asks the inertialize node above it to ease each jump. The graph must have that inertialize node; the builder refuses it otherwise.

`maniAdvanceTrajectory` predicts the character's root a few tenths of a second ahead from a desired velocity and facing, with critically damped springs, and `maniMotionDatabase_ComputeTrajectoryFeatures` turns the prediction into the node's trajectory parameters. A program may use its own predictor instead.

## Learned motion matching

A learned model (`maniOpenLearnedModel`) replaces a database and its clips by three networks: one gives a pose from a compact state, one steps the state forward, one projects a query onto the nearest state. `maniLearnedModel_Project`, `maniLearnedModel_Step` and `maniLearnedModel_Decompress` run them; a graph's learned node plays a model as the matching node plays a database. A model costs a few megabytes however long the data, and more instructions a frame than a database search, so it pays off for large databases. Opening a model checks every weight is finite, but finite weights can still carry a value past the float range: the three calls then return `mani_errorFormat` and write nothing.

The library trains models itself, from a database, its clips and skeleton (`maniCreateLearnedTrainer`, `maniLearnedTrainer_Train`, `maniBuildLearnedModelFromTrainer`), with the same result on every platform and with any number of threads through the program's task hooks. Training runs in bounded slices of iterations, and checkpoints (`maniLearnedTrainer_SaveCheckpoint`, `maniLearnedTrainer_RestoreCheckpoint`) stop and resume it exactly. A model can also be built from networks trained elsewhere (`maniBuildLearnedModel`).

## Not obvious

- A database is tied to its skeleton and clips by identity; changing a clip means rebuilding the database.
- Features are normalized per channel, so channel weights compare channels directly; a channel that never varies is refused when building.
- The node moves only while it is weighed; under a machine state that is not active it neither moves nor searches.
- Training at the default sizes runs 500,000 iterations a stage; run it off the frame, a slice at a time.
