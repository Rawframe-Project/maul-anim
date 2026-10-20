# Maul Anim API reference

Generated from the public headers by `tools/gen_api.py`. The headers
are the source of truth; this file mirrors them.

## `base.h`

The base of the Maul Anim API: the library version, the export and attribute macros, and the result codes every fallible function returns.

```c
maniVersion maniGetVersion(void);
```
Returns the version of the library that was linked, which may differ from the MANI_VERSION macros a program was compiled with.  @return The library version. @par Thread safety Safe from any thread.

```c
uint64_t maniHash64(uint64_t seed, const void* data, int32_t byteCount);
```
Hashes bytes into a 64-bit value, the hash the library's determinism checks are built on and a host can use to compare poses between machines: eight bytes a round, read in the host's byte order, xored in, multiplied by an odd constant and folded, the leftover bytes one at a time. Its constants are frozen.  @param seed       MANI_HASH_INIT, or a hash to continue. @param data       The bytes. May be NULL when byteCount is 0. @param byteCount  The number of bytes, at least 0. @return The hash. @par Thread safety Safe from any thread.

```c
const char* maniResultName(maniResult result);
```
Returns the name of a result code, for diagnostics.  @param result  Any value; an unknown one is named as such. @return A static, NUL-terminated string such as "mani_errorCapacity". @par Thread safety Safe from any thread.

## `blend.h`

Blending (mani-0020): poses mixed by weights, per bone through masks, in caller memory. Poses and differences never mix: a blend takes layers of one kind and writes that kind.

```c
MANI_NODISCARD MANI_API maniResult maniBlendPoses(const maniSkeleton* skeleton, const maniBlendLayer* layers, uint32_t count, float threshold, maniPose* out);
```
Blends layers into a pose. Each bone takes from each layer its weight times its bone weight, divided by the bone's total (or by the threshold where the total is below it, the rest taking the shortfall; a difference's rest is the identity), so the result slides into the rest as weights fall. Rotations add on the arc that agrees with the sum so far (so the layers' order counts) and are then normalized; translations and scales add linearly and never pass the largest input's magnitude by more than rounding, so inputs up to 2^127 stay finite. With no weight on a bone it takes its rest as it is.  @param skeleton   The layers' and the output's skeleton. @param layers     count layers, all poses or all differences. @param count      The number of layers. @param threshold  Above 0 and at most 1 (MANI_BLEND_THRESHOLD in graphs). @param out        Receives the blend, of the layers' kind; not one of the layers' poses. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a pose of another skeleton, layers of both kinds, out among the layers, a weight outside 0 to 1 or a threshold out of range (out unchanged). @par Thread safety Safe from any thread; out is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniBuildMask(const maniSkeleton* skeleton, const maniMaskChain* chains, uint32_t count, float defaultWeight, float* weights, uint32_t capacity);
```
Makes per-bone weights from chains. A bone takes a chain on itself alone first, else the subtree chain on its nearest ancestor or itself, else the default; of two chains on one bone and kind the later wins.  @param skeleton       The skeleton. @param chains         count chains. @param count          The number of chains. @param defaultWeight  The weight of bones no chain reaches. @param weights        Receives a weight per bone in skeleton order. @param capacity       The floats weights holds. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a bone past the skeleton or a negative or non-finite weight; `mani_errorCapacity` when capacity is below the bone count (weights unchanged). @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniPoseDifference(const maniSkeleton* skeleton, const maniPose* pose, const maniPose* basis, maniPose* out);
```
Writes the difference of a pose from a basis, bone by bone: `d_t = t - t_b`, `d_r = r_b^-1 r` (normalized), `d_s = s / s_b` per axis. The output may be the pose or the basis.  @param skeleton  The poses' skeleton. @param pose      A pose of the skeleton. @param basis     A pose of the skeleton. @param out       Receives the difference. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a pose of another skeleton, a difference given as pose or basis, or a difference not finite (a basis scale component of 0 or near it, translations past binary32's range apart; out unchanged). @par Thread safety Safe from any thread; out is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniAddPoses(const maniSkeleton* skeleton, maniPose* pose, const maniBlendLayer* layers, uint32_t count);
```
Applies differences to a pose in order, each bone at its layer weight times its bone weight (both 0 to 1): `t + w d_t`, `r nlerp(1, d_r, w)` (the difference's turn taken the short way), `s (1 + w (d_s - 1))` per axis, a factor between 1 and `d_s`.  @param skeleton  The poses' skeleton. @param pose      A pose of the skeleton, changed in place. @param layers    count layers, each a difference. @param count     The number of layers. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a pose of another skeleton, a difference as the pose, a layer not a difference, or a weight outside 0 to 1 (the pose unchanged). @par Thread safety Safe from any thread; the pose is used by one thread at a time.

## `bodyik.h`

Full-body inverse kinematics (mani-0031): a part of a skeleton, every bone on a path from a top bone to one of several effectors, bent so the effectors reach their goals, within per-bone stiffness and limits. One description, two solvers over it: position-based rigid bodies with XPBD compliance (fast, stiffness meaning the same at any iteration count as the solve converges) and selectively damped least squares (Buss and Kim; every effector in one least-squares step, no damping constant). Both start from the input pose on every call and keep nothing between calls; the caller lends the scratch memory. As the other solvers (ik.h), they read the pose's locals, compose the model frames of the bones they need, write the locals of the bones they move and leave the model stale. Goals are in the model space of the pose; angles and axes fixed to a bone are in its parent's frame unless stated.

```c
size_t maniBodyIkScratchSize(const maniSkeleton* skeleton, const maniBodySolve* solve);
```
Returns the bytes of scratch memory a full-body solve needs, by either solver (the larger of the two).  @param skeleton  The skeleton, or NULL. @param solve     The solve, or NULL. @return The size; 0 for NULL or a solve maniSolveBodyXpbd refuses. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSolveBodyXpbd(maniPose* pose, const maniSkeleton* skeleton, const maniBodySolve* solve, void* scratch, size_t scratchSize, bool* reachedOut);
```
Bends the solved part of a pose toward its effectors' goals as rigid bodies (Müller et al.'s positional and angular corrections with XPBD compliance): each bone a body whose resistance to turning comes from its rotation stiffness, each joint holding a child's origin at its place in the parent (softly by position stiffness with stretch), each effector pulling its bone's origin and rotation by its strength, then limits; one Gauss-Seidel pass an iteration in a fixed order (effectors by index, joints from the tips up, stiffness, limits). Preferred angles, pull chains and pre-pull act first. The bones' rotations come from the bodies; their translations stay the input's unless stretching. Stops early once every effector lies within the tolerance. Strength and stiffness set the pose the passes converge to (XPBD's compliance means the same whatever the count only as the solve converges); short of convergence they have acted less, so the result changes with the iteration count.  @param pose         A pose of the skeleton (not a difference); the solved part's locals are written. @param skeleton     Its skeleton. @param solve        The solve. @param scratch      Scratch memory, 16-byte aligned. @param scratchSize  Its size: at least maniBodyIkScratchSize. @param reachedOut   Receives whether every effector misses by no more than the tolerance at full weight; may be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL pose, skeleton, solve or scratch, a pose of another skeleton or a difference, a top or bone out of range, an effector not the top or below it, a bone twice among the effectors or the settings, a setting's bone off the solved part, counts out of range, a solved part past MANI_BODY_IK_MAX_BONES, a goal, rotation or preferred angle not finite, a rotation of length 0, a limit of an unknown kind or out of range, a weight, strength, pull chain or stiffness outside [0, 1], a tolerance below 0 or not finite, a top mode unknown, misaligned or too little scratch. @par Thread safety Safe from any thread; the pose and the scratch are used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniSolveBodySdls(maniPose* pose, const maniSkeleton* skeleton, const maniBodySolve* solve, void* scratch, size_t scratchSize, bool* reachedOut);
```
Bends the solved part of a pose toward its effectors' goals by selectively damped least squares (Buss and Kim): each iteration takes the Jacobian of every effector's position (and, with a rotation weight, its rotation, as a length: the angle times the effector's reach from the top) by three model axes per bone (and three translations for a free top), its rows scaled by strength and its columns by one minus rotation stiffness (a locked bone and a held top have none); decomposes it (one-sided Jacobi, at most 30 sweeps); and steps each singular direction by the most its effectors' movement allows, each bone's turn at most pi/4 an iteration; then limits. Each miss counts at most half the mean bone length. Preferred angles, pull chains and pre-pull act first. Stops early once every effector lies within the tolerance. Every length is kept: stretch is refused.  The decomposition's work in an iteration grows with the square of the rows (six an effector, those of an effector without a rotation weight free) times the columns, so an iteration costs far more than one of maniSolveBodyXpbd's: about 1.8 million instructions for a 22-bone part with five effectors, 54 million for 128 bones and 32 effectors without rotations (XPBD: 57 thousand and half a million). The iteration count is the caller's budget: suited to small parts, a few iterations, or work off the frame.  @param pose         A pose of the skeleton (not a difference); the solved part's locals are written. @param skeleton     Its skeleton. @param solve        The solve; its stretch must be false. @param scratch      Scratch memory, 16-byte aligned. @param scratchSize  Its size: at least maniBodyIkScratchSize. @param reachedOut   Receives whether every effector misses by no more than the tolerance at full weight; may be NULL. @return `mani_success`; `mani_errorInvalid` as maniSolveBodyXpbd, and for stretch. @par Thread safety Safe from any thread; the pose and the scratch are used by one thread at a time.

## `build.h`

The offline side: a builder turns source data into the library's files (mani-0003), which it keeps until the next build for a copy into caller memory. A builder allocates through the allocator of its def and is built only when MAUL_ANIM_OFFLINE is on.

```c
static inline maniBuilderDef maniDefaultBuilderDef(void);
```
Returns a builder def with every setting at its default.  @return The def, with a valid cookie. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniCreateBuilder(const maniBuilderDef* def, maniBuilder** builderOut);
```
Makes a builder from a def.  @param def        The def. @param builderOut Receives the builder, or NULL on failure. @return `mani_success`; `mani_errorVersion` for a def built against another major or minor version; `mani_errorInvalid` for a NULL argument, a def without its cookie or an allocator with one function; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
void maniDestroyBuilder(maniBuilder* builder);
```
Destroys a builder and the file it holds. Called back in from the builder's allocator it destroys nothing and counts one misuse.  @param builder  The builder, or NULL. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
uint32_t maniBuilderMisuses(const maniBuilder* builder);
```
Returns how many calls on the builder were refused for invalid arguments (conventions section 8) or as calls back in from its allocator (mani-0028).  @param builder  The builder. @return The count. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildSkeleton(maniBuilder* builder, const maniSkeletonSource* source);
```
Builds a skeleton file: checks every bone, orders the bones depth-first (children and roots in the source's order), normalizes rotations, and keeps the file in place of any built before.  @param builder  The builder. @param source   The bones. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse) or with the reason and source bone for a refused source; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildSkin(maniBuilder* builder, const maniSkinSource* source);
```
Builds a skin file bound to the source's skeleton, and keeps it in place of any file built before.  @param builder  The builder. @param source   The skin. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse), or with the reason and joint for a refused source (no joints or too many, a bone past the skeleton's, a matrix not finite); `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildInfluences(maniBuilder* builder, const maniInfluenceSource* source);
```
Builds an influences file bound to the source's skin: zero weights dropped, each vertex's influences sorted by decreasing weight then joint, padded to groups of four with joint 0 at weight 0 and renormalized to sum 1 (in binary64, then rounded); a vertex with no weight keeps all 0, and skinning leaves it as it is. Keeps the file in place of any built before.  @param builder  The builder. @param source   The influences. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse), or with the reason and item for a refused source (item a vertex, other an influence index): `maniBuildReasonCount` for no vertices, offsets not increasing from 0 or more than 64 weighted influences on a vertex, `maniBuildReasonJoint`, `maniBuildReasonNotFinite`, `maniBuildReasonWeight`, `maniBuildReasonRepeatedJoint`; `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildCenters(maniBuilder* builder, const maniCenterSource* source);
```
Builds the influences again with a center of rotation for every vertex (Le and Hodgins 2016, Eq. 4): the triangles' centroids weighted by their areas and by the similarity of their average weights to the vertex's, in binary64. A vertex similar to no triangle (a single influence, say) takes its rest position. Keeps the file in place of any built before.  @param builder  The builder. @param source   The influences, mesh and sigma. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse), or with the reason and item for a refused source: `maniBuildReasonNotFinite` (a position; item the vertex), `maniBuildReasonTriangle`, `maniBuildReasonOption` (sigma below 0 or not finite), `maniBuildReasonCount` (a stride under 12); `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildClip(maniBuilder* builder, const maniClipSource* source);
```
Builds a full-precision clip file: evaluates every channel at each sample time as glTF defines its interpolation (in binary64, rotations normalized and kept in one hemisphere from sample to sample), and keeps the file in place of any built before.  @param builder  The builder. @param source   The clip. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse), or with the reason and track for a refused source; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniMeasureClip(maniBuilder* builder, const maniClipSource* source, const maniClip* clip, maniClipReport* reportOut);
```
Measures a clip against the source it was built from, at every sample time and halfway between samples, through the clip's own sampling, against the source evaluated in binary64. Rotations are compared by the angle between them.  @param builder    A builder, for its allocator. @param source     The clip's source, as maniBuildClip accepts it. @param clip       The clip opened from what the build made. @param reportOut  Receives the report. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a source the build refuses, or a clip not of the source's skeleton or missing one of its float tracks; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
maniCompressOptions maniDefaultCompressOptions(void);
```
Returns the default compression options: precision 0.0001 and shell distance 0.03 (0.1 mm at 3 cm when the unit is the metre), float tracks within 0.00001.  @return The options. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniCompressClip(maniBuilder* builder, const maniClip* clip, const maniSkeleton* skeleton, const maniCompressOptions* options, maniCompressReport* reportOut);
```
Compresses a full-precision clip into the builder (mani-0019): default and constant channels folded, segments of about 16 samples, ranges over the clip and the segment, and per segment the fewest bits that keep every bone's shell and every float track within its tolerance at every sample. The same input gives the same bytes on every platform.  @param builder    The builder. @param clip       A full-precision clip. @param skeleton   Its skeleton. @param options    The tolerances. @param reportOut  Receives the report. May be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a compressed clip, a skeleton not the clip's, or a tolerance not finite and above 0; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildSpringSet(maniBuilder* builder, const maniSpringSetSource* source);
```
Builds a spring set (mani-0026): the source checked (bones, shapes, chains and centers as VRMC_springBone and its extensions require, every value in its range), each joint's drag, stiffness and gravity converted from a frame of the reference rate to a step of the step rate, colliders listed per spring.  @param builder  A builder; its last output is replaced on success. @param source   The set's source. @return `mani_success`; `mani_errorInvalid` with the reason, item and other for a refused source (or NULL arguments, a misuse); `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildRagdoll(maniBuilder* builder, const maniRagdollSource* source);
```
Builds a ragdoll profile (mani-0027): the source checked (bones, one root, shapes, sizes, masses, frames, limits, drives, the axes), the bodies ordered by bone, each joint's frame in its parent body worked out from the skeleton's rest pose.  @param builder  A builder; its last output is replaced on success. @param source   The profile's source. @return `mani_success`; `mani_errorInvalid` with the reason, item and other for a refused source (or NULL arguments, a misuse; maniBuildReasonCount for no bodies or more than the skeleton's bones); `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniCopyBuilt(maniBuilder* builder, void* bufferOut, size_t capacity, size_t* sizeOut);
```
Copies the last built file, or the last written text, into caller memory.  @param builder   The builder. @param bufferOut Receives the bytes: at least capacity bytes; may be NULL when capacity is 0. Open a file's copy from a 16-byte aligned address. @param capacity  The buffer's size in bytes. @param sizeOut   Receives the size in bytes. May be NULL. @return `mani_success`; `mani_errorCapacity` when the buffer is smaller than the file or text, with the size still written; `mani_errorInvalid` for a NULL builder, a NULL buffer with a positive capacity, or a builder holding neither. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniCopySkeletonOrder(maniBuilder* builder, uint32_t* boneOut, uint32_t capacity, uint32_t* countOut);
```
Copies where each source bone of the last built skeleton went: the skeleton's index of source bone i at boneOut[i].  @param builder   The builder. @param boneOut   Receives up to capacity indices; may be NULL when capacity is 0. @param capacity  The array's length. @param countOut  Receives the source's bone count. May be NULL. @return `mani_success`; `mani_errorCapacity` when the array is shorter than the count, with the count still written and the array filled as far as it goes; `mani_errorInvalid` for a NULL builder, a NULL array with a positive capacity, or a builder holding no skeleton. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildMorphTargets(maniBuilder* builder, const maniMorphSource* source);
```
Builds a morph targets file: checks every channel and target, sorts each sparse target's vertices and sums a vertex given twice (in binary64, in the source's order), stores a target touching every vertex densely, and keeps the file in place of any built before.  @param builder  The builder. @param source   The channels. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse), or with the reason and item for a refused source (item a channel for `maniBuildReasonCount` on a channel without targets, `maniBuildReasonName` and `maniBuildReasonDuplicateName`; a target, counted over the channels in order, otherwise): `maniBuildReasonName` for a target's name has item the target and other 1. `maniBuildReasonCount` (no channels, no vertices, more than MANI_MORPH_MAX_ATTRIBUTES attributes (item 0), or an index-less target whose count is not the vertex count), `maniBuildReasonAttribute` (item the attribute), `maniBuildReasonName`, `maniBuildReasonDuplicateName` (other the earlier channel), `maniBuildReasonWeight`, `maniBuildReasonVertex`, `maniBuildReasonNotFinite` (other the given vertex's place); `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

## `clip.h`

Clips (mani-0006): a skeleton's motion as samples on one uniform grid, read in place from a clip file and bound to its skeleton by identity. Sampling costs the same at any time, in any order.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenClip(const void* bytes, size_t size, maniClip* clipOut);
```
Opens a clip file in place: checks the container, the header, every track and every sample (finite, rotations unit) and the identity.  @param bytes    The file, 16-byte aligned. @param size     Its size in bytes; bytes past the file are ignored. @param clipOut  Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
bool maniClipCompressed(const maniClip* clip);
```
Returns whether a clip is compressed (mani-0019); a full-precision clip is the build's output and the reference.  @param clip  An opened clip. @return True for a compressed clip. @par Thread safety Safe from any thread.

```c
float maniClipDuration(const maniClip* clip);
```
Returns the clip's duration in seconds, at least 0.  @param clip  An opened clip. @return The duration. @par Thread safety Safe from any thread.

```c
float maniClipSampleRate(const maniClip* clip);
```
Returns the number of samples per second.  @param clip  An opened clip. @return The rate. @par Thread safety Safe from any thread.

```c
uint32_t maniClipSampleCount(const maniClip* clip);
```
Returns the number of samples, at least 1: one at each multiple of the sample period, the last at the duration.  @param clip  An opened clip. @return The sample count. @par Thread safety Safe from any thread.

```c
maniLoopPolicy maniClipLoopPolicy(const maniClip* clip);
```
Returns the clip's loop policy.  @param clip  An opened clip. @return maniLoopClamp or maniLoopRepeat. @par Thread safety Safe from any thread.

```c
maniAdditiveBasis maniClipAdditiveBasis(const maniClip* clip);
```
Returns what an additive clip's differences are from; sampling an additive clip writes a difference (mani-0020), its channels without a track the identity.  @param clip  An opened clip. @return maniAdditiveNone, maniAdditiveRest or maniAdditiveFirstFrame. @par Thread safety Safe from any thread.

```c
uint64_t maniClipIdentity(const maniClip* clip);
```
Returns the clip's identity: a hash of its sections, the same for the same bytes.  @param clip  An opened clip. @return The identity. @par Thread safety Safe from any thread.

```c
uint64_t maniClipSkeletonIdentity(const maniClip* clip);
```
Returns the identity of the skeleton the clip is bound to.  @param clip  An opened clip. @return The skeleton's identity. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSampleClip(const maniClip* clip, const maniSkeleton* skeleton, float time, maniPose* pose);
```
Samples the clip at a time into a pose: every bone's local transform, the clip's channels where it animates them and the skeleton's rest elsewhere; the model is marked stale. The time is clamped to [0, duration] (NaN is 0). Between two samples, translation and scale interpolate linearly and rotations by normalized linear interpolation.  @param clip      An opened clip. @param skeleton  The clip's skeleton. @param time      The time in seconds. @param pose      A pose of the skeleton. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a skeleton that is not the clip's or the pose's. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
uint32_t maniClipFloatTrackCount(const maniClip* clip);
```
Returns the number of float tracks (morph weights, curves).  @param clip  An opened clip. @return The count. @par Thread safety Safe from any thread.

```c
const char* maniClipFloatTrackName(const maniClip* clip, uint32_t track, uint32_t* lengthOut);
```
Returns a float track's name, UTF-8 and not terminated.  @param clip       An opened clip. @param track      The track, below the count. @param lengthOut  Receives the length in bytes, 0 for a track past the count. May be NULL. @return The name's bytes, or NULL for a track past the count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSampleClipBones(const maniClip* clip, const maniSkeleton* skeleton, float time, const uint32_t* bones, uint32_t count, maniPose* pose);
```
Samples some bones of the clip at a time, as maniSampleClip samples them; every other bone's local is left as it was, and the clip's other channels are not decoded.  @param clip      An opened clip. @param skeleton  The clip's skeleton. @param time      The time in seconds. @param bones     The bones, strictly increasing; may be NULL when count is 0. @param count     How many. @param pose      A pose of the skeleton. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skeleton that is not the clip's or the pose's, bones out of range or not increasing, or a difference pose (the pose unchanged). @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
size_t maniClipCursorSize(const maniClip* clip);
```
Returns the bytes a cursor needs to play a clip: a cursor made for a clip plays any clip that needs no more.  @param clip  An opened clip. @return The size in bytes; 0 for a NULL clip. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniInitClipCursor(void* memory, size_t size, maniClipCursor** cursorOut);
```
Makes a cursor in caller memory, bound to no clip yet.  @param memory     At least maniClipCursorSize bytes, MANI_CLIP_CURSOR_ALIGNMENT-aligned; the cursor lives in it. @param size       The memory's size; what it holds beyond the smallest cursor sets the clips it can play. @param cursorOut  Receives the cursor. @return `mani_success`; `mani_errorInvalid` for a NULL or misaligned argument; `mani_errorCapacity` when the memory is too small for any cursor. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSampleClipCursor(maniClipCursor* cursor, const maniClip* clip, const maniSkeleton* skeleton, float time, maniPose* pose);
```
Samples the clip at a time into a pose, as maniSampleClip does, through the cursor: a clip of another identity or at another address rebinds it, dropping what it held.  @param cursor    A cursor. @param clip      An opened clip. @param skeleton  The clip's skeleton. @param time      The time in seconds. @param pose      A pose of the skeleton. @return `mani_success`; `mani_errorInvalid` as maniSampleClip, or for a NULL cursor; `mani_errorCapacity` when the cursor is too small for the clip (the cursor and pose unchanged). @par Thread safety Safe from any thread; the cursor and pose are used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniSampleClipFloatsCursor(maniClipCursor* cursor, const maniClip* clip, float time, float* values, uint32_t capacity, uint32_t* countOut);
```
Samples every float track at a time, as maniSampleClipFloats does, through the cursor.  @param cursor    A cursor. @param clip      An opened clip. @param time      The time in seconds. @param values    Receives one value per track; may be NULL when capacity is 0. @param capacity  The array's length. @param countOut  Receives the track count. May be NULL. @return As maniSampleClipFloats; `mani_errorInvalid` also for a NULL cursor; `mani_errorCapacity` also when the cursor is too small. @par Thread safety Safe from any thread; the cursor is used by one thread at a time.

```c
uint32_t maniClipFindFloatTrack(const maniClip* clip, const char* name, size_t length);
```
Finds a float track by name.  @param clip    An opened clip. @param name    The name's UTF-8 bytes; may be NULL when length is 0. @param length  The name's length in bytes. @return The track, or MANI_NO_TRACK. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSampleClipFloats(const maniClip* clip, float time, float* values, uint32_t capacity, uint32_t* countOut);
```
Samples every float track at a time, clamped as maniSampleClip clamps it: a linear track interpolates its two samples, a step track holds the sample at or before the time.  @param clip      An opened clip. @param time      The time in seconds. @param values    Receives one value per track, in track order; may be NULL when capacity is 0. @param capacity  The array's length. @param countOut  Receives the track count. May be NULL. @return `mani_success`; `mani_errorCapacity` when the array is shorter than the track count, with the count written and nothing else; `mani_errorInvalid` for a NULL clip or a NULL array with a positive capacity. @par Thread safety Safe from any thread.

```c
uint32_t maniClipDiscreteTrackCount(const maniClip* clip);
```
Returns the number of discrete tracks (whole numbers held between keys).  @param clip  An opened clip. @return The count. @par Thread safety Safe from any thread.

```c
const char* maniClipDiscreteTrackName(const maniClip* clip, uint32_t track, uint32_t* lengthOut);
```
Returns a discrete track's name, as maniClipFloatTrackName does.  @param clip       An opened clip. @param track      The track. @param lengthOut  Receives the length in bytes. May be NULL. @return The name's bytes, or NULL for a track past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniClipFindDiscreteTrack(const maniClip* clip, const char* name, size_t length);
```
Finds a discrete track by name.  @param clip    An opened clip. @param name    The name's UTF-8 bytes; may be NULL when length is 0. @param length  The name's length in bytes. @return The track, or MANI_NO_TRACK. @par Thread safety Safe from any thread.

```c
int32_t maniClipDiscreteValue(const maniClip* clip, uint32_t track, float time);
```
Returns a discrete track's value at a time: its last key at or before the time, or its first key before that (NaN is 0).  @param clip   An opened clip. @param track  The track. @param time   The time in seconds. @return The value, or 0 for a track past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniClipEventCount(const maniClip* clip);
```
Returns the number of events.  @param clip  An opened clip. @return The count. @par Thread safety Safe from any thread.

```c
maniClipEvent maniClipEventAt(const maniClip* clip, uint32_t index);
```
Returns an event; events are in increasing time.  @param clip   An opened clip. @param index  The event. @return The event, or a zeroed one for an index past the count. @par Thread safety Safe from any thread.

```c
void maniClipEventRange(const maniClip* clip, float from, float to, uint32_t* firstOut, uint32_t* countOut);
```
Finds the events a playhead passes going from one time to another: forward (from below to) those with from <= time < to, backward (from above to) those with to < time <= from; none when the two are equal or either is NaN. They are consecutive indices.  @param clip      An opened clip. @param from      Where the playhead was. @param to        Where it is. @param firstOut  Receives the first event's index. @param countOut  Receives how many. @par Thread safety Safe from any thread.

```c
uint32_t maniClipMarkerCount(const maniClip* clip);
```
Returns the number of sync markers.  @param clip  An opened clip. @return The count. @par Thread safety Safe from any thread.

```c
maniClipEvent maniClipMarkerAt(const maniClip* clip, uint32_t index);
```
Returns a sync marker; markers are in increasing time.  @param clip   An opened clip. @param index  The marker. @return The marker, or a zeroed one for an index past the count. @par Thread safety Safe from any thread.

## `graph.h`

Graphs (mani-0023): a graph file built offline against one skeleton, opened in place, bound to its clips, and played by instances in caller memory. Its nodes form a DAG stored in topological order, each after its inputs, the output last.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenGraph(const void* bytes, size_t size, maniGraph* graphOut);
```
Opens a graph file in place: checks the container, every record, index, count, float and name, and that every node's inputs come before it.  @param bytes     The file, 16-byte aligned. @param size      Its size in bytes; bytes past the file are ignored. @param graphOut  Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint64_t maniGraphIdentity(const maniGraph* graph);
```
Returns the graph's identity, the hash of its contents.  @param graph  An opened graph. @return The identity; 0 for a NULL graph. @par Thread safety Safe from any thread.

```c
uint64_t maniGraphSkeletonIdentity(const maniGraph* graph);
```
Returns the identity of the skeleton the graph was built against.  @param graph  An opened graph. @return The identity; 0 for a NULL graph. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphNodeCount(const maniGraph* graph);
```
Returns the number of nodes.  @param graph  An opened graph. @return The count; 0 for a NULL graph. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphParameterCount(const maniGraph* graph);
```
Returns the number of parameters.  @param graph  An opened graph. @return The count; 0 for a NULL graph. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphFindParameter(const maniGraph* graph, const char* name, size_t length);
```
Finds a parameter by its name.  @param graph   An opened graph. @param name    The name's bytes, UTF-8. @param length  Its length in bytes. @return The parameter, or MANI_NO_PARAMETER. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniGraphParameterTypeOf(const maniGraph* graph, uint32_t parameter, maniGraphParameterType* typeOut);
```
Returns a parameter's type.  @param graph      An opened graph. @param parameter  The parameter. @param typeOut    Receives the type. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a parameter past the count. @par Thread safety Safe from any thread.

```c
const char* maniGraphParameterName(const maniGraph* graph, uint32_t parameter, uint32_t* lengthOut);
```
Returns a parameter's name, its bytes inside the file.  @param graph      An opened graph. @param parameter  The parameter. @param lengthOut  Receives the name's length; 0 past the count. @return The name, not terminated, or NULL past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphClipCount(const maniGraph* graph);
```
Returns the number of clip slots the graph plays.  @param graph  An opened graph. @return The count; 0 for a NULL graph. @par Thread safety Safe from any thread.

```c
uint64_t maniGraphClipIdentity(const maniGraph* graph, uint32_t clip);
```
Returns the identity of the clip a slot names.  @param graph  An opened graph. @param clip   The slot. @return The clip's identity; 0 past the count. @par Thread safety Safe from any thread.

```c
const char* maniGraphClipName(const maniGraph* graph, uint32_t clip, uint32_t* lengthOut);
```
Returns the name a slot gives its clip, its bytes inside the file.  @param graph      An opened graph. @param clip       The slot. @param lengthOut  Receives the name's length; 0 past the count. @return The name, not terminated, or NULL past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphDatabaseCount(const maniGraph* graph);
```
Returns the number of motion database slots the graph's motion matching nodes search.  @param graph  An opened graph. @return The count; 0 for a NULL graph. @par Thread safety Safe from any thread.

```c
uint64_t maniGraphDatabaseIdentity(const maniGraph* graph, uint32_t database);
```
Returns the identity of the motion database a slot names.  @param graph     An opened graph. @param database  The slot. @return The database's identity; 0 past the count. @par Thread safety Safe from any thread.

```c
const char* maniGraphDatabaseName(const maniGraph* graph, uint32_t database, uint32_t* lengthOut);
```
Returns the name a slot gives its motion database, its bytes inside the file.  @param graph      An opened graph. @param database   The slot. @param lengthOut  Receives the name's length; 0 past the count. @return The name, not terminated, or NULL past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphModelCount(const maniGraph* graph);
```
Returns the number of learned model slots the graph's learned motion matching nodes play.  @param graph  An opened graph. @return The count; 0 for a NULL graph. @par Thread safety Safe from any thread.

```c
uint64_t maniGraphModelIdentity(const maniGraph* graph, uint32_t model);
```
Returns the identity of the learned model a slot names.  @param graph  An opened graph. @param model  The slot. @return The model's identity; 0 past the count. @par Thread safety Safe from any thread.

```c
const char* maniGraphModelName(const maniGraph* graph, uint32_t model, uint32_t* lengthOut);
```
Returns the name a slot gives its learned model, its bytes inside the file.  @param graph      An opened graph. @param model      The slot. @param lengthOut  Receives the name's length; 0 past the count. @return The name, not terminated, or NULL past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphSpringSetCount(const maniGraph* graph);
```
Returns the number of spring set slots the graph's spring nodes read.  @param graph  An opened graph. @return The count; 0 for a NULL graph. @par Thread safety Safe from any thread.

```c
uint64_t maniGraphSpringSetIdentity(const maniGraph* graph, uint32_t slot);
```
Returns the identity of the spring set a slot names.  @param graph  An opened graph. @param slot   The slot. @return The set's identity; 0 past the count. @par Thread safety Safe from any thread.

```c
const char* maniGraphSpringSetName(const maniGraph* graph, uint32_t slot, uint32_t* lengthOut);
```
Returns the name a slot gives its spring set, its bytes inside the file.  @param graph      An opened graph. @param slot       The slot. @param lengthOut  Receives the name's length; 0 past the count. @return The name, not terminated, or NULL past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphMapCount(const maniGraph* graph);
```
Returns the number of retargeting map slots the graph's clips play through (mani-0032): one for each map its clip slots name.  @param graph  An opened graph. @return The count; 0 for a NULL graph. @par Thread safety Safe from any thread.

```c
uint64_t maniGraphMapIdentity(const maniGraph* graph, uint32_t slot);
```
Returns the identity of the retargeting map a slot names.  @param graph  An opened graph. @param slot   The slot. @return The map's identity; 0 past the count. @par Thread safety Safe from any thread.

```c
uint64_t maniGraphMirrorMapIdentity(const maniGraph* graph);
```
Returns the identity of the graph's mirror map (mani-0033), which its mirror nodes mirror through.  @param graph  An opened graph. @return The map's identity; 0 for a graph without one, or NULL. @par Thread safety Safe from any thread.

```c
uint64_t maniGraphMapSourceIdentity(const maniGraph* graph, uint32_t slot);
```
Returns the identity of the skeleton a map slot's map takes poses from: the skeleton its clips are of, which binding needs.  @param graph  An opened graph. @param slot   The slot. @return The source skeleton's identity; 0 past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphClipMap(const maniGraph* graph, uint32_t clip);
```
Returns the map slot a clip slot plays through.  @param graph  An opened graph. @param clip   The clip slot. @return The map slot; MANI_NO_TRACK for a clip of the graph's skeleton or past the count. @par Thread safety Safe from any thread.

```c
maniGraphNodeKind maniGraphNodeKindOf(const maniGraph* graph, uint32_t node);
```
Returns a node's kind (mani-0035): with its inputs and the output node, what a debugger draws the graph's tree from. Nodes are in order, each after its inputs, subgraphs inlined.  @param graph  An opened graph. @param node   The node. @return Its kind; 0 for a NULL graph or past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphNodeInputCount(const maniGraph* graph, uint32_t node);
```
Returns how many inputs a node has (a machine's are its states, a one-shot's its base and shot).  @param graph  An opened graph. @param node   The node. @return The count; 0 for a NULL graph or past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphNodeInput(const maniGraph* graph, uint32_t node, uint32_t input);
```
Returns a node's input by its place among the node's inputs.  @param graph  An opened graph. @param node   The node. @param input  The place, below maniGraphNodeInputCount. @return The input node; MANI_GRAPH_NONE for a NULL graph or past a count. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphOutputNode(const maniGraph* graph);
```
Returns the node whose pose a graph gives: its last.  @param graph  An opened graph. @return The output node; MANI_GRAPH_NONE for NULL. @par Thread safety Safe from any thread.

```c
uint32_t maniGraphNodeSourcePlace(const maniGraph* graph, uint32_t node);
```
Returns a node's place in the source it was built from: the builder orders nodes after their inputs, so a file's node may sit elsewhere than its source's (and its JSON text's) place; subgraphs' nodes are numbered after the source's, as the builder's refusals name them.  @param graph  An opened graph. @param node   The node. @return The place; MANI_GRAPH_NONE for a NULL graph or past the count. @par Thread safety Safe from any thread.

```c
size_t maniGraphBindingSize(const maniGraph* graph);
```
Returns the bytes a binding of the graph needs.  @param graph  An opened graph. @return The size; 0 for a NULL graph. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniBindGraph(const maniGraph* graph, const maniGraphAssets* assets, void* memory, size_t size, maniGraphBinding** bindingOut, uint32_t* unboundOut);
```
Binds a graph to its assets: each map slot to the retargeting map of its identity among the maps, made for the graph's skeleton, and to the map's source skeleton among the source skeletons; each clip slot to the clip of its identity among the clips, of the graph's skeleton (or its map's source) and of the kind the slot was built with; each database slot to the database of its identity among the databases, of the graph's skeleton, whose clips are the slots the graph lists for it and whose channels take the graph's query parameters; each model slot to the learned model of its identity among the models, of the graph's skeleton; each spring set slot to the spring set of its identity among the sets, of the graph's skeleton; a graph's mirror map to the assets' mirror map, of its identity. The graph and its assets must outlive the binding.  @param graph       An opened graph. @param assets      The skeleton the graph was built against, and the clips, databases, models, spring sets, maps, source skeletons and mirror map. @param memory      At least maniGraphBindingSize bytes, MANI_GRAPH_BINDING_ALIGNMENT-aligned. @param size        The memory's size. @param bindingOut  Receives the binding. @param unboundOut  Receives the first slot left unbound on failure: a clip slot, the graph's clip count plus a database slot, the clip and database counts plus a model slot, the clip, database and model counts plus a spring set slot, or those and the spring set count plus a map slot, or those and the map count for the mirror map (MANI_NO_TRACK otherwise). May be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL or misaligned argument, a list NULL with a count, a skeleton that is not the graph's, a slot no asset binds, or clips whose root motion names different bones (a mapped clip's naming the target bone that follows its own alone, none refused); `mani_errorCapacity` when the memory is too small. @par Thread safety Safe from any thread.

```c
size_t maniGraphInstanceSize(const maniGraphBinding* binding);
```
Returns the bytes an instance of a bound graph needs.  @param binding  A binding. @return The size; 0 for a NULL binding. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniInitGraphInstance(const maniGraphBinding* binding, void* memory, size_t size, maniGraphInstance** instanceOut);
```
Makes an instance in caller memory: parameters at their defaults, every playhead at 0, nothing weighed until the first update. The instance holds no pointer into itself, so a `memcpy` of its bytes is a copy.  @param binding      A binding, which must outlive the instance. @param memory       At least maniGraphInstanceSize bytes, MANI_GRAPH_INSTANCE_ALIGNMENT-aligned. @param size         The memory's size. @param instanceOut  Receives the instance. @return `mani_success`; `mani_errorInvalid` for a NULL or misaligned argument; `mani_errorCapacity` when the memory is too small. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSetGraphParameter(maniGraphInstance* instance, uint32_t parameter, maniGraphParameterType type, maniGraphValue value);
```
Sets a parameter: bool and trigger 0 or 1 (b), int (i) and float (f[0]) clamped to their bounds, the vectors (f) finite, a quaternion (f) normalized.  @param instance   An instance. @param parameter  The parameter. @param type       Its type, as the graph declares it. @param value      The value. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a parameter past the count, a type that is not the parameter's, or a value out of form (not finite, a bool past 1, a quaternion of zero length; nothing changed). @par Thread safety Safe from any thread; the instance is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniSetGraphPose(maniGraphInstance* instance, uint32_t parameter, const maniPose* pose);
```
Sets a pose parameter to a pose's local transforms (copied).  @param instance   An instance. @param parameter  A parameter of type maniGraphParameterPose. @param pose       A pose of the graph's skeleton, not a difference. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a parameter past the count or of another type, or a pose of another skeleton or a difference (nothing changed). @par Thread safety Safe from any thread; the instance and pose are used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniGetGraphPose(const maniGraphInstance* instance, uint32_t parameter, maniPose* poseOut);
```
Reads a pose parameter's local transforms into a pose.  @param instance   An instance. @param parameter  A parameter of type maniGraphParameterPose. @param poseOut    A pose of the graph's skeleton; it becomes a pose (not a difference), its model not current. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a parameter past the count or of another type, or a pose of another skeleton. @par Thread safety Safe from any thread; the instance and pose are used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniGetGraphParameter(const maniGraphInstance* instance, uint32_t parameter, maniGraphValue* valueOut);
```
Reads a parameter's value.  @param instance   An instance. @param parameter  The parameter. @param valueOut   Receives the value. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a parameter past the count. @par Thread safety Safe from any thread; the instance is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniInspectGraphNode(const maniGraphInstance* instance, uint32_t node, maniGraphNodeView* out);
```
Reads what a node of an instance did in its last update, for a debugger: its share of the output, playhead, current state and transition under way. Reads only; the update keeps nothing more for it. reserved is 0.  @param instance  An instance. @param node      The node. @param out       The view. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a node past the count. @par Thread safety Safe from any thread; the instance is used by one thread at a time. It reads what the last update left, so it runs between updates, on the thread that updates or after handing the instance over.

```c
size_t maniGraphStateSize(const maniGraphBinding* binding);
```
Returns the bytes an instance's saved state takes: its parameters, its nodes' state, its state machines' and one-shots', and its pose parameters' locals (the skeleton's bones), with the graph's identity.  @param binding  A binding. @return The size; 0 for a NULL binding. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSaveGraphInstance(const maniGraphInstance* instance, void* out, size_t capacity);
```
Writes an instance's state as bytes that hold no pointer, to be restored into any instance of the same graph.  @param instance  An instance. @param out       At least maniGraphStateSize bytes. @param capacity  The bytes out holds. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorCapacity` when capacity is too small (nothing written). @par Thread safety Safe from any thread; the instance is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniFileResult maniRestoreGraphInstance(maniGraphInstance* instance, const void* bytes, size_t size);
```
Restores a saved state, checked as external bytes: the graph's identity, the size, every parameter in form, every node's, state machine's and one-shot's state finite and in range, and every pose parameter's locals finite with rotations of length 1 within a thousandth. The instance's cursors start over (they cache nothing a pose depends on).  @param instance  An instance of the graph the state was saved from. @param bytes     The state. @param size      Its size in bytes. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorFormat` with the offset of the first byte found wrong (the instance unchanged). @par Thread safety Safe from any thread; the instance is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniUpdateGraph(maniGraphInstance* instance, float delta, maniGraphEvent* events, uint32_t capacity, uint32_t* eventCountOut, maniRootMove* rootMotionOut);
```
Moves an instance through time: shares each node's weight down from the output (normalized blends, masks by their largest bone weight, additive layers by their weights, the selected input), moves every clip node's playhead by the step times its speed and the time scales above it (a seek whose trigger is set moving its playheads to its time instead, and consuming the trigger), moves learned nodes' states (stepped, at most MANI_GRAPH_LEARNED_MAX_STEPS steps an update, and projected), lists the events clip nodes and learned nodes' models weighed past MANI_GRAPH_EVENT_THRESHOLD give in node order, and blends the clips' and learned nodes' root motion by the weights the output gives it (masks by their weight at the root motion bone, additive layers none).  @param instance      An instance. @param delta         The step in seconds, finite, either sign. @param events        Receives up to capacity events. May be NULL with a capacity of 0. @param capacity      The events events holds. @param eventCountOut Receives the true count, which may pass the capacity (saturating at UINT32_MAX). May be NULL. @param rootMotionOut Receives the update's root motion. May be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL instance, events NULL with a capacity, or a step not finite (nothing changed). @par Thread safety Safe from any thread; the instance is used by one thread at a time.

```c
size_t maniGraphScratchSize(const maniGraphBinding* binding);
```
Returns the bytes of the scratch an evaluation of a bound graph's instances needs: a pose per slot, room for one node's layers and, with map slots, for carrying one clip's pose (two poses of the largest source skeleton, a pose of the graph's and the transfer's scratch). One scratch serves any number of instances, one at a time.  @param binding  A binding. @return The size; 0 for a NULL binding. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniEvaluateGraph(maniGraphInstance* instance, void* scratch, size_t size, maniPose* pose);
```
Writes an instance's pose as its last update weighed it: each node of weight above 0, in order, into its slot (a clip sampled at its playhead through its cursor; a learned node's model decompressed at its state, taking the hidden values its last update kept when the two states' bytes are equal: maniUpdateGraph writes that cache, the evaluation only reads it; blends, masks and additive layers by the blending functions over their weighed inputs; a select, time scale or seek passing its input on), then the output's. The rest pose before the first update.  @param instance  An instance. @param scratch   At least maniGraphScratchSize bytes, MANI_GRAPH_SCRATCH_ALIGNMENT-aligned, used by one thread at a time. @param size      The scratch's size. @param pose      A pose of the graph's skeleton. @return `mani_success`; `mani_errorInvalid` for a NULL or misaligned argument or a pose of another skeleton; `mani_errorCapacity` when the scratch is too small. @par Thread safety Safe from any thread; the instance, scratch and pose are used by one thread at a time.

```c
size_t maniGraphSubsetSize(const maniGraphBinding* binding);
```
Returns the bytes a subset plan of a binding takes, whatever its bones.  @param binding  A binding. @return The size; 0 for a NULL binding. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniPlanGraphSubset(const maniGraphBinding* binding, const uint32_t* bones, uint32_t count, void* memory, size_t size, maniGraphSubset** planOut);
```
Plans the evaluation of some bones (a server's hit-test bones, a distant crowd's level of detail): the bones listed, every ancestor of them, and every bone the graph's nodes read to write them, to a fixed point: under a mirror map each planned bone's partner; for an IK node writing a planned bone its chain; for a spring node its set's joints, tails, colliders' bones and centers; for a full-body node its paths from its top to its effectors. A node of these kinds writing no planned bone does not run under the plan. Made once, used by any instance of the binding.  @param binding  A binding. @param bones    The bones, strictly increasing; may be NULL when count is 0. @param count    How many. @param memory   At least maniGraphSubsetSize bytes, MANI_GRAPH_SUBSET_ALIGNMENT-aligned. @param size     The memory's size. @param planOut  The plan, in memory. @return `mani_success`; `mani_errorInvalid` for a NULL or misaligned argument, bones out of range or not increasing; `mani_errorCapacity` when the memory is too small. @par Thread safety Safe from any thread; the memory is used by one thread at a time.

```c
const uint32_t* maniGraphSubsetBones(const maniGraphSubset* plan, uint32_t* countOut);
```
Returns the bones a plan evaluates and writes: the listed bones, their ancestors and what the graph's nodes read to write them, strictly increasing.  @param plan      A plan. @param countOut  Their count; 0 for a NULL plan. @return The bones, in the plan's memory; NULL for a NULL plan. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniEvaluateGraphSubset(maniGraphInstance* instance, const maniGraphSubset* plan, void* scratch, size_t size, maniPose* pose);
```
Writes the plan's bones (maniGraphSubsetBones: with the listed ones' ancestors, so their model pass is right) of an instance's pose as maniEvaluateGraph writes them, bit for bit, computing no more than the plan needs; every other bone is left as it was. A node of a kind that writes bones from others and writes no planned bone passes its input on (a spring set not stepped), and an inertialization keeps its offsets and output outside the plan's groups as they were, so bones joining a later plan start from what they last had; a turn limit node steps the planned bones only, the others turning from what they last had when an evaluation next computes them. Retargeted clips are transferred whole.  @param instance  An instance. @param plan      A plan of the instance's binding. @param scratch   As maniEvaluateGraph's. @param size      The scratch's size. @param pose      A pose of the graph's skeleton. @return As maniEvaluateGraph; `mani_errorInvalid` also for a NULL plan or one of another binding. @par Thread safety Safe from any thread; the instance, scratch and pose are used by one thread at a time.

## `graphbuild.h`

Building a graph file (mani-0023), offline: a graph's parameters, clip slots, sync groups and nodes, checked against one skeleton and its clips, ordered so each node follows its inputs, its poses given slots, and written.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildGraph(maniBuilder* builder, const maniGraphSource* source);
```
Builds a graph file: inlines subgraphs (their nodes after the source's, in order, nested ones after them; refusals name nodes by their place there), checks every parameter, clip slot, sync group, database slot, model slot, spring set slot and node, gives each retargeting map the clip slots name a map slot, orders the nodes so each follows its inputs (the output last, the source's order kept where it may be), makes each mask's bone weights, marks each node as giving poses or differences and as mirrored or not (the parity of the mirror nodes above it), gives each node's pose a slot reused once its last reader has run, and keeps the file in place of any built before.  @param builder  The builder. @param source   The graph. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse) or with the reason, the item and another the reason names for a refused source: Count (more items than the named limits, or none of nodes), Name, DuplicateName and NameHash with the list bits of parameters, clips, sync groups, databases, models or spring sets, Parameter, Clip, Database, Model, SpringSetSlot, Node, Input, Cycle, Unreachable, Scalar, Weight, Difference, MirrorGraph and MirrorParity (a node under both a mirrored and an unmirrored path, item the node, other the reader reaching it the other way: give each path its own copy of the nodes beneath); and `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniWriteGraphJson(maniBuilder* builder, const maniGraphSource* source);
```
Writes a graph source as its JSON text form (UTF-8, two-space indents, a line per list item, ending in a new line), building it first: a source maniBuildGraph refuses is refused the same way and not written. Parameters, clip slots, sync groups, database slots (a "databases" list, when the graph has any), model slots (a "models" list, when it has any), spring set slots (a "springSets" list, when it has any) and the bones of mask chains are written by name, nodes and the output by their place in the source, subgraphs as nested graphs with their parameters mapped by name, and numbers as the shortest text that reads back to the same binary32 (ECMAScript's layout). The text is kept in place of any file or text kept before; maniCopyBuilt copies it.  @param builder  The builder. @param source   The graph. @return As maniBuildGraph. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildGraphJson(maniBuilder* builder, const char* text, size_t length, const maniGraphNamedAssets* assets);
```
Builds a graph file from its JSON text form, as maniWriteGraphJson writes it: members an object does not take are refused; those with an off value may be left out (a parameter's default: false, 0, the zero vector or the identity quaternion, and none for a pose parameter; bounds, both or neither; allEvents false; a clip node's syncGroup null, speed 1, loop "clip", leader false; a blend's direct false; a mask's chains empty and defaultWeight 0; a chain's subtree false; a seek's time 0; a 1D space's period 0, a banded space's polar false, a triangulated space's triangles none (its Delaunay triangulation), a position's y 0; a machine's entry 0, chainLimit 0 (4), transitions and finals empty; a transition's priority 0, interruption "none", entry "restart", fade "linear", duration and cooldown 0 and conditions empty; a one-shot's fadeOutTrigger and abort none, curve "linear", fadeIn and fadeOut 0, mask none (the whole body); an inertialize node's trigger none, method "inertialization", duration 0 and, for "deadBlending", halfLifeScale 0.3, halfLifeMin 0.1 and halfLifeMax 1.0; a motion matching node's trigger none, query empty, period 0.1, jumpThreshold, endExclusion, historyTime, continuingBias, transitionCost and historyCount 0, requiredTags and excludedTags 0 (a whole number, or {"parameter": name}) and duration 0; parameters, clips, syncGroups, databases, models and springSets empty). Each clip slot name, at any nesting, takes the host's clip of that name (and its map), each database slot name the host's database of that name, each model slot name the host's model of that name and each spring set slot name the host's spring set of that name (the first of equal names); every graph read takes the host's mirror map. The source read is then built as maniBuildGraph builds it.  @param builder    The builder. @param text       The UTF-8 text (a leading byte order mark skipped); may be NULL when length is 0. @param length     The text's length in bytes. @param assets     The graph's skeleton (mask chains name its bones), the clips, databases, models and spring sets with their slot names, and the mirror map. @return As maniBuildGraph, items by place in the text's lists; or `mani_errorInvalid` with maniBuildReasonJson or maniBuildReasonForm (other a maniGraphForm), item the byte offset. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

## `ik.h`

Inverse kinematics on a pose (mani-0029, mani-0030): a two-bone chain bent to reach a goal and turned toward a pole; a bone aimed at a goal, or a chain of up to MANI_AIM_MAX_BONES sharing the turn (a look-at over spine, neck and head), its up toward a pole; a longer chain bent by CCD or FABRIK within its bones' limits. A solver reads the pose's locals, composes the model frames of its bones' root path itself (the pose's model need not be current), writes the corrected locals of the bones it turns and leaves the model stale. Goals and poles are in the model space of the pose; axes fixed to a bone are in that bone's local frame. Stateless: the result is a function of the pose and the inputs.

```c
MANI_NODISCARD MANI_API maniResult maniSolveTwoBone(maniPose* pose, const maniSkeleton* skeleton, const maniTwoBoneSolve* solve, bool* reachedOut);
```
Bends a two-bone chain of a pose toward a goal: the middle bone turns about its hinge so the chain spans the goal's distance (a goal out of reach straightens it, one closer than the bones' difference folds it), then the start bone turns the chain onto the goal, its bend toward the pole, twisted. A goal at the start turns nothing at the start; a bone of no length turns nothing at the middle. The solve on its own result changes nothing.  @param pose       A pose of the skeleton (not a difference); the start and middle bones' locals are written. @param skeleton   Its skeleton. @param solve      The solve. @param reachedOut Receives whether the goal is reached at full weight (within the softened reach and further than the bones' difference); may be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL pose, skeleton or solve, a pose of another skeleton or a difference, bones out of range or not each an ancestor of the next, a goal, pole, hinge or twist not finite, a soften or weight outside [0, 1]. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniSolveAim(maniPose* pose, const maniSkeleton* skeleton, const maniAimSolve* solve, bool* reachedOut);
```
Aims a bone of a pose at a goal: its forward from its offset point turned onto the goal by the shortest arc, then about that line so its up axis lies in the plane of the line and the pole, then by the twist; with more bones each turns its share. A cone stops the forward at its edge. A goal at the aiming bone, or nearer to it than the offset lies from the forward line, turns nothing.  @param pose       A pose of the skeleton (not a difference); the aim's bones' locals are written. @param skeleton   Its skeleton. @param solve      The aim. @param reachedOut Receives whether the forward points at the goal at full weight (not stopped by the cone, the goal reachable); may be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL pose, skeleton or solve, a pose of another skeleton or a difference, a count of 0 or past MANI_AIM_MAX_BONES, bones out of range or not each an ancestor of the next, shares negative, not finite or all 0, a forward not of length 1 within a thousandth, a goal, pole, offset, up or twist not finite, a cone outside [0, MANI_AIM_MAX_CONE], a weight outside [0, 1]. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniSolveCcd(maniPose* pose, const maniSkeleton* skeleton, const maniChainSolve* solve, bool* reachedOut);
```
Bends a chain of a pose toward a goal by cyclic coordinate descent: each iteration sweeps from the end's parent to the root, each bone turning by the shortest arc that takes its direction to the end onto its direction to the goal (cut to the step bound), then limited. Stops early once the end lies within the tolerance; then the end bone turns toward the wanted rotation by its weight. A bone whose origin lies on the end or the goal turns nothing.  @param pose       A pose of the skeleton (not a difference); the chain's turning bones' and the end's locals are written. @param skeleton   Its skeleton. @param solve      The solve. @param reachedOut Receives whether the end lies within the tolerance of the goal at full weight; may be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL pose, skeleton or solve, a pose of another skeleton or a difference, bones out of range, a root not an ancestor of the end or a path past MANI_IK_CHAIN_MAX_BONES, a limit of an unknown kind or out of range or a hinge along its bone's axis, a goal, pole or rotation not finite, a pole not zero, a rotation of length 0, iterations out of range, a tolerance below 0 or not finite, a step outside [0, pi], a rotation weight or weight outside [0, 1]. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniSolveFabrik(maniPose* pose, const maniSkeleton* skeleton, const maniChainSolve* solve, bool* reachedOut);
```
Bends a chain of a pose toward a goal by FABRIK (Aristidou and Lasenby): with a pole, the chain first turns about the root-to-goal line so its bend faces the pole (blended near the line); each iteration places the end on the goal and each joint at its bone's length from the one below it, back to the root, then from the root turns each bone by the shortest arc onto its joint's place (its twist kept), limited, the next joint where the turned bone puts it. Stops early once the end lies within the tolerance; then the end bone turns toward the wanted rotation by its weight.  @param pose       A pose of the skeleton (not a difference); the chain's turning bones' and the end's locals are written. @param skeleton   Its skeleton. @param solve      The solve; its step must be 0. @param reachedOut Receives whether the end lies within the tolerance of the goal at full weight; may be NULL. @return `mani_success`; `mani_errorInvalid` as maniSolveCcd, but for a pole other than zero, which FABRIK takes, and a step other than 0, which it refuses. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

## `import.h`

glTF 2.0 import (mani-0008), offline: a document opened from JSON or GLB bytes the caller keeps alive, its external buffers supplied by the host, then skeletons, skins and clips built from it.

```c
static inline maniImportClipOptions maniDefaultImportClipOptions(void);
```

```c
static inline maniGltfOptions maniDefaultGltfOptions(void);
```

```c
MANI_NODISCARD MANI_API maniImportResult maniOpenGltf(maniBuilder* builder, const void* bytes, size_t size, maniGltf** gltfOut);
```
Opens a glTF document from JSON or GLB bytes (told apart by GLB's magic), checking the container, the asset version, required extensions, buffers, buffer views and accessors. `data:` buffers are decoded into the document; external buffers wait for maniGltfSupplyBuffer. Buffer views compressed by `EXT_meshopt_compression` or `KHR_meshopt_compression` are decoded into the document once their streams' buffers are present (here for GLB and `data:` ones); a refused stream gives maniImportReasonData at its offset in its buffer, the view as the item.  @param builder  A builder; the document uses its allocator and must be closed before the builder is destroyed. @param bytes    The file; kept alive and unchanged while the document is open. @param size     Its size in bytes. @param gltfOut  Receives the document. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorFormat` with a reason, an offset and an item; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
void maniCloseGltf(maniGltf* gltf);
```
Closes a document and frees its memory. NULL is allowed.  @param gltf  The document. @par Thread safety Safe from any thread; the document is used by one thread at a time.

```c
uint32_t maniGltfBufferCount(const maniGltf* gltf);
```
Returns the number of buffers.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
const char* maniGltfBufferUri(const maniGltf* gltf, uint32_t buffer, uint32_t* lengthOut);
```
Returns an external buffer's URI, as the JSON gives it after its escapes (a URI reference relative to the document; percent escapes are left to the host, which resolves it), or NULL for a buffer the document holds (GLB or `data:`), one it never reads (a meshopt fallback, or a buffer without a URI that only compressed views use) or past the count.  @param gltf       The document. @param buffer     The buffer. @param lengthOut  Receives the URI's length in bytes, 0 for NULL. May be NULL. @return The URI, UTF-8 and not terminated, or NULL. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfSupplyBuffer(maniGltf* gltf, uint32_t buffer, const void* bytes, size_t size);
```
Supplies an external buffer's bytes, which the caller keeps alive and unchanged while the document is open. They must hold the buffer's `byteLength`; bytes past it are ignored. Meshopt streams in them are decoded into the document; when one is refused the buffer is left unsupplied.  @param gltf    The document. @param buffer  An external buffer. @param bytes   Its bytes. @param size    Their size. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a buffer that is not external; `mani_errorFormat` with maniImportReasonBuffer for too few bytes, or maniImportReasonData for a refused stream; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the document is used by one thread at a time.

```c
uint32_t maniGltfSkeletonCount(const maniGltf* gltf);
```
Returns the number of skeletons the document's skins make: skins sharing any node of their joints' hierarchy share one, numbered by their lowest skin.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfSkinCount(const maniGltf* gltf);
```
Returns the number of skins.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfSkinSkeleton(const maniGltf* gltf, uint32_t skin);
```
Returns the skeleton a skin is part of.  @param gltf  The document. @param skin  The skin. @return The skeleton, or UINT32_MAX for a skin past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfNodeCount(const maniGltf* gltf);
```
Returns the number of nodes.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildSkeleton(maniBuilder* builder, const maniGltf* gltf, uint32_t skeleton, const maniGltfOptions* options);
```
Builds a skeleton of the document into the builder: its skins' joints, their common root and the nodes between, named by their nodes' names (renamed `name#node` where missing or repeated).  @param builder   The builder. @param gltf      The document. @param skeleton  The skeleton, below maniGltfSkeletonCount. @param options   The options. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skeleton past the count or options out of range; `mani_errorFormat` with a reason; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildNodeSkeleton( maniBuilder* builder, const maniGltf* gltf, uint32_t node, const maniGltfOptions* options);
```
Builds a node's subtree as a skeleton, for rigid hierarchies.  @param builder  The builder. @param gltf     The document. @param node     The root node. @param options  The options. @return As maniGltfBuildSkeleton. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfSkeletonPlacement(const maniGltf* gltf, uint32_t skeleton, const maniGltfOptions* options, maniAffine* placementOut);
```
Returns a skeleton's placement: the product of the local transforms of the nodes above its root, the basis change applied; identity when the options fold it into the root.  @param gltf          The document. @param skeleton      The skeleton. @param options       The options. @param placementOut  Receives the placement. @return `mani_success`; `mani_errorInvalid` as maniGltfBuildSkeleton; `mani_errorFormat` for a node above the root whose matrix is refused. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildSkin(maniBuilder* builder, const maniGltf* gltf, uint32_t skin, const maniSkeleton* skeleton, const maniGltfOptions* options);
```
Builds a skin into the builder against an opened skeleton: each joint found by its bone name, the inverse binds read (identity when the skin has none) and the basis change applied.  @param builder   The builder. @param gltf      The document. @param skin      The skin. @param skeleton  An opened skeleton holding the skin's joints. @param options   The options the skeleton was built with. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skin past the count or options out of range; `mani_errorFormat` with a reason; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
const char* maniGltfNodeBoneName(const maniGltf* gltf, uint32_t node, uint32_t* lengthOut);
```
Returns the bone name a node has in every skeleton, skin and clip built from the document (mani-0009): its name, or, when it has none or an earlier node has the same, that name followed by `#` and the node index.  @param gltf       The document. @param node       The node. @param lengthOut  Receives the length in bytes, 0 past the count. May be NULL. @return The name, UTF-8 and not terminated, or NULL past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfRenamedCount(const maniGltf* gltf);
```
Returns the number of nodes whose bone name is not their name as given: unnamed, or named as an earlier node. Their names follow the node index, so a host binding by name across exports can warn or keep its own map.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfRenamedNode(const maniGltf* gltf, uint32_t index);
```
Returns a renamed node, in node order.  @param gltf   The document. @param index  Below maniGltfRenamedCount. @return The node, or UINT32_MAX past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfAnimationCount(const maniGltf* gltf);
```
Returns the number of animations.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
const char* maniGltfAnimationName(const maniGltf* gltf, uint32_t animation, uint32_t* lengthOut);
```
Returns an animation's name, UTF-8 and not terminated; empty for an animation without one.  @param gltf       The document. @param animation  The animation. @param lengthOut  Receives the length in bytes, 0 past the count. May be NULL. @return The name, or NULL past the count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildClip(maniBuilder* builder, const maniGltf* gltf, uint32_t animation, const maniSkeleton* skeleton, const maniGltfOptions* options, const maniImportClipOptions* clipOptions, uint32_t* boundOut);
```
Builds one animation as a clip of an opened skeleton (mani-0010): translation, rotation and scale channels on its bones (found by name), morph weights of its bones and of meshes it skins, and `KHR_animation_pointer` channels to anything but nodes, each as float tracks; the basis change applied and, with placement folded, the nodes above the root composed in. The duration is the animation's latest key over every channel. The clip's root motion, if any, is taken out as maniBuildClip takes it.  @param builder     The builder. @param gltf        The document, its buffers supplied. @param animation   The animation. @param skeleton    An opened skeleton. @param options     The options the skeleton was built with. @param clipOptions The clip's settings; sample rate 0 for 30. @param boundOut    Receives how many of the animation's channels the clip holds. May be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL argument, an animation past the count, options out of range, or a clip setting out of range (a rate not finite or negative, a loop, basis or root motion source field out of range, a root motion bone past the skeleton); `mani_errorFormat` with a reason (Data with the channel as item for keys the clip builder refuses; Build with the build reason as item for anything else it refuses, such as a root motion bone the animation leaves without a track); `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfReadSprings(maniGltf* gltf, const maniSkeleton* skeleton, const maniGltfOptions* options, maniSpringSetSource* setOut);
```
Reads the document's `VRMC_springBone` extension (version 1.0) as spring bone sources on an opened skeleton (mani-0011): colliders, collider groups and springs, nodes found by bone name, offsets, tails and gravity directions in the output's basis. Colliders' `VRMC_springBone_extended_collider` and joints' `VRMC_springBone_limit` (version 1.0; another version is ignored) give inside spheres and capsules, planes and limits, a limit's rotation such that its frame is glTF's in the output's basis (mani-0026); a last joint's limit is ignored. A document without the extension gives an empty set. The arrays are the document's until the next call or maniCloseGltf.  @param gltf      The document. @param skeleton  An opened skeleton holding the springs' nodes. @param options   The options the skeleton was built with. @param setOut    Receives the sources. @return `mani_success`; `mani_errorInvalid` for a NULL argument or options out of range; `mani_errorFormat` with a reason; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the document is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfReadRagdoll(maniGltf* gltf, const maniSkeleton* skeleton, const maniGltfOptions* options, maniVec3 facing, maniVec3 up, maniRagdollSource* sourceOut);
```
Reads the document's `OMI_physics_body`, `OMI_physics_shape` and `OMI_physics_joint` extensions as a ragdoll profile's source on an opened skeleton (mani-0027). Each dynamic or kinematic body sits on the bone of its node or of the node's nearest ancestor that is a bone, its frame the frame of its one collider's node (rigid, in the bone's model frame with scale left out), its shape that collider's (a box's size halved, a sphere's radius, a capsule's height and radius) times the node's scale, its mass the motion's. Each joint joins a body to its nearest ancestor body: its frame the child side's node's, every linear axis fixed, angular limits as twist (x) and symmetric swings (y, z), and angular force-mode drives, equal on their axes and without targets, as the drive. The root body's facing and up axes are facing and up (glTF's basis, unit and perpendicular) turned into its frame at rest. Inertia, initial velocities, triggers, collision settings and materials are ignored. A document without bodies on the skeleton gives an empty source. The arrays are the document's until the next call or maniCloseGltf.  @param gltf       The document. @param skeleton   An opened skeleton holding the bodies' bones. @param options    The options the skeleton was built with. @param facing     The character's facing (out of the chest) in glTF's basis; +Z for glTF's convention. @param up         Its up axis in glTF's basis; +Y for glTF's. @param sourceOut  Receives the source. @return `mani_success`; `mani_errorInvalid` for a NULL argument, options out of range or axes not unit and perpendicular within a thousandth; `mani_errorFormat` with a reason (Unsupported for what a profile cannot hold); `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the document is used by one thread at a time.

```c
uint32_t maniGltfMeshCount(const maniGltf* gltf);
```
Returns the number of meshes in the document.  @param gltf  The document. @return The mesh count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfPrimitiveCount(const maniGltf* gltf, uint32_t mesh);
```
Returns the number of primitives a mesh has.  @param gltf  The document. @param mesh  The mesh. @return The primitive count; 0 for a mesh past the count or one whose primitives are not an array. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfPrimitiveTargetCount(const maniGltf* gltf, uint32_t mesh, uint32_t primitive);
```
Returns the number of morph targets a mesh primitive has.  @param gltf       The document. @param mesh       The mesh. @param primitive  The primitive. @return The target count; 0 for none or past a count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfNodeMesh(const maniGltf* gltf, uint32_t node);
```
Returns the mesh a node instances.  @param gltf  The document. @param node  The node. @return The mesh, or UINT32_MAX for none or a node past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfNodeSkin(const maniGltf* gltf, uint32_t node);
```
Returns the skin that deforms a node's mesh.  @param gltf  The document. @param node  The node. @return The skin, or UINT32_MAX for none or a node past the count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfMorphWeights(const maniGltf* gltf, uint32_t node, float* weightsOut, uint32_t count);
```
Reads the morph weights a node's mesh starts with: the node's `weights`, else its mesh's, else zeros.  @param gltf        The document. @param node        The node. @param weightsOut  Receives count weights. @param count       The mesh's morph target count. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a node past the count or a count other than its mesh's targets; `mani_errorFormat` (Property) for weights not an array of that many numbers. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildInfluences(maniBuilder* builder, const maniGltf* gltf, uint32_t mesh, uint32_t primitive, const maniSkin* skin);
```
Builds a primitive's influences into the builder, bound to an opened skin built from the glTF skin that deforms it (its joints in the skin's order): every `JOINTS_n` and `WEIGHTS_n` set read, zero weights dropped, then sorted and renormalized as maniBuildInfluences does. A primitive `KHR_draco_mesh_compression` compresses has its stream decoded: the sets the extension names are read from it, in the stream's vertex order, each accessor matching the decoded attribute (its count, component type and component count; its `normalized` decides the values, Draco's own flag is not read); the others are read from their views.  @param builder    The builder. @param gltf       The document. @param mesh       The mesh. @param primitive  The primitive. @param skin       The opened skin. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a mesh or primitive past its count; `mani_errorFormat` with a reason: Property (no `JOINTS_0`, sets not paired, an accessor of the wrong type or count; a Draco extension without its view or attributes, a name it gives not among the primitive's, a unique id not in the stream, an accessor not matching its attribute), Influence, Data (a Draco stream the decoder refuses, at its view), Pending, Build; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildMorphTargets(maniBuilder* builder, const maniGltf* gltf, uint32_t mesh, uint32_t primitive, const maniGltfOptions* options);
```
Builds a primitive's morph targets into the builder: a channel a target, at full weight 1, named by the mesh's `extras.targetNames` when it names every target and by the target's index otherwise (weight tracks are the node's bone name, a slash and that name); positions, normals, tangents, texture coordinate and color sets read (other semantics left), positions, normals and tangents in the output's basis, VEC3 colors with alpha 0; a target keeps the vertices it moves. A primitive `KHR_draco_mesh_compression` compresses is read the same way: Draco never holds its targets.  @param builder    The builder. @param gltf       The document. @param mesh       The mesh. @param primitive  The primitive. @param options    The options the skeleton was built with. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a mesh or primitive past its count, a primitive without targets or options out of range; `mani_errorFormat` with a reason: Property (a semantic the primitive lacks, an accessor of the wrong type or count, more than 32 attributes), Index, Data, Pending, Build; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniOpenFbx(maniBuilder* builder, const void* bytes, size_t size, maniFbx** fbxOut);
```
Opens an FBX document (mani-0012) from the bytes of a file, binary (told apart by its magic, either byte order) or ASCII, versions 3000 to 7700. Every record, token and length is checked against the bytes; no array is decoded and no number converted until asked for.  @param builder  The builder whose allocator the document uses; it must be closed before the builder is destroyed. @param bytes    The file; kept alive and unchanged while the document is open. @param size     Its size in bytes. @param fbxOut   Receives the document. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorFormat` with a reason (Container, Version, Syntax) and the offset of the first byte found wrong; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
void maniCloseFbx(maniFbx* fbx);
```
Closes an FBX document and frees its memory. NULL is allowed.  @param fbx  The document. @par Thread safety Safe from any thread; the document is used by one thread at a time.

```c
uint32_t maniFbxVersion(const maniFbx* fbx);
```
Returns the document's version (7400 for FBX 7.4), 0 for NULL.  @param fbx  The document. @return The version. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxModelCount(const maniFbx* fbx);
```
Returns the number of models (the objects of kind Model, in the order of the file's `Objects`), 0 for NULL.  @param fbx  The document. @return The count. @par Thread safety Safe from any thread.

```c
const char* maniFbxModelBoneName(const maniFbx* fbx, uint32_t model, uint32_t* lengthOut);
```
Returns the bone name a model has in every skeleton, skin and clip built from the document (mani-0014): its name without its class, or, when it has none or an earlier model has the same, that name followed by `#` and the model index.  @param fbx        The document. @param model      The model. @param lengthOut  Receives the length in bytes, 0 past the count. May be NULL. @return The name, UTF-8 and not terminated, or NULL past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxRenamedCount(const maniFbx* fbx);
```
Returns the number of models whose bone names differ from their names, 0 for NULL.  @param fbx  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxRenamedModel(const maniFbx* fbx, uint32_t index);
```
Returns a renamed model, in model order.  @param fbx    The document. @param index  Below maniFbxRenamedCount. @return The model, or UINT32_MAX past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxSkeletonCount(const maniFbx* fbx);
```
Returns the number of skeletons (mani-0014): skins' linked models and limb chains, joined where they share a model, numbered by their lowest skin, then chain; 0 for NULL.  @param fbx  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxSkinCount(const maniFbx* fbx);
```
Returns the number of skins (deformers of subclass Skin, in object order), 0 for NULL.  @param fbx  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxSkinSkeleton(const maniFbx* fbx, uint32_t skin);
```
Returns the skeleton a skin is part of.  @param fbx   The document. @param skin  The skin. @return The skeleton, or UINT32_MAX for a skin past the count or one linking no model. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxSkinModel(const maniFbx* fbx, uint32_t skin);
```
Returns the model whose mesh a skin deforms: the skin's geometry's model (its own parent in 6.x files).  @param fbx   The document. @param skin  The skin. @return The model, or UINT32_MAX for a skin past the count or one with no mesh model. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniFbxReadSettings(const maniFbx* fbx, maniFbxSettings* settingsOut);
```
Reads the scene's settings, FBX's defaults where the file is silent: X right, Y up, Z front, a centimetre a unit, 30 frames a second.  @param fbx          The document. @param settingsOut  Receives the settings. @return `mani_success`; `mani_errorInvalid` for a NULL argument. @par Thread safety Safe from any thread.

```c
static inline maniFbxOptions maniDefaultFbxOptions(void);
```

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxBuildSkeleton(maniBuilder* builder, const maniFbx* fbx, uint32_t skeleton, const maniFbxOptions* options);
```
Builds a skeleton of the document into the builder (mani-0014): its skins' linked models and limb chains, the models between and their common root, named by their bone names, with helper bones (`name#unscaled`) where FBX's scale inheritance needs them.  @param builder   The builder. @param fbx       The document. @param skeleton  The skeleton, below maniFbxSkeletonCount. @param options   The options. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skeleton past the count or options out of range; `mani_errorFormat` with a reason; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxBuildModelSkeleton( maniBuilder* builder, const maniFbx* fbx, uint32_t model, const maniFbxOptions* options);
```
Builds a model's subtree as a skeleton, for rigid hierarchies.  @param builder  The builder. @param fbx      The document. @param model    The root model, below maniFbxModelCount. @param options  The options. @return As maniFbxBuildSkeleton. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxSkeletonPlacement(const maniFbx* fbx, uint32_t skeleton, const maniFbxOptions* options, maniAffine* placementOut);
```
Returns where a skeleton's root hangs: the world of the model above it (its unscaled world when the root does not inherit its scale normally), the options' axes and unit applied; identity when the root is at the top or the options fold the placement.  @param fbx           The document. @param skeleton      The skeleton. @param options       The options. @param placementOut  Receives the placement. @return `mani_success`; `mani_errorInvalid` as maniFbxBuildSkeleton; `mani_errorFormat` for a model above the root whose properties are refused. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxBuildSkin(maniBuilder* builder, const maniFbx* fbx, uint32_t skin, const maniSkeleton* skeleton, const maniFbxOptions* options);
```
Builds a skin into the builder against an opened skeleton (mani-0015): each cluster linking a model a joint, found by the model's bone name, its inverse bind the mesh relative to the bone at bind (the cluster's matrices, a bind pose or the rest, as ufbx reads them) times the mesh's geometric transform, the options' axes and unit applied. Vertex weights are the mesh's to carry.  @param builder   The builder. @param fbx       The document. @param skin      The skin, below maniFbxSkinCount. @param skeleton  An opened skeleton holding the skin's joints. @param options   The options the skeleton was built with. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skin past the count or options out of range; `mani_errorFormat` with a reason (Bone for a joint the skeleton lacks, Matrix for a bind that is not affine or not invertible, Property for an additive cluster); `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
maniSkinningMethod maniFbxSkinMethod(const maniFbx* fbx, uint32_t skin);
```
Returns how a skin blends its joints, as ufbx evaluates it: dual quaternion for SkinningType DualQuaternion, blended for Blend or a skin that gives per-vertex blend weights, linear otherwise (Rigid among them).  @param fbx   The document. @param skin  The skin. @return The method; linear for a skin past the count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxBuildInfluences(maniBuilder* builder, const maniFbx* fbx, uint32_t skin, const maniSkin* built);
```
Builds a skin's influences into the builder, a vertex a control point of its mesh, bound to the opened skin maniFbxBuildSkin built from it (a cluster's index its joint): every cluster's Indexes and Weights, indices past the control points skipped, a vertex given twice by one cluster summed, then sorted and renormalized as maniBuildInfluences does; blend weights (BlendIndexes and BlendWeights, 1 for a dual quaternion skin, 0 else) unless the method is linear.  @param builder  The builder. @param fbx      The document. @param skin     The skin. @param built    The opened skin. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a skin past the count; `mani_errorFormat` with a reason: Property (no control points, Indexes and Weights of different counts), Influence (a weight below 0, a blend weight outside 0 to 1), Data, Build; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
uint32_t maniFbxMorphCount(const maniFbx* fbx);
```
Returns how many sets of morph targets the document has: its BlendShape deformers, in file order, then (before 7.2) the meshes holding Shape children.  @param fbx  The document; NULL gives 0. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxMorphModel(const maniFbx* fbx, uint32_t morph);
```
Returns the mesh model a set of morph targets deforms; its bone name begins the clips' weight tracks for it (`<mesh>/<target>`).  @param fbx    The document. @param morph  The set. @return The model, or UINT32_MAX for none or a set past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxMorphTargetCount(const maniFbx* fbx, uint32_t morph);
```
Returns the number of targets a set of morph targets has: its channels' shapes, but a shape after another of the same full weight (whose weight track is always 0).  @param fbx    The document. @param morph  The set. @return The count; 0 past the count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxBuildMorphTargets( maniBuilder* builder, const maniFbx* fbx, uint32_t morph, const maniFbxOptions* options);
```
Builds a set of morph targets into the builder, as ufbx reads them: a channel a BlendShapeChannel (a Shape child before 7.2), its shapes the targets at FullWeights / 100 (as the clips order them), a vertex a control point; positions and normals as offsets in the options' axes (positions in its unit too), indices past the control points skipped, Blender's per-vertex weights folded in; a shape after another of the same full weight left out, as is a channel without shapes. Channels are named as their objects (one repeating an earlier one's name followed by '#' and its index); a target as its shape, else as its channel with '/' and its index past the first, as weight tracks are.  @param builder  The builder. @param fbx      The document. @param morph    The set. @param options  The options the skeleton was built with. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a set past the count or options out of range; `mani_errorFormat` with a reason: Property (no control points, a shape's arrays of mismatched sizes), Data, Build (a full weight of 0, or a name not UTF-8; the item the builder's reason); `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxMorphWeights(const maniFbx* fbx, uint32_t morph, float* weightsOut, uint32_t count);
```
Reads the target weights a set of morph targets has at rest: each channel's DeformPercent walked through its shapes, as the clips' weight tracks are.  @param fbx         The document. @param morph       The set. @param weightsOut  Receives count weights. @param count       The set's target count. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a set past the count or another count; `mani_errorFormat` (Data) for a FullWeights value not finite; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxClipCount(const maniFbx* fbx);
```
Returns how many clips the document has: its animation stacks (takes), in file order (mani-0016).  @param fbx  The document; NULL gives 0. @return The count. @par Thread safety Safe from any thread.

```c
const char* maniFbxClipName(const maniFbx* fbx, uint32_t clip, uint32_t* lengthOut);
```
Returns a clip's name: its stack's, without the class, UTF-8 and not terminated.  @param fbx        The document. @param clip       The clip. @param lengthOut  Receives the length in bytes, 0 past the count. May be NULL. @return The name, or NULL past the count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxBuildClip(maniBuilder* builder, const maniFbx* fbx, uint32_t clip, const maniSkeleton* skeleton, const maniFbxOptions* options, const maniImportClipOptions* clipOptions, uint32_t* boundOut);
```
Builds a clip into the builder for an opened skeleton (mani-0016): the stack's layers and curves evaluated as ufbx evaluates them, at every sample of the clip's grid from the stack's start (its local, reference or take times, else its keys), whole ticks apart. Each bone of the skeleton named as a model (or a model's helper, `name#unscaled`) takes its local as a skeleton built from the file formed its rest, the options' axes and unit applied and, with placement folded, the placement composed into the root; a channel that never leaves the skeleton's rest is left out. Other numbers a bone's model animates become float tracks named `bone/property` (`/0` to `/2` added for vectors). The clip's root motion, if any, is taken out as maniBuildClip takes it (its bone's track then keeps translation and rotation).  @param builder     The builder. @param fbx         The document. @param clip        The clip, below maniFbxClipCount. @param skeleton    An opened skeleton. @param options     The options the skeleton was built with. @param clipOptions The clip's settings; sample rate 0 for the scene's frame rate up to 120, else 30. @param boundOut    Receives how many of the stack's channel bindings drive models the clip reads. May be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a clip past the count, options out of range, or a clip setting out of range (as maniGltfBuildClip); `mani_errorFormat` with a reason (Property with the clip as item for a stack's curves or channels refused, or with the model for an animated integer input out of range; Matrix with the model for a local that does not decompose at a sample; Name with the model for a repeated float track name; Build with the build reason as item for a timing, basis or root motion the builder refuses); `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxModelPlacement(const maniFbx* fbx, uint32_t model, const maniFbxOptions* options, maniAffine* placementOut);
```
Returns where a model skeleton's root hangs, as maniFbxSkeletonPlacement.  @param fbx           The document. @param model         The root model. @param options       The options. @param placementOut  Receives the placement. @return As maniFbxSkeletonPlacement. @par Thread safety Safe from any thread.

## `learned.h`

Learned motion matching (mani-0025): a model of three networks that stands in for a motion database and its clips. The Projector maps a query to the state of the nearest frame it learned (its features and latent values), the Stepper moves a state one step on, and the Decompressor gives the pose, the root's motion, the float tracks and the events' states of a state. Models are built offline and opened in place; every evaluation gives the same bits on every target.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenLearnedModel(const void* bytes, size_t size, maniLearnedModel* modelOut);
```
Opens a model file in place: checks the container, every record, count and float, that each network's layers chain and fit its role (the Decompressor from a state to the outputs, the Stepper from a state to a state's rate of change, each Projector from features to a state), and the names.  @param bytes     The file, 16-byte aligned. @param size      Its size in bytes; bytes past the file are ignored. @param modelOut  Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint64_t maniLearnedModelIdentity(const maniLearnedModel* model);
```
Returns a model's identity: the hash of its sections.  @param model  An opened model, or NULL. @return The identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
uint64_t maniLearnedModelSkeletonIdentity(const maniLearnedModel* model);
```
Returns the identity of the skeleton a model was built for.  @param model  An opened model, or NULL. @return The identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
uint64_t maniLearnedModelDatabaseIdentity(const maniLearnedModel* model);
```
Returns the identity of the motion database a model learned.  @param model  An opened model, or NULL. @return The identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniLearnedModelChannel(const maniLearnedModel* model, uint32_t channel, maniMatchChannel* channelOut);
```
Reads a channel of the database a model learned (a query is laid out as that database's features).  @param model       An opened model. @param channel     The channel's index. @param channelOut  Receives the channel. @return `mani_success`; `mani_errorInvalid` for a NULL argument or an index past the channels. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniLearnedModelNormalization(const maniLearnedModel* model, float* offsetsOut, float* scalesOut);
```
Reads the normalization of the database a model learned: per feature, the offset taken from a value and the scale it is divided by.  @param model       An opened model. @param offsetsOut  Receives the feature count of offsets, or NULL. @param scalesOut   Receives the feature count of scales, or NULL. @return `mani_success`; `mani_errorInvalid` for a NULL model. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniLearnedModelProjector(const maniLearnedModel* model, uint32_t projector, uint32_t* requiredOut, uint32_t* excludedOut);
```
Reads the tag filter a Projector learned: an entry's tags held all of required and none of excluded for its frames to be targets.  @param model        An opened model. @param projector    The Projector's index. @param requiredOut  Receives the required tags, or NULL. @param excludedOut  Receives the excluded tags, or NULL. @return `mani_success`; `mani_errorInvalid` for a NULL model or an index past the Projectors. @par Thread safety Safe from any thread.

```c
const char* maniLearnedModelFloatTrackName(const maniLearnedModel* model, uint32_t track, uint32_t* lengthOut);
```
Returns the name of a float track a model gives.  @param model      An opened model. @param track      The track's index. @param lengthOut  Receives the name's length in bytes, or NULL. @return The name, UTF-8, not terminated; NULL for a NULL model or an index past the tracks. @par Thread safety Safe from any thread.

```c
const char* maniLearnedModelEventName(const maniLearnedModel* model, uint32_t event, uint32_t* lengthOut);
```
Returns the name of an event a model gives the state of.  @param model      An opened model. @param event      The event's index. @param lengthOut  Receives the name's length in bytes, or NULL. @return The name, UTF-8, not terminated; NULL for a NULL model or an index past the events. @par Thread safety Safe from any thread.

```c
size_t maniLearnedScratchSize(const maniLearnedModel* model);
```
Returns the bytes of scratch memory an evaluation of the model needs.  @param model  An opened model, or NULL. @return The size; 0 for NULL. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniLearnedProject(const maniLearnedModel* model, uint32_t projector, const float* query, float* stateOut, void* scratch, size_t scratchSize);
```
Projects a query onto the state of the nearest frame a Projector learned.  @param model        An opened model. @param projector    The Projector's index. @param query        The model's feature count of features, normalized as the database's. @param stateOut     Receives the state. @param scratch      Scratch memory, 4-byte aligned. @param scratchSize  Its size: at least maniLearnedScratchSize. @return `mani_success`; `mani_errorInvalid` for a NULL argument, an index past the Projectors, a query value not finite, misaligned or too little scratch. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniLearnedStep(const maniLearnedModel* model, float* state, void* scratch, size_t scratchSize);
```
Moves a state one step on: adds the Stepper's rate of change times one step's seconds (1 / the model's step rate).  @param model        An opened model. @param state        The state, moved in place. @param scratch      Scratch memory, 4-byte aligned. @param scratchSize  Its size: at least maniLearnedScratchSize. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a state value not finite, misaligned or too little scratch. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniLearnedDecompress(const maniLearnedModel* model, const float* state, maniLearnedOutput* output, void* scratch, size_t scratchSize);
```
Decompresses a state: every bone's translation, its rotation from two matrix columns (made orthonormal, the first kept), its scale (given, or the constant the model keeps), and the rest of the outputs.  @param model        An opened model. @param state        The state. @param output       Receives what the Decompressor gives. @param scratch      Scratch memory, 4-byte aligned. @param scratchSize  Its size: at least maniLearnedScratchSize. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a state value not finite, a pose of another bone count or holding a difference, misaligned or too little scratch. @par Thread safety Safe from any thread.

## `learnedbuild.h`

Building learned motion matching models (mani-0025) from networks' weights: what a trainer gives, or networks trained elsewhere.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildLearnedModel(maniBuilder* builder, const maniLearnedModelSource* source);
```
Builds a model: checks the networks' shapes against their roles and every value, copies the database's channels, normalization and axes, and writes the weights as asked (16-bit: each input's weights divided by the largest of their magnitudes over 32767, rounded to nearest, ties to even, that scale kept).  @param builder  The builder. @param source   The model. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse), or with the reason and item for a refused source: `maniBuildReasonDatabase` (a database of another skeleton), `maniBuildReasonTiming` (the step rate), `maniBuildReasonOption` (the latent count or weight format), `maniBuildReasonCount` (no Projector, or a list past its limit), `maniBuildReasonJoint` (a scaled bone past the skeleton or not increasing; item its place), `maniBuildReasonNotFinite` (a bone's scale; item the bone), `maniBuildReasonNetwork` (item the network: 0 the Decompressor, 1 the Stepper, 2 + p Projector p; other its layer, or its layer count for its normalization), `maniBuildReasonFloatTrack` or `maniBuildReasonEvent` combined with `maniBuildReasonName` or `maniBuildReasonDuplicateName`; `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

## `learnedtrain.h`

Training learned motion matching models (mani-0025) in the library, from a motion database, its clips and skeleton: Holden's networks, losses and optimizer, the same bits on every platform, with any thread count and across checkpoints.

```c
static inline maniLearnedTrainerDef maniDefaultLearnedTrainerDef(void);
```
Returns a trainer def with every setting at its default.  @return The def, with a valid cookie. @par Thread safety Safe from any thread.

```c
static inline maniLearnedTrainerOptions maniDefaultLearnedTrainerOptions(void);
```
Returns Holden's options.  @return The options. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniBuildResult maniCreateLearnedTrainer( const maniLearnedTrainerDef* def, const maniLearnedTrainerSource* source, maniLearnedTrainer** trainerOut);
```
Makes a trainer: checks the def and source, and samples the training data from the database's clips at its frames. The source's objects and arrays must outlive the trainer.  @param def        The def. @param source     What to train on, and how. @param trainerOut Receives the trainer, or NULL on failure. @return `mani_success`; `mani_errorVersion` for a def built against another major or minor version; `mani_errorInvalid` for a NULL argument, a def without its cookie, an allocator or task hooks with one function, a list missing, or with the reason and item for a refused source: `maniBuildReasonOption` (item the option's place in the options: a count, width or layer count past its limit, a batch not a multiple of 4 or past MANI_LEARNED_MAX_BATCH, a window below 2, past MANI_LEARNED_MAX_WINDOW or longer than every entry, no iterations, a rate or seconds not finite or negative; item 1 for networks past MANI_LEARNED_MAX_WEIGHTS), `maniBuildReasonDatabase` (a database of another skeleton), `maniBuildReasonClip` (item the clip whose identity is not the database's; the clip count and the database's as item and other when they differ), `maniBuildReasonTiming` (a database rate past MANI_LEARNED_MAX_RATE), `maniBuildReasonCount` (no Projector, or a list past its limit), `maniBuildReasonTags` (item the Projector), `maniBuildReasonFloatTrack` or `maniBuildReasonEvent` combined with `maniBuildReasonName` or `maniBuildReasonDuplicateName`; `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread.

```c
void maniDestroyLearnedTrainer(maniLearnedTrainer* trainer);
```
Destroys a trainer. Called back in from the trainer's allocator or task hooks it destroys nothing and counts one misuse (mani-0028).  @param trainer  The trainer, or NULL. @par Thread safety Safe from any thread; the trainer is used by one thread at a time.

```c
uint32_t maniLearnedTrainerMisuses(const maniLearnedTrainer* trainer);
```
The misuses counted on a trainer: calls back in from its allocator or task hooks, which are refused with mani_errorState (mani-0028).  @param trainer  A trainer, or NULL (0). @return The count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniTrainLearned(maniLearnedTrainer* trainer, uint64_t iterations, maniLearnedTrainerReport* reportOut);
```
Trains on: up to iterations iterations, stage after stage. A stage whose iterations are all done ends (its network kept, the rest of its memory released) and the next begins when it first runs; the first stage's end gives every frame's latent values.  @param trainer    The trainer. @param iterations Iterations to run at most; 0 only reports. @param reportOut  Receives the progress. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorCapacity` when memory runs out as a stage begins or ends (the trainer keeps its progress, and a later call goes on). @par Thread safety Safe from any thread; the trainer is used by one thread at a time. Its chunks run on the def's task hooks.

```c
size_t maniLearnedCheckpointSize(const maniLearnedTrainer* trainer);
```
Returns the size of a checkpoint of the trainer as it is now.  @param trainer  The trainer. @return The size in bytes; 0 for NULL. @par Thread safety Safe from any thread; the trainer is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniSaveLearnedCheckpoint(const maniLearnedTrainer* trainer, void* bytes, size_t capacity);
```
Writes a checkpoint of the trainer: its progress, generator, networks and AdamW moments, under the signature of its source and options.  @param trainer  The trainer. @param bytes    Receives the checkpoint. @param capacity The size of bytes. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorCapacity` when bytes holds less than maniLearnedCheckpointSize. @par Thread safety Safe from any thread; the trainer is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniFileResult maniRestoreLearnedCheckpoint(maniLearnedTrainer* trainer, const void* bytes, size_t size);
```
Restores a checkpoint into a trainer of the same source and options, replacing its progress; training on gives the same bits as training through.  @param trainer  The trainer. @param bytes    The checkpoint, 16-byte aligned. @param size     Its size. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong (the identity's, 40, for a checkpoint of another source or options); `mani_errorVersion` for a newer format; `mani_errorCapacity` when memory runs out (the trainer then starts over). @par Thread safety Safe from any thread; the trainer is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildLearnedModelFromTrainer( maniBuilder* builder, const maniLearnedTrainer* trainer, maniLearnedWeights weightFormat);
```
Builds the model a trainer has trained, every stage done, through maniBuildLearnedModel: its step rate the database's sample rate.  @param builder      The builder. @param trainer      The trainer. @param weightFormat How the file keeps the weights. @return As maniBuildLearnedModel; `mani_errorInvalid` for a NULL argument or a trainer with a stage to run. @par Thread safety Safe from any thread; the builder is used by one thread at a time. The trainer is not trained meanwhile.

## `match.h`

Motion matching (mani-0024): a motion database built offline from a skeleton's clips, opened in place. Each frame of the database is a clip at a time; its features describe the pose and the root's path around that time in the root's space, normalized per channel, so a search compares a query's features with every frame's.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenMotionDatabase(const void* bytes, size_t size, maniMotionDatabase* databaseOut);
```
Opens a motion database file in place: checks the container, every record, count and float, that each entry's frames follow from its span and the rate, and that every bound is its run's exact minimum or maximum.  @param bytes       The file, 16-byte aligned. @param size        Its size in bytes; bytes past the file are ignored. @param databaseOut Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint64_t maniMotionDatabaseIdentity(const maniMotionDatabase* database);
```
Returns a database's identity: the hash of its sections.  @param database  An opened database, or NULL. @return The identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
uint64_t maniMotionDatabaseMirrorMapIdentity(const maniMotionDatabase* database);
```
Returns the identity of the mirror map a database's mirrored entries play through (mani-0033).  @param database  An opened database, or NULL. @return The map's identity; 0 for a database without mirrored entries, or NULL. @par Thread safety Safe from any thread.

```c
uint64_t maniMotionDatabaseSkeletonIdentity(const maniMotionDatabase* database);
```
Returns the identity of the skeleton a database was built for.  @param database  An opened database, or NULL. @return The identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
uint64_t maniMotionDatabaseClipIdentity(const maniMotionDatabase* database, uint32_t clip);
```
Returns the identity of a database's clip.  @param database  An opened database. @param clip      The clip's index. @return The identity; 0 for NULL or an index past the clips. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniMotionDatabaseChannel(const maniMotionDatabase* database, uint32_t channel, maniMatchChannel* channelOut);
```
Reads a database's channel.  @param database    An opened database. @param channel     The channel's index. @param channelOut  Receives the channel. @return `mani_success`; `mani_errorInvalid` for a NULL argument or an index past the channels. @par Thread safety Safe from any thread.

```c
const char* maniMotionDatabaseChannelName(const maniMotionDatabase* database, uint32_t channel, uint32_t* lengthOut);
```
Returns a float track channel's name.  @param database   An opened database. @param channel    The channel's index. @param lengthOut  Receives the name's length in bytes, or NULL. @return The name, UTF-8, not terminated; NULL for a NULL database, an index past the channels or a channel of another kind. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniMotionDatabaseEntry(const maniMotionDatabase* database, uint32_t entry, maniMatchEntry* entryOut);
```
Reads a database's entry.  @param database  An opened database. @param entry     The entry's index. @param entryOut  Receives the entry. @return `mani_success`; `mani_errorInvalid` for a NULL argument or an index past the entries. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniMotionDatabaseFrame(const maniMotionDatabase* database, uint32_t frame, uint32_t* entryOut, float* timeOut);
```
Finds a frame's entry and its clip time.  @param database  An opened database. @param frame     The frame. @param entryOut  Receives the entry's index. @param timeOut   Receives the clip time, seconds. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a frame past the frames. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniMotionDatabaseFeatures( const maniMotionDatabase* database, uint32_t frame, float* featuresOut);
```
Reads a frame's features, normalized: (value - offset) / scale.  @param database     An opened database. @param frame        The frame. @param featuresOut  Receives the database's feature count of values. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a frame past the frames. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniMotionDatabaseNormalization( const maniMotionDatabase* database, float* offsetsOut, float* scalesOut);
```
Reads a database's normalization: per feature, the offset taken from a value and the scale it is divided by.  @param database  An opened database. @param offsetsOut Receives the feature count of offsets, or NULL. @param scalesOut  Receives the feature count of scales, or NULL. @return `mani_success`; `mani_errorInvalid` for a NULL database. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSearchMotionDatabase(const maniMotionDatabase* database, const maniMatchQuery* query, maniMatchResult* resultOut);
```
Finds the frame of least cost, exactly (Holden's search with bounds over runs of 16 and 64 frames, which prune without changing the result). A frame's cost is its entry's cost bias plus the transition cost (the continuing frame: the continuing bias), plus the squared differences of the normalized query and its features in feature order, in binary32. The continuing frame is a candidate when its entry's tags pass; another frame when its entry's tags pass, its entry is not MANI_MATCH_NO_JUMP, and no exclusion holds. Ties go to the continuing frame, then to the lowest frame.  @param database  An opened database. @param query     The search. @param resultOut Receives the result. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a feature or cost not finite, a negative time, a continuing frame or pick past the frames, or more than MANI_MATCH_MAX_HISTORY picks. @par Thread safety Safe from any thread.

## `matchbuild.h`

Building motion databases (mani-0024): channels sampled from a skeleton's clips at a rate, normalized per channel as Holden's reference does, with bounds over runs of frames for the search.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildMotionDatabase(maniBuilder* builder, const maniMotionDatabaseSource* source);
```
Builds a motion database: samples every entry's frames, each channel at each of its times (clamped to the entry's ends, or wrapped when it loops), in the root's space at the frame (the clip's root motion carrying the root between times; a bone's velocity the difference of its positions a frame either side over their time apart), normalizes each channel, and bounds every run of 16 and 64 frames. A mirrored entry's frames are sampled from its clip's poses mirrored, its root moves reflected and each float track channel from the track the map pairs its name with, as a mirrored clip's would be. Keeps the file in place of any built before.  @param builder  The builder. @param source   The database. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse), or with the reason and item for a refused source: `maniBuildReasonTiming` (the rate), `maniBuildReasonOption` (the axes), `maniBuildReasonCount` (no channels, clips or entries, too many, or past MANI_MATCH_MAX_FRAMES frames or MANI_MATCH_MAX_FEATURES features), `maniBuildReasonJoint` (a clip of another skeleton or additive; item the clip), `maniBuildReasonChannel` (item the channel), `maniBuildReasonEntry` (item the entry), `maniBuildReasonMissingTrack` (a float track an entry's clip lacks, for a mirrored entry the track the mirror map pairs the channel's with; item the channel, other the entry), `maniBuildReasonSpread` (a channel that never varies; item the channel), `maniBuildReasonMirrorDatabase` (a mirror map of another skeleton, item 0; a mirrored entry without one, item the entry); `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

## `mirror.h`

Mirror maps (mani-0033): how poses of a skeleton are mirrored across a plane through the model's origin. A map is built offline for its skeleton (mirrorbuild.h) and opened in place; each bone takes its partner's local (its own when unpaired) carried through the reference pose's model rotations, so rigs whose left and right bones do not have mirrored axes mirror to their reference.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenMirrorMap(const void* bytes, size_t size, maniMirrorMap* mapOut);
```
Opens a mirror map file in place: checks the container, the header (the skeleton's identity and bone count, the axis, the name count, the first unmatched bone), every bone's partner (within the count, pairing back), flags (a scale permutation or the approximate flag, the first unmatched one the header's) and corrections (finite, of length 1), and every name pair (by increasing hash, its partner's name inside the names, UTF-8, of its hash).  @param bytes   The file, 16-byte aligned. @param size    Its size in bytes; bytes past the file are ignored. @param mapOut  Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint64_t maniMirrorMapIdentity(const maniMirrorMap* map);
```
Returns a map's identity: the hash of its sections.  @param map  An opened map, or NULL. @return The identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
uint64_t maniMirrorMapSkeletonIdentity(const maniMirrorMap* map);
```
Returns the identity of the skeleton a map mirrors.  @param map  An opened map, or NULL. @return The skeleton's identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
maniMirrorAxis maniMirrorMapAxis(const maniMirrorMap* map);
```
Returns the axis a map's plane is normal to.  @param map  An opened map, or NULL. @return maniMirrorAxisX, Y or Z; maniMirrorAxisX for NULL. @par Thread safety Safe from any thread.

```c
uint32_t maniMirrorPartner(const maniMirrorMap* map, uint32_t bone);
```
Returns a bone's partner: the bone it takes its mirrored local from (itself when unpaired).  @param map   An opened map, or NULL. @param bone  A bone of the map's skeleton. @return The partner; MANI_NO_BONE for NULL or a bone past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniMirrorBoneFlags(const maniMirrorMap* map, uint32_t bone);
```
Returns how a bone's mirrored local is made, as flags: MANI_MIRROR_BONE_UNMATCHED when its parent's partner is not its partner's parent (its mirrored pose is its partner's local turn mirrored rather than its partner's model turn), and MANI_MIRROR_BONE_SCALE_APPROXIMATE when the turn between its reference frame and its partner's mirrored one takes no axis to an axis, so a scale that differs between axes is carried as the nearest axis scale (a uniform scale stays exact). Otherwise a scale is exact, each axis taking the partner's axis the turn maps to it.  @param map   An opened map, or NULL. @param bone  A bone of the map's skeleton. @return The flags; 0 for NULL or a bone past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniMirrorMapUnmatchedBone(const maniMirrorMap* map);
```
Returns the first bone whose parent's partner is not its partner's parent: where the hierarchy is not mirror-symmetric, so that its mirrored pose is its partner's local turn mirrored rather than its partner's model turn.  @param map  An opened map, or NULL. @return The bone; MANI_NO_BONE for a symmetric hierarchy or NULL. @par Thread safety Safe from any thread.

```c
const char* maniMirrorName(const maniMirrorMap* map, const char* name, size_t length, size_t* lengthOut);
```
Returns the partner of a float track's, event's or marker's name: the name the map pairs it with, or the name itself.  @param map        An opened map. @param name       The name's UTF-8 bytes; may be NULL when length is 0. @param length     The name's length in bytes. @param lengthOut  Receives the partner's length. May be NULL. @return The partner's bytes inside the map, not terminated, or name itself when unpaired (or for a NULL map). @par Thread safety Safe from any thread.

```c
uint64_t maniMirrorNameHash(const maniMirrorMap* map, uint64_t hash);
```
Returns the hash of a name's partner from the name's hash, for names kept hashed (events, markers): the hash of the name the map pairs it with, or the hash itself.  @param map   An opened map, or NULL (the hash itself). @param hash  The name's hash, as maniClipEvent's nameHash holds it. @return The partner's hash. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniMirrorPose(const maniMirrorMap* map, const maniSkeleton* skeleton, const maniPose* in, maniPose* out);
```
Mirrors a pose: each bone takes its partner's local (its own when unpaired), the translation's component along the normal negated and turned by the bone's correction from its partner's parent's reference frame mirrored to its parent's, the rotation mirrored between the corrections into its own reference axes, the scale as it is. On a mirror-symmetric hierarchy each bone's model position is its partner's reflected and its model rotation its partner's mirrored, carried to its own reference axes, so the rest's model rotations stay; mirroring twice gives the pose back. Writes every local of out and leaves its model stale.  @param map       An opened map. @param skeleton  The map's skeleton. @param in        A pose of it (not a difference). @param out       A pose of it; may be in. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skeleton whose identity or bone count is not the map's, a pose of another skeleton, or a difference. @par Thread safety Safe from any thread; out is used by one thread at a time.

```c
maniRootMove maniMirrorRootMotion(const maniMirrorMap* map, maniRootMove move);
```
Mirrors a root motion move: its translation's component along the normal negated, its rotation mirrored (the turn the reflection makes of it).  @param map   An opened map, or NULL (the move unchanged). @param move  The move. @return The mirrored move. @par Thread safety Safe from any thread.

## `mirrorbuild.h`

Building mirror maps (mani-0033): a skeleton's bones paired by explicit pairs and by find/replace rules on their names, a reference pose, a plane, and the names of float tracks, events and markers paired the same way, laid out as each bone's partner and corrections so that mirroring at runtime only composes rotations. Offline.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildMirrorMap(maniBuilder* builder, const maniMirrorMapSource* source);
```
Builds a mirror map file (kind 13) from a source and keeps it in the builder. A rule's partner must pair back by the rules (or be left to the rules by the pairs). The map records the first bone whose parent's partner is not its partner's parent, without refusing it.  @param builder  The builder. @param source   The map. @return `mani_success`; `mani_errorInvalid` with `maniBuildReasonNone` for a NULL builder, source or skeleton, a count without its list, or reserved not 0; `maniBuildReasonOption` for an unknown axis; `maniBuildReasonMirrorReference` (item the bone); `maniBuildReasonMirrorRule` (item the rule); `maniBuildReasonMirrorPair` (item the bone, other its partner); `maniBuildReasonMirrorName` (item the name, other the earlier); `maniBuildReasonNameHash` for two names of one hash (item the later name, other the earlier); `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniWriteMirrorMapJson(maniBuilder* builder, const maniMirrorMapSource* source);
```
Builds a mirror map as maniBuildMirrorMap does and, when it builds, keeps its JSON text form in the builder in place of the file: format "maul-anim mirror map", version 1; the axis ("x", "y", "z"); the reference pose as the bones whose local is not the rest bit for bit, by name, with translation, rotation and scale; bone pairs by name; rules (find, place of "prefix", "suffix" or "anywhere", replace); name pairs and names. Numbers as the shortest text that reads back to the same float.  @param builder  The builder. @param source   The map. @return As maniBuildMirrorMap. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildMirrorMapJson(maniBuilder* builder, const char* text, size_t length, const maniSkeleton* skeleton);
```
Builds a mirror map file from its JSON text form, as maniWriteMirrorMapJson writes it: members an object does not take are refused; the reference, pairs, rules, name pairs and names may be left out (empty), as may a reference entry's translation, rotation or scale (its bone's rest). A bone the reference lists twice is refused. The source read is then built as maniBuildMirrorMap builds it.  @param builder   The builder. @param text      The UTF-8 text (a leading byte order mark skipped); may be NULL when length is 0. @param length    The text's length in bytes. @param skeleton  The skeleton. @return As maniBuildMirrorMap, items by place in the text's lists; or `mani_errorInvalid` with maniBuildReasonNone for a NULL builder or skeleton or a NULL text with a length, maniBuildReasonJson or maniBuildReasonForm (other a maniGraphForm), item the byte offset. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildMirroredClip(maniBuilder* builder, const maniMirrorClipSource* source);
```
Builds a full-precision clip of a clip mirrored (bake it once, play it as any clip), and keeps it in the builder. The clip keeps the clip's sample rate, duration, loop policy and additive basis. At each sample the pose is mirrored by maniMirrorPose; a channel that is the rest at every sample is left out. An additive clip's differences are each added to the rest, mirrored, and taken as differences from the rest mirrored. Float tracks keep their values at the samples and their interpolation; discrete tracks, events and markers are copied; every name is renamed by maniMirrorName. Root motion moves to the root motion bone's partner: each stored move (in the bone's parent's frame) reflected as maniMirrorRootMotion reflects it and turned into the partner's parent's frame mirrored, its source's axes following that turn (all axes, and the whole rotation, where it takes an axis off the axes).  @param builder  The builder. @param source   The clip to mirror. @return `mani_success`; `mani_errorInvalid` with `maniBuildReasonNone` for a NULL builder, source or field, or reserved not 0; `maniBuildReasonMirrorClip` (other 0 the skeleton, 1 the clip: a skeleton not the map's, a clip not of the map's skeleton); `maniBuildReasonBasis` for an additive clip whose difference from its basis mirrored is not finite; `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

## `morph.h`

Morph targets (mani-0021): a mesh primitive's blend shapes over the caller's bytes. Named channels, each one or more targets at increasing full weights (glTF's targets have one at 1; FBX's in-betweens several), each target deltas for some of the file's attributes (positions, normals, tangents, texture coordinate and color sets), dense or sparse.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenMorphTargets(const void* bytes, size_t size, maniMorphTargets* targetsOut);
```
Opens a morph targets file in place: checks the container, the sizes, the attribute table, the channels covering the targets in order, every name inside the name bytes, every full weight finite, not 0 and increasing within its channel, every sparse target's indices increasing and below the vertex count, every delta finite, and the identity.  @param bytes       The file, 16-byte aligned. @param size        Its size in bytes; bytes past the file are ignored. @param targetsOut  Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint32_t maniMorphVertexCount(const maniMorphTargets* targets);
```
Returns the number of vertices the targets move.  @param targets  Opened morph targets. @return The vertex count. @par Thread safety Safe from any thread.

```c
uint32_t maniMorphChannelCount(const maniMorphTargets* targets);
```
Returns the number of channels, each weighted by its own value.  @param targets  Opened morph targets. @return The channel count. @par Thread safety Safe from any thread.

```c
uint32_t maniMorphTargetCount(const maniMorphTargets* targets);
```
Returns the number of targets over all channels, in channel order.  @param targets  Opened morph targets. @return The target count. @par Thread safety Safe from any thread.

```c
uint32_t maniMorphAttributeCount(const maniMorphTargets* targets);
```
Returns the number of attributes the targets move.  @param targets  Opened morph targets. @return The attribute count, at most MANI_MORPH_MAX_ATTRIBUTES. @par Thread safety Safe from any thread.

```c
maniMorphSemantic maniMorphAttribute(const maniMorphTargets* targets, uint32_t attribute, uint32_t* setOut);
```
Returns one of the attributes the targets move.  @param targets    Opened morph targets. @param attribute  The attribute, in the file's order. @param setOut     Receives its set (0 for positions, normals and tangents); may be NULL. @return Its semantic; UINT32_MAX (and set 0) past the count. @par Thread safety Safe from any thread.

```c
const char* maniMorphChannelName(const maniMorphTargets* targets, uint32_t channel, uint32_t* lengthOut);
```
Returns a channel's name, UTF-8 and not terminated.  @param targets    Opened morph targets. @param channel    The channel. @param lengthOut  Receives the name's length in bytes, 0 for a channel past the count. May be NULL. @return The name's bytes, inside the file, or NULL for a channel past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniMorphFindChannel(const maniMorphTargets* targets, const char* name, size_t length);
```
Finds a channel by its name, in time linear in the channels.  @param targets  Opened morph targets. @param name     The name's UTF-8 bytes; may be NULL when length is 0. @param length   The name's length in bytes. @return The channel, or MANI_NO_CHANNEL when none has that name. @par Thread safety Safe from any thread.

```c
const char* maniMorphTargetName(const maniMorphTargets* targets, uint32_t target, uint32_t* lengthOut);
```
Returns a target's name, UTF-8 and not terminated: the one it was built with, else its channel's, followed by '/' and its index within the channel past the first. Clip weight tracks (`<node or mesh>/<name>`) bind to targets by it.  @param targets    Opened morph targets. @param target     The target. @param lengthOut  Receives the name's length in bytes, 0 for a target past the count. May be NULL. @return The name's bytes, inside the file, or NULL for a target past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniMorphFindTarget(const maniMorphTargets* targets, const char* name, size_t length);
```
Finds a target by its name, in time linear in the targets: the first of that name.  @param targets  Opened morph targets. @param name     The name's UTF-8 bytes; may be NULL when length is 0. @param length   The name's length in bytes. @return The target, or MANI_NO_CHANNEL when none has that name. @par Thread safety Safe from any thread.

```c
uint32_t maniMorphChannelTargets(const maniMorphTargets* targets, uint32_t channel, uint32_t* firstOut);
```
Returns a channel's targets: the first's index and their count.  @param targets   Opened morph targets. @param channel   The channel. @param firstOut  Receives the first target's index; may be NULL. @return The number of targets, 0 for a channel past the count. @par Thread safety Safe from any thread.

```c
float maniMorphTargetFullWeight(const maniMorphTargets* targets, uint32_t target);
```
Returns the channel weight at which a target applies in full.  @param targets  Opened morph targets. @param target   The target. @return The full weight; 0 for a target past the count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniMorphTargetWeights(const maniMorphTargets* targets, const float* channelWeights, float* targetWeights);
```
Turns channel weights into target weights, for the job or a shader: within each channel, the weight walks linearly from an implicit zero shape at 0 through the targets at their full weights, past the last (or first) extrapolated. A channel of one target at 1 passes its weight through.  @param targets         Opened morph targets. @param channelWeights  A finite weight a channel, any sign. @param targetWeights   Receives a weight a target. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a weight not finite (nothing written). @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniApplyMorphTargets(const maniMorphJob* job);
```
Applies morph targets: each stream's vertices plus its attribute's weighted deltas, target by target in order (so ranges and sparsity give the same bits), then every normal and tangent of the range renormalized (zero kept zero). Runs before skinning, into the skinning job's input.  @param job  The job. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a weight not finite, a range past the vertices, or a stream without pointers, with a stride too small, an unknown semantic, a set beyond 0 for positions, normals or tangents, or components other than its semantic's (nothing written). @par Thread safety Safe from any thread; the output range is used by one thread at a time. Jobs over disjoint ranges run at once.

## `pose.h`

Poses (mani-0004): a skeleton's local transforms and the model matrices composed from them, in memory the caller owns. A pose holds no pointers: copying its bytes copies the pose, and it may move between threads. A pose is used by one thread at a time.

```c
size_t maniPoseSize(const maniSkeleton* skeleton);
```
Returns the bytes a pose of the skeleton needs.  @param skeleton  An opened skeleton. @return The size in bytes. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniInitPose(const maniSkeleton* skeleton, void* memory, size_t size, maniPose** poseOut);
```
Makes a pose of the skeleton in caller memory, every bone at its rest, the model matrices composed.  @param skeleton  An opened skeleton. @param memory    At least maniPoseSize bytes, MANI_POSE_ALIGNMENT aligned; the pose lives in it. @param size      The memory's size in bytes. @param poseOut   Receives the pose. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned memory; `mani_errorCapacity` when size is too small. @par Thread safety Safe from any thread.

```c
uint32_t maniPoseBoneCount(const maniPose* pose);
```
Returns the number of bones of the pose's skeleton.  @param pose  The pose. @return The bone count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
maniTransform maniPoseLocal(const maniPose* pose, uint32_t bone);
```
Returns a bone's local transform.  @param pose  The pose. @param bone  The bone. @return The transform, or the identity for a bone past the count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
void maniPoseSetLocal(maniPose* pose, uint32_t bone, maniTransform transform);
```
Sets a bone's local transform and marks the model matrices stale. A bone past the count is ignored.  @param pose       The pose. @param bone       The bone. @param transform  The transform; its rotation is used as given and should be unit. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniPoseReadLocals(const maniPose* pose, uint32_t first, uint32_t count, maniTransform* transforms);
```
Reads the local transforms of bones first to first + count - 1.  @param pose        The pose. @param first       The first bone. @param count       How many bones; the range must lie within the bone count. @param transforms  Receives count transforms. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a range past the bone count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniPoseWriteLocals(maniPose* pose, uint32_t first, uint32_t count, const maniTransform* transforms);
```
Writes the local transforms of bones first to first + count - 1 and marks the model matrices stale.  @param pose        The pose. @param first       The first bone. @param count       How many bones; the range must lie within the bone count. @param transforms  The count transforms. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a range past the bone count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniPoseSetRest(maniPose* pose, const maniSkeleton* skeleton);
```
Sets every bone to the skeleton's rest and composes the model.  @param pose      The pose. @param skeleton  The pose's skeleton. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a skeleton of another identity. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniComputeModel(maniPose* pose, const maniSkeleton* skeleton);
```
Composes every bone's model matrix: its parent's model matrix times its local transform's matrix (scale, then rotation, then translation), a root's being its local's.  @param pose      The pose. @param skeleton  The pose's skeleton. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skeleton of another identity or a difference (which has no model). @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
bool maniPoseModelCurrent(const maniPose* pose);
```
Returns whether the model matrices match the local transforms: true after maniComputeModel until a local is set.  @param pose  The pose. @return True when current. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
bool maniPoseIsDifference(const maniPose* pose);
```
Returns whether the pose holds a difference (mani-0020): what maniPoseDifference writes and additive layers apply, never mixed with poses of the skeleton. Making a pose, setting its rest and sampling a whole clip into it make it a pose of the skeleton.  @param pose  The pose. @return True for a difference. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
maniAffine maniPoseModel(const maniPose* pose, uint32_t bone);
```
Returns a bone's model matrix as last composed.  @param pose  The pose. @param bone  The bone. @return The matrix, or the identity for a bone past the count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniPoseReadModels(const maniPose* pose, uint32_t first, uint32_t count, maniAffine* matrices);
```
Reads the model matrices of bones first to first + count - 1, as last composed.  @param pose      The pose. @param first     The first bone. @param count     How many bones; the range must lie within the bone count. @param matrices  Receives count matrices. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a range past the bone count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniDecomposeAffine(const maniAffine* matrix, maniTransform* transformOut);
```
Splits an affine matrix into translation, rotation and scale: the fourth column, the rotation of the polar decomposition (the rotation nearest the matrix, exact up to rounding when the matrix has no shear) as a quaternion with w >= 0, and the scale along the rotated axes. A mirror becomes a negative x scale. A matrix with a zero or flat axis still splits, its rotation built from the axes that are left.  @param matrix        The matrix. @param transformOut  Receives the transform. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a component that is not finite. @par Thread safety Safe from any thread.

```c
maniTransform maniPoseModelTransform(const maniPose* pose, uint32_t bone);
```
Returns a bone's model matrix, as last composed, split as maniDecomposeAffine splits it.  @param pose  The pose. @param bone  The bone. @return The transform, or the identity for a bone past the count or a matrix that is not finite. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

## `ragdoll.h`

Ragdoll profiles (mani-0027): a skeleton's ragdoll as data a host maps onto its own physics engine: bodies on a subset of the bones, each with its frame, a primitive shape and a mass, and a swing-twist joint to its parent body with limits and an optional drive. A profile is built offline for one skeleton and opened in place; the library never simulates it. Lengths are the skeleton's units, angles radians, times seconds; masses and drives are the host engine's units, passed through unconverted.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenRagdoll(const void* bytes, size_t size, maniRagdoll* ragdollOut);
```
Opens a ragdoll profile file in place: checks the container and every record (bones within the skeleton's count and increasing, parents before children, one root, shapes, sizes and masses positive, frames finite with unit rotations, limits and drives in range, names, the root's facing and up axes unit and perpendicular).  @param bytes      The file, 16-byte aligned. @param size       Its size in bytes; bytes past the file are ignored. @param ragdollOut Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint64_t maniRagdollIdentity(const maniRagdoll* ragdoll);
```
Returns a profile's identity: the hash of its sections.  @param ragdoll  An opened profile, or NULL. @return The identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
uint64_t maniRagdollSkeletonIdentity(const maniRagdoll* ragdoll);
```
Returns the identity of the skeleton a profile was built for.  @param ragdoll  An opened profile, or NULL. @return The identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
uint32_t maniRagdollBodyCount(const maniRagdoll* ragdoll);
```
Returns the number of bodies in a profile.  @param ragdoll  An opened profile, or NULL. @return The count, at least 1; 0 for NULL. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniRagdollReadBody(const maniRagdoll* ragdoll, uint32_t body, maniRagdollBody* bodyOut);
```
Reads a body of a profile, in the order the profile keeps them (by bone, parents first).  @param ragdoll  An opened profile. @param body     The body's index. @param bodyOut  Receives the body. @return `mani_success`; `mani_errorInvalid` for a NULL argument or an index past the bodies. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniRagdollBodiesFromPose( const maniRagdoll* ragdoll, const maniPose* pose, const maniTransform* placement, const maniPose* previous, const maniTransform* previousPlacement, float seconds, maniRagdollBodyState* bodiesOut, uint32_t count);
```
Places a profile's bodies where a pose puts them: each body's frame on its bone's model frame (its scale left out) under the character's placement (its scale left out too: a profile is in the skeleton's units). With the pose before, its placement and the seconds between, each body's velocities are the differences over that time, the angular one the shortest turn's (so at most pi over seconds: a bone turning more than half a turn in one step reads as the shorter turn the other way); without, they are 0. Kinematic bodies and world-space drives take these. On a cut pass what the step means: no previous pose on the first frame or a clip switched without a blend (the bodies at rest); the current placement as previousPlacement on a teleport (the animation's motion alone); pose as previous for the placement's motion alone.  @param ragdoll    An opened profile. @param pose       A pose of the profile's skeleton, its model matrices current. @param placement  The character in the world; NULL for the identity. @param previous   The pose a moment before, its model current; NULL for no velocities. @param previousPlacement  Its placement; NULL for the identity. @param seconds    The time between the two, above 0 and finite when previous is given. @param bodiesOut  Receives a state per body, in the profile's order. @param count      The number of states bodiesOut holds. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a pose of another skeleton, a difference or one whose model is not current, a placement not finite, seconds out of range; `mani_errorCapacity` when count is under the body count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniRagdollJointTargets(const maniRagdoll* ragdoll, const maniRagdollBodyState* bodies, maniRagdollJointTarget* jointsOut, uint32_t count);
```
Works out each joint's motor target from bodies' states: the child's joint frame in the parent's, and the child's angular velocity less the parent's about the parent joint frame's axes. The root's entry is the identity and 0.  @param ragdoll    An opened profile. @param bodies     A state per body, in the profile's order. @param jointsOut  Receives a target per body. @param count      The number of targets jointsOut holds (and of states bodies holds). @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorCapacity` when count is under the body count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniRagdollPoseFromBodies( const maniRagdoll* ragdoll, const maniSkeleton* skeleton, const maniRagdollBodyState* bodies, uint32_t count, const maniTransform* placement, const maniPose* reference, maniPose* poseOut);
```
Makes a pose from bodies' places: a bone with a body takes its model frame from the body's, the body's frame taken off, under the placement, and its local scale from the reference; a bone without one keeps the reference's local transform below its parent. The model matrices are composed. Velocities are not read.  @param ragdoll    An opened profile. @param skeleton   Its skeleton. @param bodies     A state per body, in the profile's order: positions finite, rotations of length 1 within a thousandth. @param count      The number of states bodies holds. @param placement  The character in the world; NULL for the identity. @param reference  A pose of the skeleton for the bones without bodies and the scales; may be poseOut. @param poseOut    Receives the pose. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skeleton or pose of another skeleton, a difference, a placement or a body's place not finite or a rotation not of length 1; `mani_errorCapacity` when count is under the body count. @par Thread safety Safe from any thread; the poses are used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniRagdollGetUpPlacement(const maniRagdoll* ragdoll, const maniRagdollBodyState* bodies, uint32_t count, const maniRagdollGround* ground, maniRagdollGetUp* out);
```
Works out where a ragdoll gets up. The body lies face up when the root body's facing axis (out of the chest) points along the normal or across it. The heading is the root body's up axis (toward the head) on the ground's plane, reversed face up, plus its facing axis on the plane weighed by how upright the up axis stands (its component along the normal), so the character stands looking where its head lay face down and toward its feet face up (on its back, front or side), and where its chest faces upright; when that sum is shorter than a thousandth (the two cancel, the body inverted), the head's way alone. The placement stands on the plane under the root body, its model up along the normal and its model forward along the heading, scale 1. A pose from the bodies under it (maniRagdollPoseFromBodies) is the ragdoll in the character's new frame, to blend a get-up clip from.  @param ragdoll  An opened profile. @param bodies   A state per body, in the profile's order: positions finite, rotations of length 1 within a thousandth. @param count    The number of states bodies holds. @param ground   The ground and the character's axes. @param out      Receives the placement and whether it lies face up. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a body's place not finite or its rotation not of length 1, the ground not finite, its normal or the axes not of length 1 within a thousandth or the axes not perpendicular within a thousandth; `mani_errorCapacity` when count is under the body count. @par Thread safety Safe from any thread.

## `retarget.h`

Retargeting maps (mani-0032): how poses of one skeleton (the source) become poses of another (the target). A map is built offline for its two skeletons (retargetbuild.h) and opened in place; each target bone takes its rotation from a source bone, from two neighbors along a chain, or keeps its reference, as a model-space turn from each side's reference pose.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenRetargetMap(const void* bytes, size_t size, maniRetargetMap* mapOut);
```
Opens a retargeting map file in place: checks the container, the header (the skeletons' identities and bone counts, the pelvis's bones, axis and numbers), both reference poses (finite, rotations of length 1) and every target bone's plan (its kind, its source bones within the source's count, its place and weight in [0, 1], its translation and scale settings, unused words 0).  @param bytes  The file, 16-byte aligned. @param size   Its size in bytes; bytes past the file are ignored. @param mapOut Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint64_t maniRetargetMapIdentity(const maniRetargetMap* map);
```
Returns a map's identity: the hash of its sections.  @param map  An opened map, or NULL. @return The identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
uint64_t maniRetargetMapSourceIdentity(const maniRetargetMap* map);
```
Returns the identity of the skeleton a map takes poses from.  @param map  An opened map, or NULL. @return The source skeleton's identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
uint64_t maniRetargetMapTargetIdentity(const maniRetargetMap* map);
```
Returns the identity of the skeleton a map makes poses for.  @param map  An opened map, or NULL. @return The target skeleton's identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
size_t maniRetargetScratchSize(const maniRetargetMap* map);
```
Returns the scratch maniRetargetPose takes with a map: a rotation for every bone of both skeletons.  @param map  An opened map, or NULL. @return The bytes; 0 for NULL. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniRetargetPose( const maniRetargetMap* map, const maniSkeleton* source, const maniPose* sourcePose, const maniSkeleton* target, maniPose* targetPose, void* scratch, size_t scratchSize);
```
Makes a pose of the target from a pose of the source through a map. Each bone's model rotation is the product of its root path's local rotations. A mapped target bone's model rotation is the source bone's times the inverse of the source's reference model rotation (two source bones' such turns slerped at the bone's place), slerped from no turn by its weight, times the target's reference model rotation; its local rotation is taken against its parent's as just made. An unmapped bone keeps its reference local. Translations are the target reference's, or the source's change from its reference (scaled by the pelvis height ratio) or the source's own, turned from the source parent's reference model frame to the target parent's; a bone whose scale carries takes its reference's times the source's over the source reference's, per axis. The pelvis's model position is the target reference's plus the source pelvis's change from its reference, its horizontal and vertical parts scaled by the map's scales, blended per model axis toward the unscaled change and then toward the source pelvis's position, weighted from the reference. Writes every local of the target pose and leaves its model stale.  @param map          An opened map. @param source       The map's source skeleton. @param sourcePose   A pose of the source (not a difference). @param target       The map's target skeleton. @param targetPose   A pose of the target; its locals are written. @param scratch      Scratch memory, 16-byte aligned. @param scratchSize  Its size: at least maniRetargetScratchSize. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skeleton whose identity is not the map's, a pose of another skeleton, a source pose of differences, misaligned or too little scratch. @par Thread safety Safe from any thread; the target pose and the scratch are used by one thread at a time.

```c
maniVec3 maniRetargetRootMotion(const maniRetargetMap* map, maniVec3 delta);
```
Scales a root motion delta (model space) of the source for the target: its part along the map's up axis by the vertical scale, the rest by the horizontal scale (both 1 without a pelvis).  @param map    An opened map, or NULL (the delta unchanged). @param delta  The source's root motion translation. @return The target's. @par Thread safety Safe from any thread.

## `retargetbuild.h`

Building retargeting maps (mani-0032): bone pairs, chains of any lengths on each side, the pelvis and a reference pose per skeleton, checked against both skeletons and laid out as each target bone's plan, so the transfer at runtime only composes rotations. Offline.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildRetargetMap(maniBuilder* builder, const maniRetargetMapSource* source);
```
Builds a retargeting map file (kind 4) from a source and keeps it in the builder. Each target bone is mapped once at most: by a pair, a chain, the pelvis or a name, in that order.  @param builder  The builder. @param source   The map. @return `mani_success`; `mani_errorInvalid` with `maniBuildReasonNone` for a NULL builder, source or skeleton, a count without its list; `maniBuildReasonRetargetReference` (item the bone, other 0 for the source's pose and 1 for the target's: a local not finite or its rotation not of length 1 within a thousandth); `maniBuildReasonRetargetPair` (item the pair, other the target bone: a bone out of range, a target bone mapped before, a weight outside [0, 1], an unknown translation); `maniBuildReasonRetargetChain` (item the chain, other the target bone mapped before or 0: bones out of range, an end not below its start or past MANI_IK_CHAIN_MAX_BONES from it, an unknown mode or translation, a weight outside [0, 1]); `maniBuildReasonRetargetPelvis` (a bone out of range or the target mapped before, an up axis not of length 1, a factor negative or not finite, a blend or weight outside [0, 1], a reference pelvis at or below height 0); `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniWriteRetargetMapJson(maniBuilder* builder, const maniRetargetMapSource* source);
```
Builds a retargeting map as maniBuildRetargetMap does and, when it builds, keeps its JSON text form in the builder in place of the file: format "maul-anim retarget map", version 1; each reference pose as the bones whose local is not the rest bit for bit, by name, with translation, rotation and scale; pairs, chains and the pelvis (null when none) with bones by name and modes as words ("own", "scaled", "absolute"; "none", "interpolated", "oneToOne", "oneToOneReversed"); matchNames. Numbers as the shortest text that reads back to the same float.  @param builder  The builder. @param source   The map. @return As maniBuildRetargetMap. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildRetargetMapJson(maniBuilder* builder, const char* text, size_t length, const maniSkeleton* source, const maniSkeleton* target);
```
Builds a retargeting map file from its JSON text form, as maniWriteRetargetMapJson writes it: members an object does not take are refused; the references, pairs and chains may be left out (empty), as may the pelvis (none), matchNames (false), a reference entry's translation, rotation or scale (its bone's rest), a pair's or chain's translation ("own") and scale (false), and the pelvis's absolute and toSource (zero). A bone a reference lists twice is refused. The source read is then built as maniBuildRetargetMap builds it.  @param builder  The builder. @param text     The UTF-8 text (a leading byte order mark skipped); may be NULL when length is 0. @param length   The text's length in bytes. @param source   The source skeleton. @param target   The target skeleton. @return As maniBuildRetargetMap, items by place in the text's lists; or `mani_errorInvalid` with maniBuildReasonNone for a NULL builder or skeleton or a NULL text with a length, maniBuildReasonJson or maniBuildReasonForm (other a maniGraphForm), item the byte offset. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildRetargetedClip(maniBuilder* builder, const maniRetargetClipSource* source);
```
Builds a full-precision clip of the target from a clip of the source through a map (bake it once, play it as any clip), and keeps it in the builder. The clip keeps the source clip's sample rate, duration, loop policy and additive basis. At each sample the source's pose is made a pose of the target by maniRetargetPose; a channel that is the target's rest at every sample is left out. An additive clip's differences are each added to the source's rest, carried, and taken as differences from the source's rest carried. Float tracks keep their values at the samples and their interpolation (linear or step); discrete tracks, events and markers are copied. Root motion carries as moves: each translation's horizontal and vertical parts scaled as maniRetargetRootMotion scales them, both turned from the root motion bone's parent's reference frame to the target bone's parent's; its source's axes follow that turn (all axes, and the whole rotation, where it takes an axis off the axes).  @param builder  The builder. @param source   The clip to retarget. @return `mani_success`; `mani_errorInvalid` with `maniBuildReasonNone` for a NULL builder, source or field, or reserved not 0; `maniBuildReasonRetargetClip` (other 0 the source skeleton, 1 the target, 2 the clip: a skeleton not the map's, a clip not of the source); `maniBuildReasonRootMotion` (item the target bone: past the target's bones, or MANI_NO_BONE where no bone follows the root motion bone alone); `maniBuildReasonBasis` for an additive clip whose difference from its basis carried is not finite (a scale component of 0 in the basis); `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

## `rootmotion.h`

Root motion (mani-0022): the move a clip's declared bone makes, taken out of the bone at build and kept beside the clip, so the clip plays in place and the host moves the character by the moves it reads.

```c
maniRootMove maniRootMoveIdentity(void);
```
Returns the move of no motion.  @return Translation 0, rotation identity. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniClipRootMotion(const maniClip* clip, float time, float step, maniRootMove* moveOut);
```
Gives the move a clip's root motion makes from a playhead time over a signed step, in the character's frame where the move begins: for a clamped clip between the two times clamped to the clip; for a repeating clip across every wrap between the unwrapped times, the whole period's move composed by squaring (at most 2^32 periods), forward or backward. A clip without root motion moves nothing.  @param clip     An opened clip, of either form. @param time     The playhead's time, seconds (a repeating clip's taken within the period). @param step     The step, seconds, either sign. @param moveOut  Receives the move. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a time or step not finite (nothing written). @par Thread safety Safe from any thread.

```c
maniRootMove maniComposeRootMotion(maniRootMove first, maniRootMove then);
```
Composes two moves: first, then then from where first ends.  @param first  The first move. @param then   The move after it, in the frame first ends in. @return The whole move, in first's frame. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniBlendRootMotion(const maniRootMove* moves, const float* weights, uint32_t count, maniRootMove* moveOut);
```
Blends moves as poses are blended: the weights normalized, the translations' weighted sum, the rotations' normalized weighted sum with each turned into the hemisphere of the first weighted one. Weights of 0 count for nothing; all 0 give the move of no motion.  @param moves    count moves. @param weights  count weights, finite and not negative. @param count    The number of moves. @param moveOut  Receives the blend. @return `mani_success`; `mani_errorInvalid` for a NULL argument with a count, a weight negative or not finite, or a move not finite (nothing written). @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniRestoreRootMotion(const maniClip* clip, const maniSkeleton* skeleton, float time, maniPose* pose);
```
Puts a clip's root motion back onto its bone, in a pose sampled from the clip at the same time, for a host that keeps the motion in the pose: the bone's local becomes the motion at the time composed with it (translation m_t + m_r t, rotation m_r r, scale kept), and the model is marked stale. The time is clamped to [0, duration] (NaN is 0), as sampling clamps it. A clip without root motion leaves the pose unchanged.  @param clip      An opened clip. @param skeleton  The clip's skeleton. @param time      The time in seconds. @param pose      A pose of the skeleton, not a difference pose. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skeleton that is not the clip's or the pose's, or a difference pose (nothing written). @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
bool maniClipRootMotionSource(const maniClip* clip, maniRootMotionSource* sourceOut);
```
Reads the source a clip's root motion was taken from.  @param clip       An opened clip. @param sourceOut  Receives the source; unchanged on false. @return True when the clip has root motion. @par Thread safety Safe from any thread.

## `skeleton.h`

Skeletons (mani-0003): bones in depth-first order, each with a parent, a rest transform and a unique name, read in place from a skeleton file's bytes. Opening a file checks all of it once; the view then reads it without checks and allocates nothing.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenSkeleton(const void* bytes, size_t size, maniSkeleton* skeletonOut);
```
Opens a skeleton file in place: checks the container and every section (counts, depth-first order, rests finite with unit rotations, names valid UTF-8, unique and matching their hashes, the identity) and fills the view.  @param bytes       The file, 16-byte aligned. @param size        Its size in bytes; bytes past the file are ignored. @param skeletonOut Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint32_t maniSkeletonBoneCount(const maniSkeleton* skeleton);
```
Returns the number of bones, 1 to MANI_MAX_BONES.  @param skeleton  An opened skeleton. @return The bone count. @par Thread safety Safe from any thread.

```c
uint64_t maniSkeletonIdentity(const maniSkeleton* skeleton);
```
Returns the skeleton's identity: a hash of its bones' order, parents, rests and names, which files built against it record.  @param skeleton  An opened skeleton. @return The identity. @par Thread safety Safe from any thread.

```c
uint32_t maniSkeletonParent(const maniSkeleton* skeleton, uint32_t bone);
```
Returns a bone's parent, always a lower index.  @param skeleton  An opened skeleton. @param bone      The bone. @return The parent, or MANI_NO_BONE for a root or a bone past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniSkeletonSubtreeEnd(const maniSkeleton* skeleton, uint32_t bone);
```
Returns the end of a bone's subtree: the bone and its descendants are the indices from bone up to, not including, the end.  @param skeleton  An opened skeleton. @param bone      The bone. @return The end, or bone itself for a bone past the count (an empty range). @par Thread safety Safe from any thread.

```c
maniTransform maniSkeletonRest(const maniSkeleton* skeleton, uint32_t bone);
```
Returns a bone's rest transform, relative to its parent.  @param skeleton  An opened skeleton. @param bone      The bone. @return The rest, or the identity transform for a bone past the count. @par Thread safety Safe from any thread.

```c
const char* maniSkeletonBoneName(const maniSkeleton* skeleton, uint32_t bone, uint32_t* lengthOut);
```
Returns a bone's name, UTF-8 and not terminated.  @param skeleton  An opened skeleton. @param bone      The bone. @param lengthOut Receives the name's length in bytes, 0 for a bone past the count. May be NULL. @return The name's bytes, inside the file, or NULL for a bone past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniSkeletonFindBone(const maniSkeleton* skeleton, const char* name, size_t length);
```
Finds a bone by its name.  @param skeleton  An opened skeleton. @param name      The name's UTF-8 bytes; may be NULL when length is 0. @param length    The name's length in bytes. @return The bone, or MANI_NO_BONE when no bone has that name. @par Thread safety Safe from any thread.

## `skin.h`

Skins (mani-0005): the bones a mesh is bound to, in the mesh's order, each with its inverse bind matrix, read in place from a skin file and bound to one skeleton by its identity; and the palettes a renderer skins with.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenSkin(const void* bytes, size_t size, maniSkin* skinOut);
```
Opens a skin file in place: checks the container, the sizes, every joint against the skeleton's bone count the file records, and every inverse bind finite.  @param bytes    The file, 16-byte aligned. @param size     Its size in bytes; bytes past the file are ignored. @param skinOut  Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint32_t maniSkinJointCount(const maniSkin* skin);
```
Returns the number of joints, at least 1.  @param skin  An opened skin. @return The joint count. @par Thread safety Safe from any thread.

```c
uint64_t maniSkinSkeletonIdentity(const maniSkin* skin);
```
Returns the identity of the skeleton the skin is bound to.  @param skin  An opened skin. @return The skeleton's identity. @par Thread safety Safe from any thread.

```c
uint64_t maniSkinIdentity(const maniSkin* skin);
```
Returns the skin's own identity, which influences record to bind to it (mani-0021).  @param skin  An opened skin. @return The identity. @par Thread safety Safe from any thread.

```c
uint32_t maniSkinJoint(const maniSkin* skin, uint32_t joint);
```
Returns the skeleton bone of a joint.  @param skin   An opened skin. @param joint  The joint. @return The bone, or MANI_NO_BONE for a joint past the count. @par Thread safety Safe from any thread.

```c
maniAffine maniSkinInverseBind(const maniSkin* skin, uint32_t joint);
```
Returns a joint's inverse bind matrix.  @param skin   An opened skin. @param joint  The joint. @return The matrix, or the identity for a joint past the count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniComputePalette(const maniPose* pose, const maniSkeleton* skeleton, const maniSkin* skin, maniAffine* palette, uint32_t capacity, uint32_t* countOut);
```
Computes a skin's palette: entry j is the model matrix of joint j's bone times the joint's inverse bind matrix.  @param pose      A pose of the skeleton with its model composed. @param skeleton  The skeleton the skin is bound to. @param skin      The skin. @param palette   Receives the entries; may be NULL when capacity is 0. @param capacity  The array's length. @param countOut  Receives the joint count. May be NULL. @return `mani_success`; `mani_errorCapacity` when the array is shorter than the joint count, with the count written and nothing else; `mani_errorInvalid` for a NULL argument, a skeleton that is not the skin's or the pose's, a pose whose model is stale, or a difference. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

## `skinning.h`

Skinning (mani-0021): a mesh primitive's influences, bound to a skin, over the caller's bytes. Groups of four influences a vertex, weights decreasing and summing to 1 (or all 0, a vertex left as it is).

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenInfluences(const void* bytes, size_t size, maniInfluences* influencesOut);
```
Opens an influences file in place: checks the container, the sizes, every joint against the skin's joint count the file records, every weight finite and from 0 to 1, the dual quaternion weights and the centers when present, and the identity.  @param bytes          The file, 16-byte aligned. @param size           Its size in bytes; bytes past the file are ignored. @param influencesOut  Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint32_t maniInfluencesVertexCount(const maniInfluences* influences);
```
Returns the number of vertices.  @param influences  Opened influences. @return The vertex count. @par Thread safety Safe from any thread.

```c
uint32_t maniInfluencesPerVertex(const maniInfluences* influences);
```
Returns the influences a vertex holds: four times the group count, the last ones of weight 0 where a vertex has fewer.  @param influences  Opened influences. @return The influences a vertex. @par Thread safety Safe from any thread.

```c
bool maniInfluencesFitSkin(const maniInfluences* influences, const maniSkin* skin);
```
Returns whether the influences are a skin's: its identity and joint count are the ones recorded.  @param influences  Opened influences. @param skin        An opened skin. @return True when bound to the skin. @par Thread safety Safe from any thread.

```c
uint32_t maniInfluence(const maniInfluences* influences, uint32_t vertex, uint32_t slot, float* weightOut);
```
Returns one influence of a vertex: its joint (an index into the skin's joints) and weight; joint 0 at weight 0 past the vertex count or the influences.  @param influences  Opened influences. @param vertex      The vertex. @param slot        The influence, 0 the heaviest. @param weightOut   Receives the weight; may be NULL. @return The joint. @par Thread safety Safe from any thread.

```c
float maniInfluencesDualQuaternionWeight(const maniInfluences* influences, uint32_t vertex);
```
Returns a vertex's blend between linear (0) and dual quaternion (1) skinning, FBX's blended method; 0 for influences without them or past the vertex count.  @param influences  Opened influences. @param vertex      The vertex. @return The weight, 0 to 1. @par Thread safety Safe from any thread.

```c
bool maniInfluencesCenter(const maniInfluences* influences, uint32_t vertex, maniVec3* centerOut);
```
Reads a vertex's center of rotation, built offline.  @param influences  Opened influences. @param vertex      The vertex. @param centerOut   Receives the center; unchanged on false. @return True when the influences hold centers and the vertex is in range. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSkinLinear(const maniSkinJob* job);
```
Linear blend skinning: each vertex moved by the weighted sum of its joints' palette matrices, its directions by the sum's 3x3 part, its normals by that part's cofactor times its determinant's sign (the inverse transpose's direction), directions renormalized (zero when they collapse). A vertex without weight is copied.  @param job  The job. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a palette count other than the influences' joint count, a range past the vertices, or a stream without pointers, with a stride too small or of an unknown kind (nothing written). @par Thread safety Safe from any thread; the output range is used by one thread at a time. Jobs over disjoint ranges run at once.

```c
MANI_NODISCARD MANI_API maniResult maniSkinDualQuaternion(const maniSkinJob* job);
```
Dual quaternion skinning in two phases (Kavan et al. 2008): each vertex's scale and shear parts summed at their weights and applied first, then its joints' dual quaternions blended (each turned to the first's hemisphere) and normalized. Streams as maniSkinLinear treats them; a vertex with one influence moves exactly as under linear skinning, up to rounding.  @param job  The job, its dualQuaternions given. @return As maniSkinLinear, and `mani_errorInvalid` without dual quaternions. @par Thread safety Safe from any thread; the output range is used by one thread at a time. Jobs over disjoint ranges run at once.

```c
MANI_NODISCARD MANI_API maniResult maniSkinBlended(const maniSkinJob* job);
```
FBX's blended skinning: each vertex moved by `(1 - b)` times its linear matrix plus `b` times its dual quaternion one, b its weight in the influences (0 where they have none). A vertex at 0 or 1 gives the bits of maniSkinLinear or maniSkinDualQuaternion.  @param job  The job, its palette and dualQuaternions given. @return As maniSkinDualQuaternion. @par Thread safety Safe from any thread; the output range is used by one thread at a time. Jobs over disjoint ranges run at once.

```c
MANI_NODISCARD MANI_API maniResult maniSkinCenters(const maniSkinJob* job);
```
Skinning with optimized centers of rotation (Le and Hodgins 2016) in two phases: each vertex's scale and shear parts summed at their weights and applied first (to the vertex and its center c), then the rotation R of its joints' quaternions blended in the first's hemisphere, and the translation that carries the scaled center where linear blending of the rigid parts takes it: `v' = R (S v - S c) + sum w_j (R_j S c + t_j)`. No collapse and no bulge; a vertex with one influence moves as under linear skinning.  @param job  The job, its palette and dualQuaternions given, its influences with centers (maniBuildCenters). @return As maniSkinDualQuaternion, and `mani_errorInvalid` for influences without centers. @par Thread safety Safe from any thread; the output range is used by one thread at a time. Jobs over disjoint ranges run at once.

```c
MANI_NODISCARD MANI_API maniResult maniPaletteDualQuaternions(const maniAffine* palette, uint32_t count, float* out);
```
Writes a palette for dual quaternion skinning, on the CPU or in a shader: each matrix A split as A = R S (R the rotation of its polar decomposition, S = R^T A carrying scale, shear and mirror), 20 floats a matrix: R's quaternion (x, y, z, w with w >= 0), the dual part (half the translation times it), then S as three rows of four (the fourth 0).  @param palette  count matrices, as maniComputePalette writes them. @param count    The number of matrices. @param out      Receives 20 floats a matrix. @return `mani_success`; `mani_errorInvalid` for a NULL pointer with a count or a matrix not finite (out unchanged). @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniPaletteRows(const maniAffine* palette, uint32_t count, float* rows);
```
Writes a palette for shaders: each matrix as three rows of four floats (row-major 3x4), the translation last in each row.  @param palette  count matrices, as maniComputePalette writes them. @param count    The number of matrices. @param rows     Receives 12 floats a matrix. @return `mani_success`; `mani_errorInvalid` for a NULL pointer with a count. @par Thread safety Safe from any thread.

## `spring.h`

Spring bones (mani-0026): chains of bones that sway behind the motion as VRMC_springBone 1.0 defines them, with its extended colliders (inside spheres and capsules, planes) and angle limits (cones, hinges, spherical limits). A set is built offline for one skeleton, its per-frame values converted to a fixed step, and opened in place.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenSpringSet(const void* bytes, size_t size, maniSpringSet* setOut);
```
Opens a spring set file in place: checks the container and every record (bones within the skeleton's count, shapes, chains each joint above the next as far as the file can tell, links, names, every float finite and in its range).  @param bytes   The file, 16-byte aligned. @param size    Its size in bytes; bytes past the file are ignored. @param setOut  Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint64_t maniSpringSetIdentity(const maniSpringSet* set);
```
Returns a set's identity: the hash of its sections.  @param set  An opened set, or NULL. @return The identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
uint64_t maniSpringSetSkeletonIdentity(const maniSpringSet* set);
```
Returns the identity of the skeleton a set was built for.  @param set  An opened set, or NULL. @return The identity; 0 for NULL. @par Thread safety Safe from any thread.

```c
uint32_t maniSpringSetSpringCount(const maniSpringSet* set);
```
Returns the number of springs (chains) in a set.  @param set  An opened set, or NULL. @return The count; 0 for NULL. @par Thread safety Safe from any thread.

```c
const char* maniSpringSetSpringName(const maniSpringSet* set, uint32_t spring, uint32_t* lengthOut);
```
Returns a spring's name, its bytes inside the set (not terminated).  @param set        An opened set. @param spring     The spring's index. @param lengthOut  Receives the name's length in bytes (0 when the spring has none or the call fails). May be NULL. @return The name's first byte; NULL for a NULL set or an index past the springs, and an empty name's position for none. @par Thread safety Safe from any thread.

```c
float maniSpringSetStepRate(const maniSpringSet* set, float* referenceRateOut);
```
Returns the rate a set steps at (steps a second), and the rate its source's values were per frame of.  @param set               An opened set, or NULL. @param referenceRateOut  Receives the reference rate. May be NULL. @return The step rate; 0 for NULL. @par Thread safety Safe from any thread.

```c
size_t maniSpringStateSize(const maniSpringSet* set);
```
Returns the bytes a set's state needs.  @param set  An opened set. @return The size; 0 for NULL. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniInitSpringState(const maniSpringSet* set, const maniSkeleton* skeleton, void* memory, size_t size, maniSpringState** stateOut);
```
Makes a state for a set in caller memory, with no motion yet: the first step places every tail where the pose puts it.  @param set       An opened set, which must outlive the state. @param skeleton  The skeleton the set was built for, which must outlive the state. @param memory    At least maniSpringStateSize bytes, MANI_SPRING_STATE_ALIGNMENT aligned. @param size      Its size. @param stateOut  Receives the state (at memory). @return `mani_success`; `mani_errorInvalid` for a NULL argument, misaligned memory or a skeleton of another identity; `mani_errorCapacity` for too little memory. @par Thread safety Safe from any thread; the state is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniStepSprings(maniSpringState* state, maniPose* pose, const maniTransform* placement, const maniVec3* force, float delta);
```
Moves a state on by delta seconds (its magnitude, at most MANI_SPRING_MAX_STEPS steps' seconds) toward a pose: runs the steps due at the set's step rate, each with the pose, placement and force interpolated between the last call's and these at its time (positions and forces linearly, rotations along the shorter arc); time short of a step is kept for the next call. Tails move by inertia less drag, toward the pose's direction by stiffness and by gravity and the external force in the world (mani-0026), keep their bone's length, leave colliders (or stay inside inside ones) and keep within their limits; a spring with a center bone carries its inertia in the center's space.  @param state      A state. @param pose       The animated pose at the end of delta (its model space computed when stale). @param placement  The character's place in the world at the end of delta (translation and rotation; its scale is not read), or NULL for the identity. @param force      A force in the world on every joint at the end of delta (wind), in gravity's units (a length a second, times the head's scale), or NULL for none (a zero force: after a force it ramps down over the call; a call of no time without one first stops it at once). The first call of a new or reset state takes its force for both ends. @param delta      The time in seconds, finite, either sign. @return `mani_success`; `mani_errorInvalid` for a NULL state or pose, a pose of another bone count or holding a difference, or a delta, placement or force not finite. @par Thread safety Safe from any thread; the state and pose are used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniApplySprings(const maniSpringState* state, maniPose* pose);
```
Turns each spring joint of a pose toward its tail: the last step's tails moved on at their velocity for the time since that step (the last step's own at a host rate the step rate divides), carried into the placement of the last call, then pushed out of their colliders as the last call left them (where a step left a tail inside one, the output is pushed further than the step's own). Joints with no motion yet keep the pose's rotation.  @param state  A state. @param pose   The pose the last step took (its model space computed when stale); joints' local rotations are written. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a pose of another bone count or holding a difference. @par Thread safety Safe from any thread; the state and pose are used by one thread at a time.

```c
void maniResetSprings(maniSpringState* state);
```
Drops a state's motion: the next step places every tail where the pose puts it (a teleport) and takes its placement and force as the last call's.  @param state  A state, or NULL (nothing done). @par Thread safety Safe from any thread; the state is used by one thread at a time.

```c
size_t maniSpringSavedSize(const maniSpringSet* set);
```
Returns the bytes a state's saved form takes.  @param set  An opened set. @return The size; 0 for NULL. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSaveSpringState(const maniSpringState* state, void* out, size_t capacity);
```
Saves a state without pointers: its set's identity and its values.  @param state     A state. @param out       At least maniSpringSavedSize bytes. @param capacity  Their size. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorCapacity` for too small a buffer. @par Thread safety Safe from any thread; the state is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniFileResult maniRestoreSpringState(maniSpringState* state, const void* bytes, size_t size);
```
Restores a saved state after checking it as external bytes: the set's identity, the size, every value finite and in its range, rotations of length 1, no values but zeros without motion.  @param state  A state of the same set. @param bytes  The saved bytes. @param size   Their size. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorFormat` with the offset of the first byte found wrong (the state unchanged). @par Thread safety Safe from any thread; the state is used by one thread at a time.

## `trajectory.h`

A character's trajectory predicted for motion matching (mani-0024): Holden's critically damped springs, in closed form, take a simulated root toward a desired velocity and rotation; its samples, predicted or recorded, become the query values of a database's trajectory channels.

```c
MANI_NODISCARD MANI_API maniResult maniAdvanceTrajectory(maniTrajectory* trajectory, const maniTrajectoryGoal* goal, float seconds);
```
Moves a trajectory by seconds toward its goal, each spring in closed form (y = 2 ln 2 / (half-life + 1e-5); the position's with its acceleration, Holden's simulation_positions_update; the rotation's on the shorter arc, his simple_spring_damper_exact), in binary32: one advance is many shorter ones up to rounding, so a prediction t seconds ahead is a copy advanced by t.  @param trajectory  The trajectory; finite, its rotation unit within 1e-3. @param goal        Finite, its rotation not zero (normalized), its half-lives not negative. @param seconds     Finite, not negative. @return `mani_success`; `mani_errorInvalid` for a NULL argument or one out of range (the trajectory then unchanged). @par Thread safety Safe from any thread; the trajectory is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniTrajectoryFeatures(const maniMotionDatabase* database, const maniTrajectory* root, const maniTrajectory* sample, float* positionOut, float* facingOut);
```
Gives a sample's trajectory values as a database measures its trajectories: the sample's offset from the root and the database's forward axis turned by the sample's rotation, both in the root's frame, on the ground plane (the two axes besides the database's up axis, in order); the values of a trajectory position and a trajectory facing query parameter.  @param database     An opened database. @param root         The character's root now (its position and rotation; the rest unread). @param sample       A sample predicted ahead or recorded behind. @param positionOut  Receives 2 values, or NULL. @param facingOut    Receives 2 values, or NULL. @return `mani_success`; `mani_errorInvalid` for a NULL database, root or sample. @par Thread safety Safe from any thread.

---

321 functions across 27 headers.
