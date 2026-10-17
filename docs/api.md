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
Destroys a builder and the file it holds.  @param builder  The builder, or NULL. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
uint32_t maniBuilderMisuses(const maniBuilder* builder);
```
Returns how many calls on the builder were refused for invalid arguments (conventions section 8).  @param builder  The builder. @return The count. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

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
size_t maniGraphBindingSize(const maniGraph* graph);
```
Returns the bytes a binding of the graph needs.  @param graph  An opened graph. @return The size; 0 for a NULL graph. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniBindGraph( const maniGraph* graph, const maniSkeleton* skeleton, const maniClip* const* clips, uint32_t count, const maniMotionDatabase* const* databases, uint32_t databaseCount, void* memory, size_t size, maniGraphBinding** bindingOut, uint32_t* unboundOut);
```
Binds a graph to its skeleton, clips and motion databases: each clip slot to the clip of its identity among clips (in any order, others ignored), each of the graph's skeleton and of the kind the slot was built with; each database slot to the database of its identity among databases, of the graph's skeleton, whose clips are the slots the graph lists for it and whose channels take the graph's query parameters. The graph, skeleton, clips and databases must outlive the binding.  @param graph          An opened graph. @param skeleton       The skeleton the graph was built against. @param clips          count opened clips. @param count          The number of clips. @param databases      databaseCount opened databases. @param databaseCount  The number of databases. @param memory         At least maniGraphBindingSize bytes, MANI_GRAPH_BINDING_ALIGNMENT-aligned. @param size           The memory's size. @param bindingOut     Receives the binding. @param unboundOut     Receives the first slot left unbound on failure: a clip slot, or the graph's clip count plus a database slot (MANI_NO_TRACK otherwise). May be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL or misaligned argument, a skeleton that is not the graph's, a slot no clip or database binds, or clips whose root motion names different bones; `mani_errorCapacity` when the memory is too small. @par Thread safety Safe from any thread.

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
MANI_NODISCARD MANI_API maniResult maniGetGraphParameter(const maniGraphInstance* instance, uint32_t parameter, maniGraphValue* valueOut);
```
Reads a parameter's value.  @param instance   An instance. @param parameter  The parameter. @param valueOut   Receives the value. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a parameter past the count. @par Thread safety Safe from any thread; the instance is used by one thread at a time.

```c
size_t maniGraphStateSize(const maniGraphBinding* binding);
```
Returns the bytes an instance's saved state takes: its parameters, its nodes' state and its state machines' and one-shots', with the graph's identity.  @param binding  A binding. @return The size; 0 for a NULL binding. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSaveGraphInstance(const maniGraphInstance* instance, void* out, size_t capacity);
```
Writes an instance's state as bytes that hold no pointer, to be restored into any instance of the same graph.  @param instance  An instance. @param out       At least maniGraphStateSize bytes. @param capacity  The bytes out holds. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorCapacity` when capacity is too small (nothing written). @par Thread safety Safe from any thread; the instance is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniFileResult maniRestoreGraphInstance(maniGraphInstance* instance, const void* bytes, size_t size);
```
Restores a saved state, checked as external bytes: the graph's identity, the size, every parameter in form and every node's, state machine's and one-shot's state finite and in range. The instance's cursors start over (they cache nothing a pose depends on).  @param instance  An instance of the graph the state was saved from. @param bytes     The state. @param size      Its size in bytes. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorFormat` with the offset of the first byte found wrong (the instance unchanged). @par Thread safety Safe from any thread; the instance is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniUpdateGraph(maniGraphInstance* instance, float delta, maniGraphEvent* events, uint32_t capacity, uint32_t* eventCountOut, maniRootMove* rootMotionOut);
```
Moves an instance through time: shares each node's weight down from the output (normalized blends, masks by their largest bone weight, additive layers by their weights, the selected input), moves every clip node's playhead by the step times its speed and the time scales above it (a seek whose trigger is set moving its playheads to its time instead, and consuming the trigger), lists the events clip nodes weighed past MANI_GRAPH_EVENT_THRESHOLD pass in node order, and blends the clips' root motion by the weights the output gives it (masks by their weight at the root motion bone, additive layers none).  @param instance      An instance. @param delta         The step in seconds, finite, either sign. @param events        Receives up to capacity events. May be NULL with a capacity of 0. @param capacity      The events events holds. @param eventCountOut Receives the true count, which may pass the capacity (saturating at UINT32_MAX). May be NULL. @param rootMotionOut Receives the update's root motion. May be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL instance, events NULL with a capacity, or a step not finite (nothing changed). @par Thread safety Safe from any thread; the instance is used by one thread at a time.

```c
size_t maniGraphScratchSize(const maniGraphBinding* binding);
```
Returns the bytes of the scratch an evaluation of a bound graph's instances needs: a pose per slot and room for one node's layers. One scratch serves any number of instances, one at a time.  @param binding  A binding. @return The size; 0 for a NULL binding. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniEvaluateGraph(maniGraphInstance* instance, void* scratch, size_t size, maniPose* pose);
```
Writes an instance's pose as its last update weighed it: each node of weight above 0, in order, into its slot (a clip sampled at its playhead through its cursor; blends, masks and additive layers by the blending functions over their weighed inputs; a select, time scale or seek passing its input on), then the output's. The rest pose before the first update.  @param instance  An instance. @param scratch   At least maniGraphScratchSize bytes, MANI_GRAPH_SCRATCH_ALIGNMENT-aligned, used by one thread at a time. @param size      The scratch's size. @param pose      A pose of the graph's skeleton. @return `mani_success`; `mani_errorInvalid` for a NULL or misaligned argument or a pose of another skeleton; `mani_errorCapacity` when the scratch is too small. @par Thread safety Safe from any thread; the instance, scratch and pose are used by one thread at a time.

## `graphbuild.h`

Building a graph file (mani-0023), offline: a graph's parameters, clip slots, sync groups and nodes, checked against one skeleton and its clips, ordered so each node follows its inputs, its poses given slots, and written.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildGraph(maniBuilder* builder, const maniGraphSource* source);
```
Builds a graph file: inlines subgraphs (their nodes after the source's, in order, nested ones after them; refusals name nodes by their place there), checks every parameter, clip slot, sync group, database slot and node, orders the nodes so each follows its inputs (the output last, the source's order kept where it may be), makes each mask's bone weights, marks each node as giving poses or differences, gives each node's pose a slot reused once its last reader has run, and keeps the file in place of any built before.  @param builder  The builder. @param source   The graph. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse) or with the reason, the item and another the reason names for a refused source: Count (more items than the named limits, or none of nodes), Name, DuplicateName and NameHash with the list bits of parameters, clips, sync groups or databases, Parameter, Clip, Database, Node, Input, Cycle, Unreachable, Scalar, Weight and Difference; and `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniWriteGraphJson(maniBuilder* builder, const maniGraphSource* source);
```
Writes a graph source as its JSON text form (UTF-8, two-space indents, a line per list item, ending in a new line), building it first: a source maniBuildGraph refuses is refused the same way and not written. Parameters, clip slots, sync groups, database slots (a "databases" list, when the graph has any) and the bones of mask chains are written by name, nodes and the output by their place in the source, subgraphs as nested graphs with their parameters mapped by name, and numbers as the shortest text that reads back to the same binary32 (ECMAScript's layout). The text is kept in place of any file or text kept before; maniCopyBuilt copies it.  @param builder  The builder. @param source   The graph. @return As maniBuildGraph. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildGraphJson( maniBuilder* builder, const char* text, size_t length, const maniSkeleton* skeleton, const maniGraphClipSource* clips, uint32_t count, const maniGraphDatabaseSource* databases, uint32_t databaseCount);
```
Builds a graph file from its JSON text form, as maniWriteGraphJson writes it: members an object does not take are refused; those with an off value may be left out (a parameter's default: false, 0, the zero vector or the identity quaternion; bounds, both or neither; allEvents false; a clip node's syncGroup null, speed 1, loop "clip", leader false; a blend's direct false; a mask's chains empty and defaultWeight 0; a chain's subtree false; a seek's time 0; a 1D space's period 0, a banded space's polar false, a triangulated space's triangles none (its Delaunay triangulation), a position's y 0; a machine's entry 0, chainLimit 0 (4), transitions and finals empty; a transition's priority 0, interruption "none", entry "restart", fade "linear", duration and cooldown 0 and conditions empty; a one-shot's fadeOutTrigger and abort none, curve "linear", fadeIn and fadeOut 0, mask none (the whole body); an inertialize node's trigger none, method "inertialization", duration 0 and, for "deadBlending", halfLifeScale 0.3, halfLifeMin 0.1 and halfLifeMax 1.0; a motion matching node's trigger none, query empty, period 0.1, jumpThreshold, endExclusion, historyTime, continuingBias, transitionCost and historyCount 0, requiredTags and excludedTags 0 (a whole number, or {"parameter": name}) and duration 0; parameters, clips, syncGroups and databases empty). Each clip slot name, at any nesting, takes the host's clip of that name, and each database slot name the host's database of that name (the first of equal names). The source read is then built as maniBuildGraph builds it.  @param builder    The builder. @param text       The UTF-8 text (a leading byte order mark skipped); may be NULL when length is 0. @param length     The text's length in bytes. @param skeleton   The graph's skeleton; mask chains name its bones. @param clips      count clips with their slot names; may be NULL when count is 0. @param count      The number of clips. @param databases  databaseCount databases with their slot names; may be NULL when databaseCount is 0. @param databaseCount The number of databases. @return As maniBuildGraph, items by place in the text's lists; or `mani_errorInvalid` with maniBuildReasonJson or maniBuildReasonForm (other a maniGraphForm), item the byte offset. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

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
Opens a glTF document from JSON or GLB bytes (told apart by GLB's magic), checking the container, the asset version, required extensions, buffers, buffer views and accessors. `data:` buffers are decoded into the document; external buffers wait for maniGltfSupplyBuffer.  @param builder  A builder; the document uses its allocator and must be closed before the builder is destroyed. @param bytes    The file; kept alive and unchanged while the document is open. @param size     Its size in bytes. @param gltfOut  Receives the document. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorFormat` with a reason, an offset and an item; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

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
Returns an external buffer's URI, as the JSON gives it after its escapes (a URI reference relative to the document; percent escapes are left to the host, which resolves it), or NULL for a buffer the document holds (GLB or `data:`) or past the count.  @param gltf       The document. @param buffer     The buffer. @param lengthOut  Receives the URI's length in bytes, 0 for NULL. May be NULL. @return The URI, UTF-8 and not terminated, or NULL. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfSupplyBuffer(maniGltf* gltf, uint32_t buffer, const void* bytes, size_t size);
```
Supplies an external buffer's bytes, which the caller keeps alive and unchanged while the document is open. They must hold the buffer's `byteLength`; bytes past it are ignored.  @param gltf    The document. @param buffer  An external buffer. @param bytes   Its bytes. @param size    Their size. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a buffer that is not external; `mani_errorFormat` with maniImportReasonBuffer for too few bytes. @par Thread safety Safe from any thread; the document is used by one thread at a time.

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
Reads the document's `VRMC_springBone` extension (version 1.0) as spring bone sources on an opened skeleton (mani-0011): colliders, collider groups and springs, nodes found by bone name, offsets, tails and gravity directions in the output's basis. A document without the extension gives an empty set. The arrays are the document's until the next call or maniCloseGltf.  @param gltf      The document. @param skeleton  An opened skeleton holding the springs' nodes. @param options   The options the skeleton was built with. @param setOut    Receives the sources. @return `mani_success`; `mani_errorInvalid` for a NULL argument or options out of range; `mani_errorFormat` with a reason; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the document is used by one thread at a time.

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
Builds a primitive's influences into the builder, bound to an opened skin built from the glTF skin that deforms it (its joints in the skin's order): every `JOINTS_n` and `WEIGHTS_n` set read, zero weights dropped, then sorted and renormalized as maniBuildInfluences does.  @param builder    The builder. @param gltf       The document. @param mesh       The mesh. @param primitive  The primitive. @param skin       The opened skin. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a mesh or primitive past its count; `mani_errorFormat` with a reason: Property (no `JOINTS_0`, sets not paired, an accessor of the wrong type or count), Extension (Draco-compressed), Influence, Data, Pending, Build; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildMorphTargets(maniBuilder* builder, const maniGltf* gltf, uint32_t mesh, uint32_t primitive, const maniGltfOptions* options);
```
Builds a primitive's morph targets into the builder: a channel a target, at full weight 1, named by the mesh's `extras.targetNames` when it names every target and by the target's index otherwise (weight tracks are the node's bone name, a slash and that name); positions, normals, tangents, texture coordinate and color sets read (other semantics left), positions, normals and tangents in the output's basis, VEC3 colors with alpha 0; a target keeps the vertices it moves.  @param builder    The builder. @param gltf       The document. @param mesh       The mesh. @param primitive  The primitive. @param options    The options the skeleton was built with. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a mesh or primitive past its count, a primitive without targets or options out of range; `mani_errorFormat` with a reason: Property (a semantic the primitive lacks, an accessor of the wrong type or count, more than 32 attributes), Index, Extension (Draco), Data, Pending, Build; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

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
Destroys a trainer.  @param trainer  The trainer, or NULL. @par Thread safety Safe from any thread; the trainer is used by one thread at a time.

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
Builds a motion database: samples every entry's frames, each channel at each of its times (clamped to the entry's ends, or wrapped when it loops), in the root's space at the frame (the clip's root motion carrying the root between times; a bone's velocity the difference of its positions a frame either side over their time apart), normalizes each channel, and bounds every run of 16 and 64 frames. Keeps the file in place of any built before.  @param builder  The builder. @param source   The database. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse), or with the reason and item for a refused source: `maniBuildReasonTiming` (the rate), `maniBuildReasonOption` (the axes), `maniBuildReasonCount` (no channels, clips or entries, too many, or past MANI_MATCH_MAX_FRAMES frames or MANI_MATCH_MAX_FEATURES features), `maniBuildReasonJoint` (a clip of another skeleton or additive; item the clip), `maniBuildReasonChannel` (item the channel), `maniBuildReasonEntry` (item the entry), `maniBuildReasonMissingTrack` (a float track an entry's clip lacks; item the channel, other the entry), `maniBuildReasonSpread` (a channel that never varies; item the channel); `mani_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

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

237 functions across 19 headers.
