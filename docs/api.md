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
Copies the last built file into caller memory.  @param builder   The builder. @param bufferOut Receives the bytes: at least capacity bytes; may be NULL when capacity is 0. Open the copy from a 16-byte aligned address. @param capacity  The buffer's size in bytes. @param sizeOut   Receives the file's size in bytes. May be NULL. @return `mani_success`; `mani_errorCapacity` when the buffer is smaller than the file, with the size still written; `mani_errorInvalid` for a NULL builder, a NULL buffer with a positive capacity, or a builder holding no file. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

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

## `import.h`

glTF 2.0 import (mani-0008), offline: a document opened from JSON or GLB bytes the caller keeps alive, its external buffers supplied by the host, then skeletons, skins and clips built from it.

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
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildClip(maniBuilder* builder, const maniGltf* gltf, uint32_t animation, const maniSkeleton* skeleton, const maniGltfOptions* options, float sampleRate, maniLoopPolicy loop, uint32_t* boundOut);
```
Builds one animation as a clip of an opened skeleton (mani-0010): translation, rotation and scale channels on its bones (found by name), morph weights of its bones and of meshes it skins, and `KHR_animation_pointer` channels to anything but nodes, each as float tracks; the basis change applied and, with placement folded, the nodes above the root composed in. The duration is the animation's latest key over every channel.  @param builder     The builder. @param gltf        The document, its buffers supplied. @param animation   The animation. @param skeleton    An opened skeleton. @param options     The options the skeleton was built with. @param sampleRate  Samples per second; 0 for 30. @param loop        The clip's loop policy. @param boundOut    Receives how many of the animation's channels the clip holds. May be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL argument, an animation past the count or options out of range; `mani_errorFormat` with a reason (Data with the channel as item for keys the clip builder refuses); `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

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
Builds a set of morph targets into the builder, as ufbx reads them: a channel a BlendShapeChannel (a Shape child before 7.2), its shapes the targets at FullWeights / 100 (as the clips order them), a vertex a control point; positions and normals as offsets in the options' axes (positions in its unit too), indices past the control points skipped, Blender's per-vertex weights folded in; a shape after another of the same full weight left out, as is a channel without shapes. Channels are named as their objects; a target as its shape, else as its channel with '/' and its index past the first, as weight tracks are.  @param builder  The builder. @param fbx      The document. @param morph    The set. @param options  The options the skeleton was built with. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a set past the count or options out of range; `mani_errorFormat` with a reason: Property (no control points, a shape's arrays of mismatched sizes), Data, Build (a full weight of 0, or a name not UTF-8; the item the builder's reason); `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

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
MANI_NODISCARD MANI_API maniImportResult maniFbxBuildClip( maniBuilder* builder, const maniFbx* fbx, uint32_t clip, const maniSkeleton* skeleton, const maniFbxOptions* options, float sampleRate, maniLoopPolicy loop, uint32_t* boundOut);
```
Builds a clip into the builder for an opened skeleton (mani-0016): the stack's layers and curves evaluated as ufbx evaluates them, at every sample of the clip's grid from the stack's start (its local, reference or take times, else its keys), whole ticks apart. Each bone of the skeleton named as a model (or a model's helper, `name#unscaled`) takes its local as a skeleton built from the file formed its rest, the options' axes and unit applied and, with placement folded, the placement composed into the root; a channel that never leaves the skeleton's rest is left out. Other numbers a bone's model animates become float tracks named `bone/property` (`/0` to `/2` added for vectors).  @param builder     The builder. @param fbx         The document. @param clip        The clip, below maniFbxClipCount. @param skeleton    An opened skeleton. @param options     The options the skeleton was built with. @param sampleRate  Samples per second; 0 for the scene's frame rate up to 120, else 30. @param loop        The clip's loop policy. @param boundOut    Receives how many of the stack's channel bindings drive models the clip reads. May be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a clip past the count, a rate not finite or negative, or options out of range; `mani_errorFormat` with a reason (Property with the clip as item for a stack's curves or channels refused, or with the model for an animated integer input out of range; Matrix with the model for a local that does not decompose at a sample; Name with the model for a repeated float track name; Build for a timing the builder refuses); `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxModelPlacement(const maniFbx* fbx, uint32_t model, const maniFbxOptions* options, maniAffine* placementOut);
```
Returns where a model skeleton's root hangs, as maniFbxSkeletonPlacement.  @param fbx           The document. @param model         The root model. @param options       The options. @param placementOut  Receives the placement. @return As maniFbxSkeletonPlacement. @par Thread safety Safe from any thread.

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

---

164 functions across 10 headers.
