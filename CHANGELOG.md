# Changelog

All notable changes to this project are recorded here, grouped as
Added, Changed, Fixed, Removed and Security. The project uses
[Semantic Versioning](https://semver.org/). Before 1.0.0, any release
may change the API, the ABI and every data format.

## [0.10.0](https://github.com/Rawframe-Project/maul-anim/compare/v0.9.0...v0.10.0) (2026-11-05)


### ⚠ BREAKING CHANGES

* **skin:** `maniSkin`, `maniInfluences` and `maniMorphTargets` change their fields. A base, a size and each section's offset (and size) replace the section pointers and #251's counts, and a cookie is added. Every call checks the view, refusing a view no open wrote, or one whose counts do not fit its file, with `mani_errorInvalid` or the getter's documented none.
* **morph:** `maniMorphTargets` gains `indexCount`, `deltaCount` and `nameBytes`. `maniMorphTargets_GetChannelTargets` gives 0 targets for a range that falls outside the targets.
* **clip:** `maniClip`'s fields change: a base, a size and each section's offset and size replace the section pointers and the names' size, and a cookie is added. Every call checks the view, refusing a view no open wrote, or one whose counts do not fit its file, with `mani_errorInvalid` or the getter's documented none; the getters, which read the fields unchecked before, now return those nones too. `maniClip_GetEventRange` writes an empty range for such a view.
* **skeleton:** `maniSkeleton`'s fields change: base, size and section offsets replace the section pointers, and a cookie is added. Every call checks the view, refusing a view no open wrote, or one whose counts do not fit its file, with `mani_errorInvalid` or the getter's documented none. `maniPose_ComputeModel`, `maniPose_SetRest` and `maniInitPose` give `mani_errorFormat` for a parent in the skeleton's bytes that is not a lower index.
* **ik:** `maniPose_SolveBodySdls` and `maniPose_SolveBodyXpbd` give different results for a part whose offsets' largest component is past 2^24 or below 2^-24, which they solved wrongly from about 2^64 and 2^-70 on. Other figures keep their bits.
* **ik:** `maniPose_SolveAim` carries `forward` and `up` by the aiming bone's whole model frame. Under a bone whose model frame holds a scale other than 1 or a mirror, the turns change: the bone's real forward now points at the goal. Rigs without scales keep their bits.
* **ik:** `maniPose_SolveAim` carries `offset` by the aiming bone's whole model frame. Under a bone whose model frame holds a scale other than 1 or a mirror, the offset now moves with that scale and mirror, and the turns change. Rigs without scales keep their bits.
* **ik:** `maniPose_SolveAim` gives different turns where its goal from the aiming bone or its offset is longer than about 2^64 or shorter than about 2^-75, which it aimed wrongly or not at all. `maniPose_SolveCcd` and `maniPose_SolveFabrik` turn chains whose bones are shorter than about 2^-75, which they left as they were. A FABRIK pole, a two-bone hinge, or an aim's up or pole shorter than about 2^-75 now acts as given instead of as none. Other finite inputs keep their bits.
* **ik:** `maniPose_SolveTwoBone` gives different turns for a chain whose bones' largest component is past 2^24 or below 2^-24, which main turned wrongly from about 2^32 and 2^-32 on. Every IK solver gives different turns under a mirrored parent whose determinant, a product of three scales, underflowed to zero in binary32, such as a uniform scale below about 2^-50. Other inputs keep their bits.
* **learned:** the learned model format version is 3. The header no longer holds the weight format, and the step rate and axes move 4 bytes earlier. The weight format is section 12, written only for 16-bit weights. A model's bytes and identity change. Models of format version 2 are refused with mani_errorVersion; build or train them again.
* **format:** maniBuildGraphJson, maniBuildRetargetMapJson and maniBuildMirrorMapJson refuse a key the reader does not know, and an integer version other than 1, with mani_errorVersion (maniBuildReasonForm, maniGraphFormMember or maniGraphFormVersion, and the byte offset). They returned mani_errorInvalid.
* **match:** the motion database format version is 5. The header no longer holds the leaf size, and the node count moves to +48. The leaf size is section 14, written only off its default of 16 frames. A database's bytes and identity change. Databases of format version 4 are refused with mani_errorVersion; build them again from their source.
* **retarget:** the retargeting map format version is 4. The header holds only the skeletons' identities and bone counts. The pelvis block is section 5, written only for a map with a pelvis. maniRetargetMap's header field is now pelvis. A map's bytes and identity change. Maps of format version 3 are refused with mani_errorVersion; build them again from their source.
* **clip:** the clip format version is 5. The clip header drops the loop policy, the form word and the additive basis. The loop policy (tag 17) and the additive basis (tag 18) are optional sections, each written only off its default. A clip's bytes and identity change, and so do the bytes of a graph that names it. Clips of format version 4 are refused with mani_errorVersion; build or compress them again from their source.
* **format:** opening a file is stricter. Every opener refuses, with mani_errorFormat, a section not placed where the writer puts it, a nonzero byte between sections, bytes past the last section, an empty section, and a tag of 0. Every asset opener refuses a section tag past its kind's last with mani_errorVersion (it was mani_errorFormat, or the section was ignored for retargeting maps, mirror maps, ragdoll profiles and spring sets). No file the library writes changes, and no format version moves.

### Fixed

* **clip:** check the clip view on every call ([#249](https://github.com/Rawframe-Project/maul-anim/issues/249)) ([c8f58ea](https://github.com/Rawframe-Project/maul-anim/commit/c8f58ea4ca99b0b4908c8d6eedef029b485179ae))
* **clip:** store the loop policy and additive basis as optional fields ([#221](https://github.com/Rawframe-Project/maul-anim/issues/221)) ([a510f95](https://github.com/Rawframe-Project/maul-anim/commit/a510f95f87588fe9d81e2ba1ee8c734179de7eeb)), closes [#220](https://github.com/Rawframe-Project/maul-anim/issues/220) [#86](https://github.com/Rawframe-Project/maul-anim/issues/86)
* **format:** leave keys at their defaults out of the map JSON forms ([#233](https://github.com/Rawframe-Project/maul-anim/issues/233)) ([b08eea4](https://github.com/Rawframe-Project/maul-anim/commit/b08eea4801b086157704714670c03c067e48b9c6)), closes [#232](https://github.com/Rawframe-Project/maul-anim/issues/232) [#86](https://github.com/Rawframe-Project/maul-anim/issues/86)
* **format:** refuse a later JSON form with mani_errorVersion ([#229](https://github.com/Rawframe-Project/maul-anim/issues/229)) ([36280d4](https://github.com/Rawframe-Project/maul-anim/commit/36280d4b314fde3a5fe6a86e342ee28c4c587e13)), closes [#228](https://github.com/Rawframe-Project/maul-anim/issues/228) [#86](https://github.com/Rawframe-Project/maul-anim/issues/86)
* **format:** refuse unknown tags, loose sections and nonzero padding ([#218](https://github.com/Rawframe-Project/maul-anim/issues/218)) ([e38551b](https://github.com/Rawframe-Project/maul-anim/commit/e38551b90721f41ab148d8c164adbc5e70deee21)), closes [#217](https://github.com/Rawframe-Project/maul-anim/issues/217) [#86](https://github.com/Rawframe-Project/maul-anim/issues/86)
* **graph:** leave keys at their defaults out of the JSON form ([#231](https://github.com/Rawframe-Project/maul-anim/issues/231)) ([61fc16c](https://github.com/Rawframe-Project/maul-anim/commit/61fc16ceb6e8ba7c5be0984130abeac559b3d98d)), closes [#230](https://github.com/Rawframe-Project/maul-anim/issues/230) [#86](https://github.com/Rawframe-Project/maul-anim/issues/86)
* **graph:** make the slot poses' headers on every evaluation ([#262](https://github.com/Rawframe-Project/maul-anim/issues/262)) ([34ff006](https://github.com/Rawframe-Project/maul-anim/commit/34ff006be47d73905bf4ce7592c80b4a127684ee)), closes [#261](https://github.com/Rawframe-Project/maul-anim/issues/261)
* **ik:** aim, CCD and FABRIK on vectors far from unit length ([#239](https://github.com/Rawframe-Project/maul-anim/issues/239)) ([c5efe40](https://github.com/Rawframe-Project/maul-anim/commit/c5efe405d72f6a40c838d6a1999f65e9dafcffcf)), closes [#236](https://github.com/Rawframe-Project/maul-anim/issues/236)
* **ik:** carry the aim forward and up by the bone's whole model frame ([#242](https://github.com/Rawframe-Project/maul-anim/issues/242)) ([942bfb2](https://github.com/Rawframe-Project/maul-anim/commit/942bfb2191e821dcabd37b8c0679ca4cad176772)), closes [#241](https://github.com/Rawframe-Project/maul-anim/issues/241)
* **ik:** carry the aim offset by the bone's whole model frame ([#240](https://github.com/Rawframe-Project/maul-anim/issues/240)) ([ff3968f](https://github.com/Rawframe-Project/maul-anim/commit/ff3968fb41d16d30feb8bfc0be40e06b42f196f3)), closes [#238](https://github.com/Rawframe-Project/maul-anim/issues/238)
* **ik:** solve a body far from unit size ([#243](https://github.com/Rawframe-Project/maul-anim/issues/243)) ([78254ca](https://github.com/Rawframe-Project/maul-anim/commit/78254ca6f5078eccb404c26ebce961c2eee09af8)), closes [#237](https://github.com/Rawframe-Project/maul-anim/issues/237)
* **ik:** solve a two-bone chain the same at any finite size ([#235](https://github.com/Rawframe-Project/maul-anim/issues/235)) ([eb2ab4c](https://github.com/Rawframe-Project/maul-anim/commit/eb2ab4c13f232c2f60f1a93c7ac777157d15c88d)), closes [#234](https://github.com/Rawframe-Project/maul-anim/issues/234)
* **learned:** store the weight format as an optional section ([#227](https://github.com/Rawframe-Project/maul-anim/issues/227)) ([004280d](https://github.com/Rawframe-Project/maul-anim/commit/004280dbac32f9d5a59c546c33dae4f9e8868b26)), closes [#226](https://github.com/Rawframe-Project/maul-anim/issues/226) [#86](https://github.com/Rawframe-Project/maul-anim/issues/86)
* make MANI_NODISCARD the attribute under GCC 14's C23 too ([#256](https://github.com/Rawframe-Project/maul-anim/issues/256)) ([6e810d5](https://github.com/Rawframe-Project/maul-anim/commit/6e810d5975613e03b8bda095faa2f0f22741149d)), closes [#255](https://github.com/Rawframe-Project/maul-anim/issues/255)
* mark every internal bool function nodiscard and check it ([#259](https://github.com/Rawframe-Project/maul-anim/issues/259)) ([dcdc074](https://github.com/Rawframe-Project/maul-anim/commit/dcdc074e432a9061edb8ee0e7de17c97fcf768ad)), closes [#258](https://github.com/Rawframe-Project/maul-anim/issues/258)
* **match:** store the leaf size as an optional section ([#225](https://github.com/Rawframe-Project/maul-anim/issues/225)) ([1f4aab5](https://github.com/Rawframe-Project/maul-anim/commit/1f4aab5aaf5f6e59293737b112f54973e0f6a828)), closes [#224](https://github.com/Rawframe-Project/maul-anim/issues/224) [#86](https://github.com/Rawframe-Project/maul-anim/issues/86)
* **morph:** bound what the morph calls read from the file's bytes ([#251](https://github.com/Rawframe-Project/maul-anim/issues/251)) ([f576392](https://github.com/Rawframe-Project/maul-anim/commit/f576392bed762847e915ae772d2b4e93ef8f9ddf)), closes [#250](https://github.com/Rawframe-Project/maul-anim/issues/250)
* **retarget:** store the pelvis block as an optional section ([#223](https://github.com/Rawframe-Project/maul-anim/issues/223)) ([ae087b5](https://github.com/Rawframe-Project/maul-anim/commit/ae087b5d40eac3ff28e6c2caa7e06d58af54f76b)), closes [#222](https://github.com/Rawframe-Project/maul-anim/issues/222) [#86](https://github.com/Rawframe-Project/maul-anim/issues/86)
* **skeleton:** check the skeleton view on every call ([#248](https://github.com/Rawframe-Project/maul-anim/issues/248)) ([90b144e](https://github.com/Rawframe-Project/maul-anim/commit/90b144e5380cf6d64fa38df70cb182d10d54f5ea))
* **skin:** check the skin, influences and morph views on every call ([#252](https://github.com/Rawframe-Project/maul-anim/issues/252)) ([501b8d8](https://github.com/Rawframe-Project/maul-anim/commit/501b8d8b0471d22fed976114cd4c7d722a415c3f))


### Changed

* **clip:** form the sampler's sections and counts once a call ([#254](https://github.com/Rawframe-Project/maul-anim/issues/254)) ([203e8d9](https://github.com/Rawframe-Project/maul-anim/commit/203e8d9fa9f4dc61fbebbadcd98c839998eaa1c4))

## [0.9.0](https://github.com/Rawframe-Project/maul-anim/compare/v0.8.0...v0.9.0) (2026-11-02)

### Changed

- **Breaking: `maniSkeleton_BlendPoses` refuses a threshold below
  `FLT_MIN`.** The threshold must be at least `FLT_MIN` (2^-126) and at
  most 1. A subnormal threshold, which 0.8.0 accepted and which could
  blend translations and scales to infinity or NaN, now returns
  `mani_errorInvalid` and leaves the output pose as it was. Graphs blend
  at `MANI_BLEND_THRESHOLD` (1e-5) and are not affected
  ([#216](https://github.com/Rawframe-Project/maul-anim/pull/216)).

### Fixed

- Sampling a clip between two rotation samples in opposite hemispheres
  could give the identity rotation. A clip file may hold such samples,
  for example (1, 0, 0, 2^-80) and (-1, 0, 2^-80, 0); halfway between
  them, the interpolated rotation is too short for its squared length to
  be a normal binary32. It is now scaled by a power of two before it is
  normalized, so it keeps its direction. The builder keeps consecutive
  samples in one hemisphere, so built clips do not reach this case
  ([#211](https://github.com/Rawframe-Project/maul-anim/pull/211)).
- The skinning jobs gave a zero normal or tangent for a direction that
  had not collapsed when the palette's scale was far from 1: a uniform
  scale of about 2^-40 or 2^40 for normals, and 2^-70 or 2^70 for
  tangents. They now keep its direction. Only a zero direction, or one
  that is not finite, still comes out zero
  ([#214](https://github.com/Rawframe-Project/maul-anim/pull/214)).

Results change only for the inputs above, which gave the identity or a
zero direction before. Every other pose, sample, skinned vertex, file
byte and format version is the same, and no pinned hash moves. As with
every release before 1.0.0, a graph instance, spring state or trainer
checkpoint saved by 0.8.0 is refused with `mani_errorVersion`: keep
assets, not saved states, across the upgrade.

## [0.8.0](https://github.com/Rawframe-Project/maul-anim/compare/v0.7.0...v0.8.0) (2026-10-31)

### Changed

- **Breaking: kinds, modes and choices are `int32_t`, and flag sets have
  `uint32_t` typedefs.** All 33 kind, mode and choice typedefs, such as
  `maniLoopPolicy`, are `int32_t`; a negative value is refused like one
  past the last value. New flag-set typedefs are `maniGraphSyncFlags`,
  `maniGraphClipFlags`, `maniRootAxes` and `maniMatchEntryFlags`, and the
  fields that hold them take them, as does `maniClipSource.loop`, now a
  `maniLoopPolicy`. Rename `MANI_MATCH_LOOP`, `MANI_MATCH_NO_JUMP` and
  `MANI_MATCH_MIRRORED` to `maniMatchEntryLoop`, `maniMatchEntryNoJump`
  and `maniMatchEntryMirrored`. `maniMorphTargets_GetAttribute` returns
  -1 past the count, where it returned `UINT32_MAX`. Code that keeps a
  kind in a `uint32_t` variable or compares it with an unsigned value
  updates its types
  ([#199](https://github.com/Rawframe-Project/maul-anim/pull/199)).
- **Breaking: two names follow the family's naming rules.**
  `maniLearnedTrainerReport.iteration` is `iterationCount`, and
  `maniRootMoveIdentity` is `maniMakeRootMoveIdentity`
  ([#198](https://github.com/Rawframe-Project/maul-anim/pull/198)).
- **Breaking: the CPU checks return `bool`, and `maniHash64` takes an
  `int64_t` count.** `maniIsCpuBackendSupported` and `maniCheckCpu`
  return `bool` instead of an `int32_t` 1 or 0, so a caller comparing
  the result with 1 tests it as a `bool`. `maniHash64` hashes any byte
  count up to `INT64_MAX`; pass the count without narrowing it. Hash
  values are unchanged
  ([#176](https://github.com/Rawframe-Project/maul-anim/pull/176)).
- **Breaking: a refused call or a "no" clears its outputs.** A call on a
  builder, a glTF or FBX document or a trainer clears its sizes, counts,
  reports, structs and handles before any check, so a failed call never
  leaves an earlier call's value to be read as its answer.
  `maniBuilder_CopyBuilt`, `maniBuilder_CopySkeletonOrder`,
  `maniGltf_GetMorphWeights`, `maniFbx_GetMorphWeights` and
  `maniLearnedTrainer_SaveCheckpoint` also zero the caller's array up to
  its capacity; a short skeleton order is no longer filled as far as it
  goes. A too-small buffer still reports the size or count it needed.
  `maniBuildGltfClip` and `maniBuildFbxClip` write `boundOut` only when
  the clip builds. `maniClip_GetRootMotionSource` and
  `maniInfluences_GetCenter` clear their output when they return false,
  and the name getters of learned models and motion databases clear
  `lengthOut` when they return NULL. The headers say what each output
  holds after a failure
  ([#165](https://github.com/Rawframe-Project/maul-anim/pull/165),
  [#174](https://github.com/Rawframe-Project/maul-anim/pull/174)).
- **Breaking: a call refused as busy leaves its outputs.** A builder or
  trainer call that its own allocator or task hook made is refused with
  `mani_errorBusy` and writes none of its outputs, since they may be the
  ones the call in progress is filling. `maniOpenGltf` and
  `maniOpenFbx` still set their handle to NULL
  ([#173](https://github.com/Rawframe-Project/maul-anim/pull/173)).
- **Breaking: more calls back in from host code are refused as busy.**
  `maniGltf_SupplyBuffer`, `maniGltf_ReadSprings` and
  `maniGltf_ReadRagdoll` return `mani_errorBusy` when called from the
  builder's allocator. `maniLearnedTrainer_SaveCheckpoint` and
  `maniBuildLearnedModelFromTrainer` return it when called from the
  trainer's allocator or task hooks. While a model is built from a
  trainer, `maniLearnedTrainer_Train`,
  `maniLearnedTrainer_RestoreCheckpoint` and
  `maniDestroyLearnedTrainer` on that trainer return it too. Each
  refusal counts one misuse
  ([#187](https://github.com/Rawframe-Project/maul-anim/pull/187),
  [#197](https://github.com/Rawframe-Project/maul-anim/pull/197)).
- The headers say which outputs may be their input:
  `maniBlendRootMotion`'s `moveOut` may be one of `moves`,
  `maniLearnedModel_Project`'s `stateOut` may be `query`, and
  `maniMorphTargets_ComputeTargetWeights`' `targetWeightsOut` may be
  `channelWeights`
  ([#195](https://github.com/Rawframe-Project/maul-anim/pull/195)).

### Fixed

- `maniMorphTargets_ComputeTargetWeights` gave wrong weights when its
  output was its own channel weights. It now computes them in place with
  the same bits as apart
  ([#193](https://github.com/Rawframe-Project/maul-anim/pull/193)).
- `maniRetargetMap_RetargetPose` gave a wrong pose when the target pose
  was the source pose, or shared memory with it. It now refuses any
  overlap of the two poses with `mani_errorInvalid` and writes nothing
  ([#194](https://github.com/Rawframe-Project/maul-anim/pull/194)).
- Destroying a builder or a trainer, or closing a glTF or FBX document,
  let the host's free hook call back into the object: a hook could
  build on a builder being destroyed, free it twice, or read a closing
  document's freed names. Every call from those frees is now refused
  with `mani_errorBusy`, and a closing document reads as empty
  ([#187](https://github.com/Rawframe-Project/maul-anim/pull/187)).
- Host code running while a model was built from a trainer could
  destroy or retrain the trainer under the build, which then read freed
  memory. The trainer is now marked busy for the whole build
  ([#197](https://github.com/Rawframe-Project/maul-anim/pull/197)).

Poses, samples, file bytes and format versions do not change. As with
every release before 1.0.0, a graph instance, spring state or trainer
checkpoint saved by 0.7.0 is refused with `mani_errorVersion`: keep
assets, not saved states, across the upgrade.

## [0.7.0](https://github.com/Rawframe-Project/maul-anim/compare/v0.6.0...v0.7.0) (2026-10-29)

### Changed

- **Breaking: every call is named by the Maul family's naming rule.** A
  call on one object is named `maniObject_Verb`: a getter
  `maniObject_GetThing`, a predicate `maniObject_IsThing`. The object is
  the library object the call takes first, such as a clip, a pose, a
  skeleton, a graph instance, a builder or a glTF document, even where
  the call only reads it: `maniSampleClip` is `maniClip_Sample`,
  `maniClipDuration` is `maniClip_GetDuration`, `maniPoseIsDifference`
  is `maniPose_IsDifference` and `maniBlendPoses` is
  `maniSkeleton_BlendPoses`. Creation, destruction and calls with no
  object keep a verb first: every `maniOpen...`, `maniInit...`,
  `maniCreate...` and `maniBuild...` keeps its name, and the importers'
  builds take that shape (`maniGltfBuildClip` is `maniBuildGltfClip`).
  The misuse counts are `maniBuilder_GetMisuse` and
  `maniLearnedTrainer_GetMisuse`, `maniResultName` is
  `maniGetResultName`, the CPU check is `maniIsCpuBackendSupported`, and
  the task hook types end in `Function`. Math on plain values keeps its
  name (`maniRootMoveIdentity`, `maniComposeRootMotion`,
  `maniDecomposeAffine`, `maniHash64`). Rename every call in the table
  below; the parameters stay the same, except for `maniPose_AddLayers`
  ([#156](https://github.com/Rawframe-Project/maul-anim/pull/156)).
- **Breaking: `maniPose_AddLayers` takes the pose first.** A pose a call
  changes in place is its object, so `maniAddPoses(skeleton, pose,
  layers, count)` is `maniPose_AddLayers(pose, skeleton, layers,
  count)`. Swap the first two arguments; a compiler refuses or warns
  about the old order, since the two pointers have different types. The
  IK solvers already took the pose first and keep their order as
  `maniPose_SolveTwoBone` and the others
  ([#156](https://github.com/Rawframe-Project/maul-anim/pull/156)).
- The user guides and the API reference use the new names
  ([#156](https://github.com/Rawframe-Project/maul-anim/pull/156)).

Poses, samples and file bytes do not change; only the names do. As
with every release before 1.0.0, a graph instance, spring state or
trainer checkpoint saved by 0.6.0 is refused with `mani_errorVersion`:
keep assets, not saved states, across the upgrade.

| Before | After |
|---|---|
| `maniResultName` | `maniGetResultName` |
| `maniTaskFn` | `maniTaskFunction` |
| `maniEnqueueTaskFn` | `maniEnqueueTaskFunction` |
| `maniFinishTaskFn` | `maniFinishTaskFunction` |
| `maniPoseDifference` | `maniSkeleton_ComputePoseDifference` |
| `maniBlendPoses` | `maniSkeleton_BlendPoses` |
| `maniBuildMask` | `maniSkeleton_BuildMask` |
| `maniAddPoses` | `maniPose_AddLayers` |
| `maniBodyIkScratchSize` | `maniSkeleton_GetBodyIkScratchSize` |
| `maniSolveBodyXpbd` | `maniPose_SolveBodyXpbd` |
| `maniSolveBodySdls` | `maniPose_SolveBodySdls` |
| `maniBuilderMisuses` | `maniBuilder_GetMisuse` |
| `maniMeasureClip` | `maniBuilder_MeasureClip` |
| `maniCopyBuilt` | `maniBuilder_CopyBuilt` |
| `maniCopySkeletonOrder` | `maniBuilder_CopySkeletonOrder` |
| `maniClipCompressed` | `maniClip_IsCompressed` |
| `maniClipDuration` | `maniClip_GetDuration` |
| `maniClipSampleRate` | `maniClip_GetSampleRate` |
| `maniClipSampleCount` | `maniClip_GetSampleCount` |
| `maniClipLoopPolicy` | `maniClip_GetLoopPolicy` |
| `maniClipAdditiveBasis` | `maniClip_GetAdditiveBasis` |
| `maniClipIdentity` | `maniClip_GetIdentity` |
| `maniClipSkeletonIdentity` | `maniClip_GetSkeletonIdentity` |
| `maniSampleClip` | `maniClip_Sample` |
| `maniClipFloatTrackCount` | `maniClip_GetFloatTrackCount` |
| `maniClipFloatTrackName` | `maniClip_GetFloatTrackName` |
| `maniSampleClipBones` | `maniClip_SampleBones` |
| `maniClipCursorSize` | `maniClip_GetCursorSize` |
| `maniClipCursorCompactSize` | `maniClip_GetCursorCompactSize` |
| `maniSampleClipCursor` | `maniClipCursor_Sample` |
| `maniSampleClipFloatsCursor` | `maniClipCursor_SampleFloats` |
| `maniClipFindFloatTrack` | `maniClip_FindFloatTrack` |
| `maniSampleClipFloats` | `maniClip_SampleFloats` |
| `maniClipDiscreteTrackCount` | `maniClip_GetDiscreteTrackCount` |
| `maniClipDiscreteTrackName` | `maniClip_GetDiscreteTrackName` |
| `maniClipFindDiscreteTrack` | `maniClip_FindDiscreteTrack` |
| `maniClipDiscreteValue` | `maniClip_GetDiscreteValue` |
| `maniClipEventCount` | `maniClip_GetEventCount` |
| `maniClipEventAt` | `maniClip_GetEvent` |
| `maniClipEventRange` | `maniClip_GetEventRange` |
| `maniClipMarkerCount` | `maniClip_GetMarkerCount` |
| `maniClipMarkerAt` | `maniClip_GetMarker` |
| `maniGraphIdentity` | `maniGraph_GetIdentity` |
| `maniGraphSkeletonIdentity` | `maniGraph_GetSkeletonIdentity` |
| `maniGraphNodeCount` | `maniGraph_GetNodeCount` |
| `maniGraphParameterCount` | `maniGraph_GetParameterCount` |
| `maniGraphFindParameter` | `maniGraph_FindParameter` |
| `maniGraphParameterTypeOf` | `maniGraph_GetParameterType` |
| `maniGraphParameterName` | `maniGraph_GetParameterName` |
| `maniGraphClipCount` | `maniGraph_GetClipCount` |
| `maniGraphClipIdentity` | `maniGraph_GetClipIdentity` |
| `maniGraphClipName` | `maniGraph_GetClipName` |
| `maniGraphDatabaseCount` | `maniGraph_GetDatabaseCount` |
| `maniGraphDatabaseIdentity` | `maniGraph_GetDatabaseIdentity` |
| `maniGraphDatabaseName` | `maniGraph_GetDatabaseName` |
| `maniGraphModelCount` | `maniGraph_GetModelCount` |
| `maniGraphModelIdentity` | `maniGraph_GetModelIdentity` |
| `maniGraphModelName` | `maniGraph_GetModelName` |
| `maniGraphSpringSetCount` | `maniGraph_GetSpringSetCount` |
| `maniGraphSpringSetIdentity` | `maniGraph_GetSpringSetIdentity` |
| `maniGraphSpringSetName` | `maniGraph_GetSpringSetName` |
| `maniGraphRetargetMapCount` | `maniGraph_GetRetargetMapCount` |
| `maniGraphRetargetMapIdentity` | `maniGraph_GetRetargetMapIdentity` |
| `maniGraphMirrorMapIdentity` | `maniGraph_GetMirrorMapIdentity` |
| `maniGraphRetargetMapSourceIdentity` | `maniGraph_GetRetargetMapSourceIdentity` |
| `maniGraphClipRetargetMap` | `maniGraph_GetClipRetargetMap` |
| `maniGraphNodeKindOf` | `maniGraph_GetNodeKind` |
| `maniGraphNodeInputCount` | `maniGraph_GetNodeInputCount` |
| `maniGraphNodeInput` | `maniGraph_GetNodeInput` |
| `maniGraphOutputNode` | `maniGraph_GetOutputNode` |
| `maniGraphNodeSourcePlace` | `maniGraph_GetNodeSourcePlace` |
| `maniGraphBindingSize` | `maniGraph_GetBindingSize` |
| `maniGraphInstanceSize` | `maniGraphBinding_GetInstanceSize` |
| `maniSetGraphParameter` | `maniGraphInstance_SetParameter` |
| `maniSetGraphPose` | `maniGraphInstance_SetPose` |
| `maniGetGraphPose` | `maniGraphInstance_GetPose` |
| `maniGetGraphParameter` | `maniGraphInstance_GetParameter` |
| `maniInspectGraphNode` | `maniGraphInstance_InspectNode` |
| `maniSaveGraphInstance` | `maniGraphInstance_Save` |
| `maniRestoreGraphInstance` | `maniGraphInstance_Restore` |
| `maniUpdateGraph` | `maniGraphInstance_Update` |
| `maniGraphScratchSize` | `maniGraphBinding_GetScratchSize` |
| `maniEvaluateGraph` | `maniGraphInstance_Evaluate` |
| `maniGraphSubsetSize` | `maniGraphBinding_GetSubsetSize` |
| `maniGraphSubsetBones` | `maniGraphSubset_GetBones` |
| `maniEvaluateGraphSubset` | `maniGraphInstance_EvaluateSubset` |
| `maniSolveTwoBone` | `maniPose_SolveTwoBone` |
| `maniSolveAim` | `maniPose_SolveAim` |
| `maniSolveCcd` | `maniPose_SolveCcd` |
| `maniSolveFabrik` | `maniPose_SolveFabrik` |
| `maniGltfBufferCount` | `maniGltf_GetBufferCount` |
| `maniGltfBufferUri` | `maniGltf_GetBufferUri` |
| `maniGltfSupplyBuffer` | `maniGltf_SupplyBuffer` |
| `maniGltfSkeletonCount` | `maniGltf_GetSkeletonCount` |
| `maniGltfSkinCount` | `maniGltf_GetSkinCount` |
| `maniGltfSkinSkeleton` | `maniGltf_GetSkinSkeleton` |
| `maniGltfNodeCount` | `maniGltf_GetNodeCount` |
| `maniGltfBuildSkeleton` | `maniBuildGltfSkeleton` |
| `maniGltfBuildNodeSkeleton` | `maniBuildGltfNodeSkeleton` |
| `maniGltfSkeletonPlacement` | `maniGltf_GetSkeletonPlacement` |
| `maniGltfBuildSkin` | `maniBuildGltfSkin` |
| `maniGltfNodeBoneName` | `maniGltf_GetNodeBoneName` |
| `maniGltfRenamedCount` | `maniGltf_GetRenamedCount` |
| `maniGltfRenamedNode` | `maniGltf_GetRenamedNode` |
| `maniGltfAnimationCount` | `maniGltf_GetAnimationCount` |
| `maniGltfAnimationName` | `maniGltf_GetAnimationName` |
| `maniGltfBuildClip` | `maniBuildGltfClip` |
| `maniGltfReadSprings` | `maniGltf_ReadSprings` |
| `maniGltfReadRagdoll` | `maniGltf_ReadRagdoll` |
| `maniGltfMeshCount` | `maniGltf_GetMeshCount` |
| `maniGltfPrimitiveCount` | `maniGltf_GetPrimitiveCount` |
| `maniGltfPrimitiveTargetCount` | `maniGltf_GetPrimitiveTargetCount` |
| `maniGltfNodeMesh` | `maniGltf_GetNodeMesh` |
| `maniGltfNodeSkin` | `maniGltf_GetNodeSkin` |
| `maniGltfMorphWeights` | `maniGltf_GetMorphWeights` |
| `maniGltfBuildInfluences` | `maniBuildGltfInfluences` |
| `maniGltfBuildMorphTargets` | `maniBuildGltfMorphTargets` |
| `maniFbxVersion` | `maniFbx_GetVersion` |
| `maniFbxModelCount` | `maniFbx_GetModelCount` |
| `maniFbxModelBoneName` | `maniFbx_GetModelBoneName` |
| `maniFbxRenamedCount` | `maniFbx_GetRenamedCount` |
| `maniFbxRenamedModel` | `maniFbx_GetRenamedModel` |
| `maniFbxSkeletonCount` | `maniFbx_GetSkeletonCount` |
| `maniFbxSkinCount` | `maniFbx_GetSkinCount` |
| `maniFbxSkinSkeleton` | `maniFbx_GetSkinSkeleton` |
| `maniFbxSkinModel` | `maniFbx_GetSkinModel` |
| `maniFbxReadSettings` | `maniFbx_ReadSettings` |
| `maniFbxBuildSkeleton` | `maniBuildFbxSkeleton` |
| `maniFbxBuildModelSkeleton` | `maniBuildFbxModelSkeleton` |
| `maniFbxSkeletonPlacement` | `maniFbx_GetSkeletonPlacement` |
| `maniFbxBuildSkin` | `maniBuildFbxSkin` |
| `maniFbxSkinMethod` | `maniFbx_GetSkinMethod` |
| `maniFbxBuildInfluences` | `maniBuildFbxInfluences` |
| `maniFbxMorphCount` | `maniFbx_GetMorphCount` |
| `maniFbxMorphModel` | `maniFbx_GetMorphModel` |
| `maniFbxMorphTargetCount` | `maniFbx_GetMorphTargetCount` |
| `maniFbxBuildMorphTargets` | `maniBuildFbxMorphTargets` |
| `maniFbxMorphWeights` | `maniFbx_GetMorphWeights` |
| `maniFbxClipCount` | `maniFbx_GetClipCount` |
| `maniFbxClipName` | `maniFbx_GetClipName` |
| `maniFbxBuildClip` | `maniBuildFbxClip` |
| `maniFbxModelPlacement` | `maniFbx_GetModelPlacement` |
| `maniLearnedModelIdentity` | `maniLearnedModel_GetIdentity` |
| `maniLearnedModelSkeletonIdentity` | `maniLearnedModel_GetSkeletonIdentity` |
| `maniLearnedModelDatabaseIdentity` | `maniLearnedModel_GetDatabaseIdentity` |
| `maniLearnedModelChannel` | `maniLearnedModel_GetChannel` |
| `maniLearnedModelNormalization` | `maniLearnedModel_GetNormalization` |
| `maniLearnedModelProjector` | `maniLearnedModel_GetProjector` |
| `maniLearnedModelFloatTrackCount` | `maniLearnedModel_GetFloatTrackCount` |
| `maniLearnedModelEventCount` | `maniLearnedModel_GetEventCount` |
| `maniLearnedModelFloatTrackName` | `maniLearnedModel_GetFloatTrackName` |
| `maniLearnedModelEventName` | `maniLearnedModel_GetEventName` |
| `maniLearnedScratchSize` | `maniLearnedModel_GetScratchSize` |
| `maniLearnedProject` | `maniLearnedModel_Project` |
| `maniLearnedStep` | `maniLearnedModel_Step` |
| `maniLearnedDecompress` | `maniLearnedModel_Decompress` |
| `maniLearnedTrainerMisuses` | `maniLearnedTrainer_GetMisuse` |
| `maniTrainLearned` | `maniLearnedTrainer_Train` |
| `maniSaveLearnedCheckpoint` | `maniLearnedTrainer_SaveCheckpoint` |
| `maniRestoreLearnedCheckpoint` | `maniLearnedTrainer_RestoreCheckpoint` |
| `maniMotionDatabaseIdentity` | `maniMotionDatabase_GetIdentity` |
| `maniMotionDatabaseMirrorMapIdentity` | `maniMotionDatabase_GetMirrorMapIdentity` |
| `maniMotionDatabaseSkeletonIdentity` | `maniMotionDatabase_GetSkeletonIdentity` |
| `maniMotionDatabaseClipIdentity` | `maniMotionDatabase_GetClipIdentity` |
| `maniMotionDatabaseChannel` | `maniMotionDatabase_GetChannel` |
| `maniMotionDatabaseChannelName` | `maniMotionDatabase_GetChannelName` |
| `maniMotionDatabaseEntry` | `maniMotionDatabase_GetEntry` |
| `maniMotionDatabaseFrame` | `maniMotionDatabase_GetFrame` |
| `maniMotionDatabaseFeatures` | `maniMotionDatabase_GetFeatures` |
| `maniMotionDatabaseNormalization` | `maniMotionDatabase_GetNormalization` |
| `maniSearchMotionDatabase` | `maniMotionDatabase_Search` |
| `maniMirrorMapIdentity` | `maniMirrorMap_GetIdentity` |
| `maniMirrorMapSkeletonIdentity` | `maniMirrorMap_GetSkeletonIdentity` |
| `maniMirrorMapAxis` | `maniMirrorMap_GetAxis` |
| `maniMirrorPartner` | `maniMirrorMap_GetPartner` |
| `maniMirrorBoneFlags` | `maniMirrorMap_GetBoneFlags` |
| `maniMirrorMapUnmatchedBone` | `maniMirrorMap_GetUnmatchedBone` |
| `maniMirrorName` | `maniMirrorMap_MirrorName` |
| `maniMirrorNameHash` | `maniMirrorMap_MirrorNameHash` |
| `maniMirrorPose` | `maniMirrorMap_MirrorPose` |
| `maniMirrorRootMotion` | `maniMirrorMap_MirrorRootMotion` |
| `maniMorphVertexCount` | `maniMorphTargets_GetVertexCount` |
| `maniMorphChannelCount` | `maniMorphTargets_GetChannelCount` |
| `maniMorphTargetCount` | `maniMorphTargets_GetTargetCount` |
| `maniMorphAttributeCount` | `maniMorphTargets_GetAttributeCount` |
| `maniMorphAttribute` | `maniMorphTargets_GetAttribute` |
| `maniMorphChannelName` | `maniMorphTargets_GetChannelName` |
| `maniMorphFindChannel` | `maniMorphTargets_FindChannel` |
| `maniMorphTargetName` | `maniMorphTargets_GetTargetName` |
| `maniMorphFindTarget` | `maniMorphTargets_FindTarget` |
| `maniMorphChannelTargets` | `maniMorphTargets_GetChannelTargets` |
| `maniMorphTargetFullWeight` | `maniMorphTargets_GetTargetFullWeight` |
| `maniMorphTargetWeights` | `maniMorphTargets_ComputeTargetWeights` |
| `maniPoseSize` | `maniSkeleton_GetPoseSize` |
| `maniPoseBoneCount` | `maniPose_GetBoneCount` |
| `maniPoseLocal` | `maniPose_GetLocal` |
| `maniPoseSetLocal` | `maniPose_SetLocal` |
| `maniPoseReadLocals` | `maniPose_ReadLocals` |
| `maniPoseWriteLocals` | `maniPose_WriteLocals` |
| `maniPoseSetRest` | `maniPose_SetRest` |
| `maniComputeModel` | `maniPose_ComputeModel` |
| `maniPoseModelCurrent` | `maniPose_IsModelCurrent` |
| `maniPoseIsDifference` | `maniPose_IsDifference` |
| `maniPoseModel` | `maniPose_GetModel` |
| `maniPoseReadModels` | `maniPose_ReadModels` |
| `maniPoseModelTransform` | `maniPose_GetModelTransform` |
| `maniRagdollIdentity` | `maniRagdoll_GetIdentity` |
| `maniRagdollSkeletonIdentity` | `maniRagdoll_GetSkeletonIdentity` |
| `maniRagdollBodyCount` | `maniRagdoll_GetBodyCount` |
| `maniRagdollReadBody` | `maniRagdoll_GetBody` |
| `maniRagdollBodiesFromPose` | `maniRagdoll_ComputeBodiesFromPose` |
| `maniRagdollJointTargets` | `maniRagdoll_ComputeJointTargets` |
| `maniRagdollPoseFromBodies` | `maniRagdoll_ComputePoseFromBodies` |
| `maniRagdollGetUpPlacement` | `maniRagdoll_ComputeGetUpPlacement` |
| `maniRetargetMapIdentity` | `maniRetargetMap_GetIdentity` |
| `maniRetargetMapSourceIdentity` | `maniRetargetMap_GetSourceIdentity` |
| `maniRetargetMapTargetIdentity` | `maniRetargetMap_GetTargetIdentity` |
| `maniRetargetScratchSize` | `maniRetargetMap_GetScratchSize` |
| `maniRetargetPose` | `maniRetargetMap_RetargetPose` |
| `maniRetargetRootMotion` | `maniRetargetMap_RetargetRootMotion` |
| `maniClipRootMotion` | `maniClip_GetRootMotion` |
| `maniRestoreRootMotion` | `maniClip_RestoreRootMotion` |
| `maniClipRootMotionSource` | `maniClip_GetRootMotionSource` |
| `maniSkeletonBoneCount` | `maniSkeleton_GetBoneCount` |
| `maniSkeletonIdentity` | `maniSkeleton_GetIdentity` |
| `maniSkeletonParent` | `maniSkeleton_GetParent` |
| `maniSkeletonSubtreeEnd` | `maniSkeleton_GetSubtreeEnd` |
| `maniSkeletonRest` | `maniSkeleton_GetRest` |
| `maniSkeletonBoneName` | `maniSkeleton_GetBoneName` |
| `maniSkeletonFindBone` | `maniSkeleton_FindBone` |
| `maniSkinJointCount` | `maniSkin_GetJointCount` |
| `maniSkinSkeletonIdentity` | `maniSkin_GetSkeletonIdentity` |
| `maniSkinIdentity` | `maniSkin_GetIdentity` |
| `maniSkinJoint` | `maniSkin_GetJoint` |
| `maniSkinInverseBind` | `maniSkin_GetInverseBind` |
| `maniComputePalette` | `maniPose_ComputePalette` |
| `maniInfluencesVertexCount` | `maniInfluences_GetVertexCount` |
| `maniInfluencesPerVertex` | `maniInfluences_GetPerVertex` |
| `maniInfluencesFitSkin` | `maniInfluences_IsSkinFit` |
| `maniInfluence` | `maniInfluences_GetInfluence` |
| `maniInfluencesDualQuaternionWeight` | `maniInfluences_GetDualQuaternionWeight` |
| `maniInfluencesCenter` | `maniInfluences_GetCenter` |
| `maniPaletteDualQuaternions` | `maniConvertPaletteToDualQuaternions` |
| `maniPaletteRows` | `maniConvertPaletteToRows` |
| `maniSpringSetIdentity` | `maniSpringSet_GetIdentity` |
| `maniSpringSetSkeletonIdentity` | `maniSpringSet_GetSkeletonIdentity` |
| `maniSpringSetSpringCount` | `maniSpringSet_GetSpringCount` |
| `maniSpringSetSpringName` | `maniSpringSet_GetSpringName` |
| `maniSpringSetStepRate` | `maniSpringSet_GetStepRate` |
| `maniSpringStateSize` | `maniSpringSet_GetStateSize` |
| `maniStepSprings` | `maniSpringState_Step` |
| `maniApplySprings` | `maniSpringState_Apply` |
| `maniResetSprings` | `maniSpringState_Reset` |
| `maniSaveSpringState` | `maniSpringState_Save` |
| `maniRestoreSpringState` | `maniSpringState_Restore` |
| `maniTrajectoryFeatures` | `maniMotionDatabase_ComputeTrajectoryFeatures` |
| `maniCpuSupportsBackend` | `maniIsCpuBackendSupported` |

## [0.6.0](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.6.0) (2026-10-29)

### Changed

- **Breaking: a saved state is read only by the release that wrote
  it.** A saved graph instance and a saved spring state start with the
  Maul family's 16-byte state prefix (magic, `MANI_ABI_VERSION`, format
  version, header size), so their headers are 32 bytes and every saved
  state's bytes change. `maniRestoreGraphInstance`,
  `maniRestoreSpringState` and `maniRestoreLearnedCheckpoint` refuse a
  state written by a release of another ABI version with
  `mani_errorVersion`: before 1.0.0 that is any other release, from
  1.0.0 another minor release. A state saved by 0.5.0 or earlier is
  refused. Keep assets, not saved states, across an upgrade. Poses,
  samples and file bytes do not change
  ([#134](https://github.com/Rawframe-Project/maul-anim/pull/134)).
- **Breaking: each save call reports the saved size.**
  `maniSaveGraphInstance`, `maniSaveSpringState` and
  `maniSaveLearnedCheckpoint` take a `sizeOut` after the capacity, and a
  NULL buffer with capacity 0 measures. `maniGraphStateSize`,
  `maniSpringSavedSize` and `maniLearnedCheckpointSize` are removed:
  call the save with a NULL buffer instead
  ([#129](https://github.com/Rawframe-Project/maul-anim/pull/129)).
- **Breaking: every call that fills an array takes its capacity and
  reports the count.** `maniMotionDatabaseFeatures`,
  `maniMotionDatabaseNormalization`, `maniLearnedModelNormalization`,
  `maniMorphTargetWeights` and `maniLearnedProject` take a capacity and
  a `countOut` after the array; `maniGltfMorphWeights` and
  `maniFbxMorphWeights` take a capacity in place of the exact count, and
  a `countOut`; `maniRagdollBodiesFromPose`, `maniRagdollJointTargets`
  and `maniBuildMask` take a `countOut` after the capacity.
  `maniLearnedOutput` gains `floatTrackCapacity` and `eventCapacity`
  after its arrays. A NULL array with capacity 0 measures
  ([#133](https://github.com/Rawframe-Project/maul-anim/pull/133)).
- **Breaking: name lengths are `size_t`.** Every name getter (the
  skeleton's bones, a clip's tracks, the graph's slots and parameters, a
  motion database's channels, a learned model's tracks and events, morph
  channels and targets, a spring set's springs, and the glTF and FBX
  importers' names) reports a length through `size_t*`, and every name
  length a public struct carries is `size_t`, so a name read from one
  call passes into a source unchanged. A builder refuses a name longer
  than the new `MANI_MAX_NAME_BYTES` (2^31 - 1 bytes) with
  `mani_errorLimit`, at the reason and item it gives that name
  otherwise; before, a morph channel name past it was refused with
  `mani_errorInvalid`, and a graph event name past it was hashed as if
  empty
  ([#135](https://github.com/Rawframe-Project/maul-anim/pull/135),
  [#138](https://github.com/Rawframe-Project/maul-anim/pull/138)).
- **Breaking: the graph's retargeting maps are named as such.**
  `maniGraphMapCount`, `maniGraphMapIdentity`,
  `maniGraphMapSourceIdentity` and `maniGraphClipMap` are
  `maniGraphRetargetMapCount`, `maniGraphRetargetMapIdentity`,
  `maniGraphRetargetMapSourceIdentity` and `maniGraphClipRetargetMap`;
  `MANI_MAX_GRAPH_MAPS` is `MANI_MAX_GRAPH_RETARGET_MAPS`;
  `maniGraphClipSource.map` is `retargetMap`, and `maniGraphAssets.maps`
  and `mapCount` are `retargetMaps` and `retargetMapCount`
  ([#136](https://github.com/Rawframe-Project/maul-anim/pull/136)).
- Every builder's reference names the `MANI_MAX_NAME_BYTES` refusal, and
  the getting-started guide gives the bound as an example of a fixed one
  ([#155](https://github.com/Rawframe-Project/maul-anim/pull/155)).

### Fixed

- **The motion database builder no longer writes past its names.**
  Channels may name one track, and their names' total was kept in 32
  bits: two channels naming one track of 2^31 + 1 bytes wrapped it, and
  copying the names wrote past the block it sized. Names past 4 GiB in
  all are refused with `mani_errorInvalid` and `maniBuildReasonCount`,
  before any name is read, and where `size_t` has 32 bits, sections past
  what one allocation holds are refused with `mani_errorCapacity`
  ([#143](https://github.com/Rawframe-Project/maul-anim/pull/143)).
- **The mirror map builder sizes its working memory in 64 bits.** Where
  `size_t` has 32 bits (WebAssembly), listed names totalling 4 GiB wrapped
  the room for their partners, which were then written past it; such a
  source is refused with `mani_errorCapacity`
  ([#142](https://github.com/Rawframe-Project/maul-anim/pull/142)).
- **The graph and mirror map builders refuse names past their files'
  32-bit offsets.** Names or partners past 4 GiB in all were laid out
  with wrapped offsets, so the file built pointed at other bytes; they
  are refused with `mani_errorInvalid` and `maniBuildReasonCount`, and
  the mirror map's sections past `size_t` with `mani_errorCapacity`
  ([#144](https://github.com/Rawframe-Project/maul-anim/pull/144),
  [#151](https://github.com/Rawframe-Project/maul-anim/pull/151)).
- **An imported name past 4 GiB is refused, not cut short.** The glTF
  and FBX importers cut a name's decoded length to 32 bits, handing the
  builder the first bytes of the real name; the whole name now reaches
  the builder, which refuses it with `mani_errorLimit`
  ([#153](https://github.com/Rawframe-Project/maul-anim/pull/153)).

## [0.5.0](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.5.0) (2026-10-28)

### Changed

- **Breaking: every file kind's layout and format version change, and
  an opener reads only its own version.** The reserved words are gone
  from the files: the container header is 48 bytes, not 64, and the
  clip, retargeting map, influences, learned model, learned checkpoint,
  spring set, ragdoll profile and mirror map lose their zero words.
  Every kind's format version goes up by one: skeleton 2, clip 4, graph
  7, retargeting map 3, skin 2, influences 2, morph targets 2, motion
  database 4, learned model 2, learned checkpoint 2, spring set 3,
  ragdoll profile 2, mirror map 3. An opener now refuses any format
  version but its own with `mani_errorVersion`, an earlier one included;
  it used to accept an earlier version and read it with the current
  layout. Build every stored file again from its source. Poses, samples
  and other outputs do not change; only hashes of file bytes move
  ([#113](https://github.com/Rawframe-Project/maul-anim/pull/113)).
- **Breaking: results take the Maul family's values.**
  `mani_errorVersion` is -7 (was -4) and `mani_errorFormat` is -8 (was
  -3). `mani_errorState` is removed: a call from an object's own host
  code (its allocator or task hooks) is refused with the new
  `mani_errorBusy` (-10) in its place, and still counts one misuse. Code
  that compares results by name only renames `mani_errorState` to
  `mani_errorBusy`
  ([#115](https://github.com/Rawframe-Project/maul-anim/pull/115)).
- **Breaking: a fixed bound passed returns `mani_errorLimit` (-9).** An
  argument, or a builder's or the trainer's source, past a bound the
  library publishes returns `mani_errorLimit` in place of
  `mani_errorInvalid`, with the same reason and item. This covers the
  aim, CCD, FABRIK and full-body IK solvers, the motion database search,
  and the skeleton, skin, influences, clip, morph targets, spring set,
  retargeting map, motion database, learned model, trainer and graph
  builders. A bound checked while opening a file is still
  `mani_errorFormat`, and a range of a parameter's meaning (a weight in
  [0, 1], a cone up to pi) is still `mani_errorInvalid`. Full-body IK
  scratch that is too small returns `mani_errorCapacity`, not
  `mani_errorInvalid`
  ([#117](https://github.com/Rawframe-Project/maul-anim/pull/117)).
- **Breaking: the bound constants are named `MANI_MAX_*`.**
  `MANI_AIM_MAX_BONES` is `MANI_MAX_AIM_BONES`, `MANI_IK_CHAIN_MAX_BONES`
  is `MANI_MAX_IK_CHAIN_BONES`, `MANI_IK_MAX_ITERATIONS` is
  `MANI_MAX_IK_ITERATIONS`, `MANI_SPRING_MAX_RATE` is
  `MANI_MAX_SPRING_RATE`, `MANI_MORPH_MAX_ATTRIBUTES` is
  `MANI_MAX_MORPH_ATTRIBUTES`, and the `MANI_BODY_IK_MAX_*`,
  `MANI_MATCH_MAX_*`, `MANI_LEARNED_MAX_*` and `MANI_GRAPH_MAX_*` names
  are `MANI_MAX_BODY_IK_*`, `MANI_MAX_MATCH_*`, `MANI_MAX_LEARNED_*` and
  `MANI_MAX_GRAPH_*`. A name that keeps the old form (`MANI_AIM_MAX_CONE`,
  `MANI_SPRING_MAX_STEPS`, `MANI_GRAPH_LEARNED_MAX_STEPS`) is a range or
  an internal cap, not a bound refused with `mani_errorLimit`
  ([#117](https://github.com/Rawframe-Project/maul-anim/pull/117)).
- **Breaking: public structs have no `reserved` field.** The field is
  removed from every public struct, and builders no longer refuse a
  source for a nonzero one. Remove it from initializers.
  `maniFileResult` is `{result, tag, offset}`: `tag` names the section
  the first wrong byte lies in, 0 in the header and table, so positional
  initializers take the tag second
  ([#109](https://github.com/Rawframe-Project/maul-anim/pull/109)).
- **Breaking: a root graph's slots are checked against the root's own
  lists.** `maniBuildGraph` refuses, with `maniBuildReasonNode` at the
  node's place, a root node that names a clip, sync group, database,
  model or spring set slot past the root's own list. In a graph with a
  subgraph such a node used to build with the subgraph's slot of that
  number: with 1 clip in the root and 1 in the subgraph, a root clip
  node naming slot 1 played the subgraph's clip. Only a host's own
  `maniGraphSource` structs reach this; the JSON form resolves each name
  within its own graph. Every other graph builds to the same bytes. To
  migrate, list in the root every asset and sync group its nodes name
  ([#93](https://github.com/Rawframe-Project/maul-anim/pull/93)).
- `MANI_MAX_INFLUENCES` (64 weighted influences on a vertex) and
  `MANI_MAX_GRAPH_DEPTH` (16 levels of subgraphs) are public
  ([#117](https://github.com/Rawframe-Project/maul-anim/pull/117)).
- On WebAssembly, the public calls no longer go through the
  floating-point environment's out-of-line entries, which have nothing
  to switch there, so the static library is about 14.7 KB smaller.
  Results do not change
  ([#101](https://github.com/Rawframe-Project/maul-anim/pull/101)).
- The user guide's getting started page explains `mani_errorLimit`,
  `mani_errorBusy` and the one format version a release reads, and its
  examples drop the `reserved` field and use the new constant names.

### Fixed

- The FBX importer bounds a file's property count. A file with 2^32 - 1
  properties or more wrapped the count, and the importer wrote past the
  property array it had allocated from it. Such a file is now refused
  at the property past the bound; every file under it reads as before
  ([#105](https://github.com/Rawframe-Project/maul-anim/pull/105)).
- A motion database file whose entry is too long for a frame count and
  stores `0xFFFFFFFF` as its count is refused at that entry's count. It
  was still refused, but at a later, wrong place. Valid databases open
  and build as before
  ([#107](https://github.com/Rawframe-Project/maul-anim/pull/107)).

## [0.4.0](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.4.0) (2026-10-25)

### Changed

- **Breaking: a subgraph's lists and slots are checked like the
  root's.** `maniBuildGraph` refuses with `maniBuildReasonNode`, at the
  node's place, a node inside a subgraph that names a clip, sync group,
  database, model or spring set slot past the subgraph's own list. Such
  a graph used to build when the root had a slot of that number, which
  the node then named: with 2 clips in the subgraph and 9 in the root,
  a clip node naming slot 3 played the root's clip 3. It also refuses
  with `maniBuildReasonCount` a subgraph with a list whose count is not
  zero and whose pointer is NULL; its clip, sync group, database, model
  and spring set lists used to be read through that pointer when a node
  named one of their slots. Only a host's own `maniGraphSource` structs
  reach either case; the JSON form resolves and checks its lists. Every
  other graph builds to the same bytes. To migrate, list in each
  subgraph every asset and sync group its nodes name
  ([#77](https://github.com/Rawframe-Project/maul-anim/pull/77)).
- On 32-bit x86 the library is built with SSE2 math
  (`-msse2 -mfpmath=sse`), so it needs a processor with SSE2, and every
  build now leaves trapping math at the compiler's default, from the
  family's compiler flags. 64-bit builds' tests and pinned hashes pass
  unchanged
  ([#73](https://github.com/Rawframe-Project/maul-anim/pull/73)).

### Fixed

- `maniBuildGraph` refuses a subgraph's sync group whose name is NULL
  with `maniBuildReasonCount` at item 0, before anything is laid out, as
  it refuses a subgraph's list that is not given. A root's own nameless
  group is still refused by the check with the sync group list's Name
  reason. Only a host's own `maniGraphSource` structs reach this; the
  JSON form always gives a name. Every graph that built before builds to
  the same bytes
  ([#81](https://github.com/Rawframe-Project/maul-anim/pull/81)).

## [0.3.0](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.3.0) (2026-10-25)

### Changed

- **Breaking: a graph's time scales are bounded.** On any path from the
  output, the product of the time scales' sizes must be at most
  `MANI_GRAPH_MAX_TIME_FACTOR` (1e6), which is far past any playback
  rate a graph needs. A scale that reads a parameter counts at the
  larger size of the parameter's bounds, and a time scale may read only
  a parameter with bounds. Scales on parallel branches never multiply.
  `maniBuildGraph` refuses a source past either rule with
  `maniBuildReasonScalar`, and `maniOpenGraph` refuses such a file with
  `mani_errorFormat`. Each node record now holds its time factor, so
  the graph format goes from 5 to 6, and `maniOpenGraph` refuses an
  earlier format with `mani_errorVersion`. To migrate, give bounds to
  each parameter a time scale reads, and build every graph file again.
  Results do not change for any graph still taken.
  ([#66](https://github.com/Rawframe-Project/maul-anim/pull/66))

### Fixed

- A clip node's step that is not finite moves nothing, and the marker
  walk's count is bounded by two loops of markers. Results do not
  change for steps shorter than about 2^53 clip lengths.
  ([#64](https://github.com/Rawframe-Project/maul-anim/pull/64))
- A graph's binary32 timers saturate at `FLT_MAX`: the fade time of a
  machine and of a one-shot, and an inertialize node's step and elapsed
  time. Every timer value below `FLT_MAX` gives the same bits as
  before.
  ([#67](https://github.com/Rawframe-Project/maul-anim/pull/67))

## [0.2.1](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.2.1) (2026-10-24)

### Changed

- The runtime's core (sampling, blending, the model pass, skeletons and
  root motion) is about 1.0 KB smaller at `-O2` with GCC: 68,562 bytes,
  against ozz-animation's 69,698. The skeleton's opener and the file
  checks are compiled for size under GCC and Clang. With GCC, opening a
  skeleton takes about 7% more instructions and opening a compressed
  clip about 9% more. Opening a full clip or a motion database does not
  change, and neither does any call made every frame. Results do not
  change.
  ([#29](https://github.com/Rawframe-Project/maul-anim/pull/29))

### Fixed

- The host's allocator and task hooks run in the host's own
  floating-point setting, like the rest of the host's code. In 0.2.0
  they ran in the library's setting while a public call was in
  progress.
  ([#27](https://github.com/Rawframe-Project/maul-anim/pull/27))

## [0.2.0](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.2.0) (2026-10-24)

### Changed

- **Breaking:** motion matching searches a motion database through an
  exact k-d tree, in place of bounds over runs of frames. Every search
  returns the same frame and cost as before. On large databases a search
  takes far fewer instructions: about 6 times fewer at 100,000 frames
  and up to 16 times fewer at 300,000 frames with the default leaf. On
  small databases with many features (around 5,000 frames and 45
  features) it can be slightly slower. The index takes about 49 bytes a
  frame at the default leaf, up from about 17.
  The motion database format version is 3, and
  `maniOpenMotionDatabase` refuses any other format version with
  `mani_errorVersion`. Build databases again from their sources, along
  with the graphs that name them, since a database's identity changes.
  A learned trainer's checkpoint does not resume against a rebuilt
  database.
  `maniMotionDatabaseSource` gains `leafFrames` (8, 16 or 32 frames a
  leaf, 0 for 16) and a reserved word that must be 0, so positional
  initializers must name them.
  ([#24](https://github.com/Rawframe-Project/maul-anim/pull/24))

### Fixed

- A host that runs its threads with flush-to-zero, denormals-are-zero
  or another rounding mode now gets the same bits as every other host.
  Every public call that computes switches to the library's
  floating-point setting and restores the host's on return, and so do
  the learned trainer's tasks on the host's worker threads. Results in
  the default setting do not change.
  ([#26](https://github.com/Rawframe-Project/maul-anim/pull/26))

## [0.1.0](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.1.0) (2026-10-24)

### Changed

- Compressed clips keep each kind of sub-track together and decode
  several sub-tracks at once: sampling at a random time takes about 28%
  fewer instructions and a forward cursor about 15% fewer, and poses and
  float tracks are the same bits as before.
  The clip format version is 3, and `maniOpenClip` refuses a clip of
  any other format version, compressed or full precision, with
  `mani_errorVersion`. Build stored clips again from their sources, or
  compress them again from full-precision clips made with this release.

### Fixed

- A learned motion matching node whose model gives a value past the
  float range, or whose root velocity moves past it over the step, no
  longer passes it on: `maniUpdateGraph`, `maniEvaluateGraph` and
  `maniEvaluateGraphSubset` complete the call with valid, finite
  outputs and return `mani_errorFormat`, and `maniLearnedStep`,
  `maniLearnedProject` and `maniLearnedDecompress` return
  `mani_errorFormat` instead of success. `maniGraphNodeView`'s reserved
  word is now `failed`.

## [0.0.1](https://github.com/Rawframe-Project/maul-anim/releases/tag/v0.0.1) (2026-10-24)

The first release: skeletal animation from import to the skinned
vertex, with the same results on every platform, compiler and SIMD
backend, and no allocation at run time.

### Added

- Skeletons, poses in local and model space composed as glTF composes
  nodes, and the polar decomposition of model matrices.
- Clips on a uniform grid sampled at the same cost at any time, with
  float and discrete tracks, events and sync markers; a forward cursor
  with the same results, in a faster form and a compact one.
- Clip compression within an error measured on a shell around each
  bone, close to ACL's sizes at a third of its error.
- Blending with per-bone masks, pose differences and additive layers
  and clips, and root motion taken out at build and read as moves
  across any number of loops, forward or backward.
- Animation graphs: clip, blend, mask, additive, select, time scale
  and seek nodes; 1D, triangulated and banded blend spaces; state
  machines with conditions on parameters, time, a clip's phase, end or
  events; one-shots; inertialization and dead blending; sync groups by
  phase or markers; subgraphs; a JSON form; saved and restored
  instances; node inspection for debuggers; evaluation of a subset of
  bones.
- Motion matching: motion databases with weighted feature channels, an
  exact pruned search, a trajectory predictor and a graph node;
  learned motion matching with a model format, a graph node and an
  in-library trainer whose results are the same on every platform and
  with any number of threads.
- Inverse kinematics: two-bone, aim and look-at over a chain, CCD and
  FABRIK chains with cone, twist and hinge limits, full-body XPBD and
  SDLS solvers, a turn limit node, each also a graph node.
- Spring bones as `VRMC_springBone` 1.0 defines them, with its extended
  colliders and limits, the same motion at any frame rate.
- Ragdoll profiles mapping bones to a physics engine's bodies and
  joints, bodies and motor targets from poses, poses from bodies, and
  get-up placement; a pose input for graphs.
- Retargeting between skeletons with different names, chain lengths,
  rests and proportions, and mirroring on rigs whose sides do not have
  mirrored axes, for poses, baked clips, graphs and motion databases;
  JSON forms for both maps.
- Skinning: skins, palettes, linear, dual quaternion and blended
  skinning and centers of rotation over one influence format, as jobs
  over strided vertex streams; morph targets with in-betweens.
- glTF 2.0 import (JSON and GLB, `EXT_meshopt_compression`,
  `KHR_meshopt_compression`, `KHR_draco_mesh_compression`,
  `KHR_animation_pointer`, `VRMC_springBone` and the `OMI_physics_*`
  extensions) and FBX import (binary and ASCII, versions 3000 to 7700):
  skeletons, skins, influences, morph targets and clips.
- Validation of every file, JSON text and saved state as it is opened,
  in every build: bad data is refused with a status and the offset of
  the first wrong byte, and counts no misuse on the builder; a misuse
  count reports only calls a correct program never makes.
- A CPU check for the AVX2 build, a portable scalar build with the same
  results, a CMake package and a pkg-config module.
- Support for Linux on x86-64 and arm64, macOS, Windows with
  `clang-cl`, and WebAssembly.
