# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and every data
format.

## [Unreleased]

### Added

- Graph files (mani-0023): `maniBuildGraph` builds a graph against a
  skeleton and its clips (parameters, clip slots, sync groups, and
  clip, blend, mask, additive, select, time scale and seek nodes as a
  DAG), ordering the nodes, marking poses and differences and reusing
  pose slots; `maniOpenGraph` opens one in place with every record
  checked; parameters and clip slots are read by name and index.
- Graph bindings and instances (mani-0023): `maniBindGraph` binds a
  graph's clip slots to the host's clips by identity in caller memory;
  `maniInitGraphInstance` makes an instance in caller memory that a
  `memcpy` copies; `maniSetGraphParameter` and `maniGetGraphParameter`
  set (clamped, normalized, checked) and read parameters;
  `maniSaveGraphInstance` and `maniRestoreGraphInstance` save a state
  without pointers and restore it after checking it as external bytes.
- Graph updates (mani-0023): `maniUpdateGraph` shares weights down from
  the output, moves clip playheads by speed and time scales (seeks set
  them), lists the events clip nodes pass across any number of wraps
  (counted past the buffer), and blends the clips' root motion by the
  weights the output gives it.
- Graph evaluation (mani-0023): `maniEvaluateGraph` writes an
  instance's pose through a per-thread scratch
  (`maniGraphScratchSize`): every weighed node in order into its slot
  (clips through their cursors, blends, masks and additive layers by the
  blending functions, bit for bit), unweighed nodes skipped.
- Graph sync groups and subgraphs (mani-0023): clip nodes of a sync
  group follow a leader (the declared one, else the member weighed
  most) by normalized phase, only the leader's events firing unless the
  group asks for all, by the markers common to all members when they
  repeat and share one (names matched, a follower placed where the
  leader's names and fraction are); `maniGraphNodeSubgraph` inlines
  another graph source at build, its parameters read through a map,
  its clips joined by identity and its sync groups by name.
- Graph text form (mani-0023): `maniWriteGraphJson` writes a graph
  source as JSON once it builds, names in place of indices (bones of
  mask chains included), subgraphs nested, a line per item, numbers as
  the shortest text that reads back to the same binary32.
- `maniBuildGraphJson` builds a graph from its JSON form against the
  host's clips by slot name, members strict, off values optional; texts
  that are not JSON or not the form are refused with the byte offset
  (`maniBuildReasonJson`, `maniBuildReasonForm`, `maniGraphForm`).
- Blend spaces (mani-0023): `maniGraphNodeBlendSpace1D` weighs its
  inputs by where a position falls among their points on a line,
  clamped or wrapped by a period; `maniGraphNodeBlendSpaceBands` by
  gradient bands, cartesian or polar, among points in a plane; the
  position a vec2 parameter or two numbers (`maniGraphPosition`);
  points not finite, equal or past a period refused
  (`maniBuildReasonPoint`); both in the JSON form.
- Blend spaces by triangles (mani-0023):
  `maniGraphNodeBlendSpaceTriangles` weighs the corners of the triangle
  holding the position, or outside them the ends of the nearest edge;
  the triangles are the author's (`maniGraphTrianglesNodeSource`,
  checked exactly for orientation and overlap,
  `maniBuildReasonSpaceTriangle`) or the points' Delaunay triangulation,
  made with exact integer predicates so every platform builds the same
  file; in the JSON form as `blendSpaceTriangles`.
- State machines (mani-0023): `maniGraphNodeMachine` blends its states
  (any subgraph, machines included) by weights its transitions set:
  from a state or any state (`MANI_GRAPH_ANY_STATE`), in priority order
  (no two equal among a state's candidates), when all their conditions
  hold (bool, int and float comparisons, triggers consumed on firing,
  time in state); cross-faded linearly or by smoothstep over seconds, an
  interrupted fade going on from the weights it reached; interruption
  none, by a lower priority value or by any; cooldowns; instant
  transitions chained to a limit; entry from the start or where the
  state was left. Machine state is saved and restored with the
  instance; refusals name the transition (`maniBuildReasonTransition`);
  in the JSON form as `machine`.
- Root motion taken out at build (mani-0022): a clip source may name a
  bone (`maniRootMotionSource`), whose translation along chosen axes
  and rotation (whole, or the twist about an axis) from its rest, its
  first sample or the origin are kept beside the clip and taken out of
  the bone, optionally looped so the motion ends where it began;
  `maniClipRootMotionSource` reads the source back; compressed clips
  keep it.
- Root motion moves (mani-0022): `maniClipRootMotion` gives the move
  from a playhead time over a signed step, clamped or across any number
  of wraps either way (the period's move composed by squaring);
  `maniComposeRootMotion`, `maniBlendRootMotion` and
  `maniRootMoveIdentity` compose and blend moves.
- `maniRestoreRootMotion` puts a clip's root motion back onto its bone
  in a sampled pose (mani-0022).
- Imported clips may carry root motion and an additive basis
  (`maniImportClipOptions`, mani-0022).
- The library skeleton: the build, the family rules and tools, the
  version and result API (`maniGetVersion`, `maniResultName`) and the
  library profile.
- `maniHash64` and `MANI_HASH_INIT`, the family's frozen 64-bit hash, for
  comparing results between machines.
- Compressed clips (mani-0019): `maniCompressClip` writes a clip of
  default, constant and animated sub-tracks, segments, range reduction
  and per-segment bit rates chosen by a shell error metric; `maniOpenClip`
  and every sampler take either form, `maniSampleClipBones` samples a
  subset, and a forward cursor (`maniClipCursorSize`,
  `maniInitClipCursor`, `maniSampleClipCursor`,
  `maniSampleClipFloatsCursor`) gives random access's bits. An ACL
  differential compares sizes, errors and playback costs.
- Blending (mani-0020): `maniBlendPoses` mixes poses by layer and
  per-bone weights with the rest filling in under a threshold,
  `maniBuildMask` makes per-bone weights from chains, and
  `maniPoseIsDifference` tells a difference from a pose of the skeleton.
- Additive layers (mani-0020): `maniPoseDifference` writes a pose's
  difference from a basis and `maniAddPoses` applies differences at
  layer and per-bone weights.
- Additive clips (mani-0020): `maniClipSource.additive` names a basis
  (the rest or the first frame), sampling such a clip writes a
  difference, compression measures it on the rest, and
  `maniClipAdditiveBasis` reports the basis.
- The ozz differential (mani-0020): `tools/ozz_differential.cc` blends
  the Quaternius clips' poses with masks and additive layers by
  ozz-animation's BlendingJob and by Maul Anim, compares them and times
  both, in the weekly differential workflow.
- Blending weights run from 0 to 1 and are divided by each bone's total
  first, so blends of finite poses stay finite; differences that are
  not finite are refused, and the palette refuses a difference.
- Influences (mani-0021): `maniBuildInfluences` writes a mesh
  primitive's influences bound to a skin, sorted and renormalized in
  groups of four; `maniOpenInfluences` and its queries read them, and
  `maniSkinIdentity` gives the identity they bind to.
- Linear blend skinning (mani-0021): `maniSkinLinear` skins vertex
  ranges of strided position, normal, tangent and further streams, in
  place or not; `maniPaletteRows` writes palettes as row-major 3x4 for
  shaders.
- Dual quaternion skinning (mani-0021): `maniPaletteDualQuaternions`
  splits each palette matrix as rotation times scale and shear,
  `maniSkinDualQuaternion` skins in two phases, and `maniSkinBlended`
  mixes linear and dual quaternion skinning at FBX's per-vertex weights.
- Skinning with optimized centers of rotation (mani-0021):
  `maniBuildCenters` computes each vertex's center offline from the
  rest mesh (Le and Hodgins 2016, Eq. 4, triangles indexed by joint
  pair), and `maniSkinCenters` skins about them without the collapse of
  linear or the bulge of dual quaternion skinning.
- Morph targets (mani-0021): `maniBuildMorphTargets` builds named
  channels of dense or sparse targets (FBX in-betweens included) over
  positions, normals, tangents, texture coordinates and colors,
  `maniOpenMorphTargets` opens them, `maniMorphTargetWeights` turns
  channel weights into target weights, and `maniApplyMorphTargets` adds
  the weighted deltas to strided vertex ranges before skinning;
  `maniMorphAttributeCount` and `maniMorphAttribute` read the table.
- glTF meshes (mani-0021): `maniGltfBuildInfluences` reads a
  primitive's joint and weight sets into influences, refusing what
  glTF forbids (new import reason `maniImportReasonInfluence`);
  `maniGltfBuildMorphTargets` reads its morph targets; `maniGltfMeshCount`,
  `maniGltfPrimitiveCount`, `maniGltfPrimitiveTargetCount`,
  `maniGltfNodeMesh`, `maniGltfNodeSkin` and `maniGltfMorphWeights`
  describe meshes and their starting weights. A mesh's target count is
  now its first primitive with targets', as glTF defines it.
- Morph targets are named (mani-0021): `maniMorphTargetName` and
  `maniMorphFindTarget`; a target without a given name takes its
  channel's, with '/' and its index past the first. Clip weight tracks bind to
  targets by these names.
- FBX influences (mani-0021): `maniFbxBuildInfluences` reads a skin's
  clusters into influences a control point at a time, as ufbx reads
  them; `maniFbxSkinMethod` names the job to run (`maniSkinningMethod`:
  linear, dual quaternion or blended, from SkinningType and the
  per-vertex blend weights).
- FBX blend shapes as morph targets (mani-0021):
  `maniFbxBuildMorphTargets` builds a BlendShape deformer's channels (or
  a 6.x mesh's Shape children) as ufbx reads them, in-betweens and
  negative targets sorted, Blender's per-vertex weights folded in, each
  target named as its clip weight track (a repeated channel name
  numbered); `maniFbxMorphCount`,
  `maniFbxMorphModel`, `maniFbxMorphTargetCount` and
  `maniFbxMorphWeights` describe the sets and their rest weights.

### Changed

- `maniGltfBuildClip` and `maniFbxBuildClip` take a
  `maniImportClipOptions` (`maniDefaultImportClipOptions`) in place of
  their sample rate and loop arguments; `maniGltfBuildClip` now refuses
  a rate not finite or negative as a misuse.
