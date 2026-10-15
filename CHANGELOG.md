# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and every data
format.

## [Unreleased]

### Added

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
