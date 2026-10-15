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
