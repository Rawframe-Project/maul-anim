# mani-0022. Root motion

Status: Accepted

## Context

A clip that walks moves its root bone through space. Played in place
it slides; the host wants to move the character itself by the clip's
motion, frame by frame, across loops and backward play, blended as the
poses are, and some instances want the motion left in the pose
(mani-0001: the library holds no world position, and reports root
motion as a delta in the character's frame). The rivals stumble on the
same points: wraps walked one at a time, a backward step that stalls,
rotations masked by Euler components, weighted partial rotations that
do not add up, and a misnamed source that silently moves nothing.

## Decision

- **Source:** a clip may name one bone; its root motion is the bone's
  translation along chosen axes and its rotation (none, the twist about
  an axis by swing-twist, or the whole rotation), each from a
  reference (the bone's rest, the clip's first sample, or the origin),
  measured in the bone's parent's frame (the model space for a root
  bone), so taking it out stays a transform of translation, rotation
  and scale. A loop option spreads the end's difference from the start
  over the clip. A bone outside the skeleton or without a track, no
  channel chosen, a field out of range, an additive clip, or a motion
  translation beyond 2^64 is refused.
- **Taken out at build:** at every sample, in binary64, the motion `M`
  and the bone's stored transform `M^-1 L` (translation
  `m_r^-1 (t - m_t)`, rotation `m_r^-1 r`), so the clip plays in place
  and `M` times the stored bone is the source.
- **Stored** as section 16 of the clip file, in either form: the
  source, the whole period's move (from the first sample to the last,
  in the first's frame), and per sample the motion's translation and
  rotation in binary32. The compressor carries it unchanged. Files
  without root motion are unchanged, their identity too. Opening checks
  every float finite and every translation component within 2^64, so a
  move across 2^32 wraps stays finite in binary32.
- **Moves:** a move is a translation and a unit rotation in the
  character's frame where it begins. From a playhead time over a signed
  step: a clamped clip gives the move between the two times, each
  clamped to the clip; a repeating clip the move between the unwrapped
  times, the partial periods at either end between samples (translation
  lerped, rotation nlerped) and the whole periods between as the
  period's move (or its inverse, backward) composed by squaring,
  bounded at 2^32 periods. So a step split anywhere composes to the
  whole step, and backward undoes forward.
- **Composed and blended** as transforms without scale: `a` then `b`
  is `(a_t + a_r b_t, a_r b_r)`; a blend takes the weighted mean of the
  translations and the normalized weighted sum of the rotations, each
  turned into the hemisphere of the first weighted one, as poses blend
  (weights of 0 count for nothing, all 0 give no move).

- **Restored** into a pose (`maniRestoreRootMotion`) for a host that
  keeps root motion in the pose: the bone of a pose sampled at a time
  becomes the motion at that time composed with it, scale kept; a
  difference pose is refused.
- **Imported:** `maniGltfBuildClip` and `maniFbxBuildClip` take a
  per-clip `maniImportClipOptions` (sample rate, loop, additive basis,
  root motion source) in place of their rate and loop arguments, as
  Unity and Unreal set these per clip; the source names a bone of the
  opened skeleton, axes in the output's basis, and the clip builder
  takes the motion out. A source out of range is a misuse; one the
  builder refuses (a bone the animation leaves without a track) is
  reported as Build with the build reason as item.

## Consequences

A host moves its character by moves it reads beside the clip, with no
extraction at runtime. Masking by axes and a twist about an axis
covers the common choices (horizontal travel, turning about up)
without an Euler order.

Moves across any number of wraps cost the logarithm of the wrap count
and never stall; a step split anywhere composes to the whole step, so
a host stepping by frames travels as far as one stepping once. Blending
weighs whole moves once, so split steps do not lose rotation. A clip
keeps one root motion source; a host wanting another rebuilds the clip.
Between samples, a restored pose differs from the source by the
difference between interpolating the motion and the stored bone apart
and interpolating their product, which vanishes at the samples.
Changing the importers' clip arguments into one struct broke their
callers once, before the first release.
