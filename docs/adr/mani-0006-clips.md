# mani-0006. Clips on one uniform grid

Status: Accepted

## Context

Clips come from glTF and FBX as keys at arbitrary times with step,
linear or cubic spline interpolation, per channel. Playback must cost
the same forward, backward and at random (blending, seeking and phase
sync all jump), and give the same bits everywhere (mani-0001).

## Decision

- **Building** (`maniBuildClip`) takes tracks for some of a skeleton's
  bones, each with translation, rotation and scale channels of keys
  and glTF's interpolation, and evaluates every channel at each sample
  time exactly as glTF's Appendix C defines it, in binary64 with the
  library's own trigonometry, rounding once. Rotations are normalized
  and each sample put in the hemisphere of the one before. Keys out of
  order, values not finite, zero rotations, one-key splines and bones
  outside the skeleton are refused with the track and a reason.
- **One grid:** samples at each multiple of 1 / rate (30 per second by
  default), the last at the duration (the last key's time by
  default), at most `MANI_MAX_SAMPLES`.
- **The file** (container kind 2) binds the clip to its skeleton by
  identity and bone count and stores the samples sample-major at full
  precision, each sample every track's animated channels in bone
  order. Opening checks every field, every sample finite with unit
  rotations, and the identity, which covers every section.
- **Sampling** (`maniSampleClip`) clamps the time to [0, duration],
  finds the two samples and the factor between them by division,
  interpolates translation and scale linearly and rotations by
  normalized lerp, and writes every bone of a pose: the clip's channels
  where it has them, the rest elsewhere.

## Consequences

Any time costs one division and two contiguous reads, the same in any
order; a clip never plays on the wrong skeleton; and the full-precision
file is the reference the compressed format is measured against. A
step key lands on the first sample at or after it, and between samples
playback interpolates, so a step is spread over one sample period; the
rate is the host's lever.
