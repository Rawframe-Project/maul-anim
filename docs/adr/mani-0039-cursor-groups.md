# mani-0039. The forward cursor's samples in the pose's groups

Status: Accepted

## Context

The forward cursor (mani-0019) holds the two decoded samples a time
falls between, so forward play decodes a sample only when the playhead
passes it. Each pose still walked every bone: read each channel's two
values from the held samples, constants and rests, interpolate, and
write the bone's slots, with the walk's branches on every channel of
every bone. Played forward on Quaternius's `ual.fbx` at ACL's default
precision, that took 25,777 instructions a pose under GCC 14 `-O3` and
AVX2 and 35,883 under Clang 20, against 16,001 and 15,681 for ACL's own
forward playback of the same clips.

Eight-lane playback was tried before, building lanes from the walk's
values each pose and scattering them back to the bones; the copies cost
more than the lanes saved. The pose already keeps its bones in groups of
eight (ten rows of eight floats a group), and the blend's kernels
(mani-0038) work on that layout.

## Decision

- **Held in groups.** On a vector backend the cursor also holds each of
  its two samples in the pose's group layout: stored channels at their
  values (constants read, rotations rebuilt), the others at the rest
  the walk writes, padding lanes the identity. A sample is laid out
  once, when the cursor decodes it. With them it keeps three rows of
  masks a group (translation, rotation, scale): set where the walk
  interpolates a channel, clear where it writes a rest.
- **One step a group.** A pose is then, group by group, the walk's
  arithmetic on eight lanes: `a + (b - a) * alpha` for vectors, and
  for rotations the dot in the walk's order, the second negated where
  it is below 0, `a + (s * b - a) * alpha` and the codec's
  normalization; each lane whose mask is clear takes the held value as
  it is. Separate multiply and add only (mani-0038), so a pose is the
  bits random access gives (mani-0019).
- **Scalar keeps the walk.** On the scalar backend the cursor plays as
  before: there the groups took more instructions than the walk under
  GCC (36,372 against 27,109 at `-O3`, with a costlier form of the
  layout step, which this saves at most 1,900 of) and about the same
  under Clang (39,714 against 39,481), and they took the core past
  ozz-animation's size, the ceiling of the size budget (mani-0036).
- **Memory.** The bytes `maniClipCursorSize` returns include the two
  group samples and the masks on a vector backend: for a 65-bone
  skeleton, 9 groups, 6,624 bytes more.

## Consequences

Forward play on `ual.fbx` takes 6,744 instructions a pose under GCC
`-O3` and AVX2 (0.42 of ACL's forward playback) and 7,236 under Clang
(0.46); 10,461 under GCC `-O2`. Random access is unchanged. A sample
held by the cursor is laid out once whatever the skeleton object, as
every skeleton a clip fits shares its identity, which covers the rests.

The clip tests play random clips of one group of bones and two through
a cursor against random access, bit for bit, with rest rotations that
are not exactly unit in binary32, -0 in rest translations, constant
channels, and rotations turning past w = 0 so held samples meet on the
longer arc; a mask that lets a rest through the interpolation, a lost
sign, or a wrong mask row fails them. CI runs them on AVX2, NEON and
the scalar build.
