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
  channel chosen, a field out of range, or an additive clip is refused.
- **Taken out at build:** at every sample, in binary64, the motion `M`
  and the bone's stored transform `M^-1 L` (translation
  `m_r^-1 (t - m_t)`, rotation `m_r^-1 r`), so the clip plays in place
  and `M` times the stored bone is the source.
- **Stored** as section 16 of the clip file, in either form: the
  source, the whole period's move (from the first sample to the last,
  in the first's frame), and per sample the motion's translation and
  rotation in binary32. The compressor carries it unchanged. Files
  without root motion are unchanged, their identity too.

The moves between playhead times across wraps, their blending and
composition, and restoring the motion into a pose follow in this
record as they land (item 7).

## Consequences

A host moves its character by moves it reads beside the clip, with no
extraction at runtime. Masking by axes and a twist about an axis
covers the common choices (horizontal travel, turning about up)
without an Euler order.
