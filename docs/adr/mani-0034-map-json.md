# mani-0034. The JSON form of maps

Status: Accepted

## Context

Retargeting maps (mani-0032) and mirror maps (mani-0033) are authored by
hand: bone pairs, chains, a pelvis, find/replace rules and name pairs.
Graphs already have a reviewable text form (mani-0023) that the binary
file is built from; maps need the same, so that a map lives in version
control as text and survives a reimport of its skeletons. Unity's
humanoid description, Godot's bone map and Unreal's mirror table all
name bones rather than number them, and Unity keeps a reference pose
as named local transforms.

## Decision

- **Calls:** `maniWriteRetargetMapJson` and `maniWriteMirrorMapJson`
  build the source first and, when it builds, keep its text in the
  builder in place of the file; `maniBuildRetargetMapJson` (against the
  source and target skeletons) and `maniBuildMirrorMapJson` (against
  the map's skeleton) read the text and build the file as the source
  calls do.
- **Form:** a top object with `format` ("maul-anim retarget map" or
  "maul-anim mirror map") and `version` (1); two-space indents, one
  line per list item, numbers as the shortest text that reads back to
  the same float, enumerations as words, a final new line; the graph
  form's reader, writer and refusals.
- **Bones** by name against the skeletons the host passes.
- **Reference poses** as lists of `{bone, translation, rotation,
  scale}` for the bones whose local is not the rest bit for bit, in
  bone order; a bone left out takes its rest, as does a member of an
  entry left out; a bone listed twice is refused. A missing reference
  and the rest given in full build the same file and write the same
  text.
- **A retargeting map:** `sourceReference`, `targetReference`, `pairs`
  (target, source, rotationWeight, translation of "own", "scaled" or
  "absolute", scale), `chains` (sourceStart, sourceEnd, targetStart,
  targetEnd, rotation of "none", "interpolated", "oneToOne" or
  "oneToOneReversed", rotationWeight, translation, scale), `pelvis`
  (null, or source, target, up, horizontal, vertical, absolute,
  toSource, translationWeight, rotationWeight) and `matchNames`.
- **A mirror map:** `axis` ("x", "y", "z"), `reference`, `pairs`
  (first, second), `rules` (find, place of "prefix", "suffix" or
  "anywhere", replace), `namePairs` (first, second) and `names`.
- **Strict reading:** a member an object does not take, or one given
  twice, is refused; members whose value means off may be left out
  (the lists empty, the pelvis null, matchNames false, a translation
  "own", a scale false, the pelvis's blends zero). Refusals are the
  graph form's: Json (item the byte offset, other what RFC 8259 broke)
  and Form (item the byte offset, other a `maniGraphForm`), then the
  map builder's own, items by place in the text's lists.

## Consequences

- A map's text names no identity and no index, so it outlives a
  reimport that reorders bones or changes a rest pose.
- A reference is as long as the bones that differ from the rest (an
  A pose's arms against a T pose's), so its diff shows the bones that
  moved.
- One reader and writer serve every text the library reads; a fuzz
  target reads any text as both maps.
