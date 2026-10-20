# mani-0010. glTF animations into clips

Status: Accepted

## Context

A glTF animation is a set of channels, each a sampler's keys driving a
node's translation, rotation, scale or morph weights, or (with
`KHR_animation_pointer`) any animatable property by JSON pointer. One
file may animate several characters and props in one animation, and a
skeleton's root may sit under animated scene nodes. Clips are per
skeleton (mani-0006).

## Decision

- **Checked at open:** every sampler and channel: input accessors scalar
  floats without a stride (or one of 4, read as none), outputs of the type the specification's
  table gives each target, key and frame counts (morph frames are one
  scalar per target, end to end), cubic splines of two keys or more,
  interpolation names, a TRS channel on a matrix node refused, a target
  used twice refused (pointers to a node's TRS or weights count as that
  target). A channel with no node and no pointer is ignored, as the
  specification says.
- **`maniGltfBuildClip`** builds one animation for one opened skeleton:
  TRS channels of nodes whose names are its bones (names are fixed per
  document, mani-0009), the basis change applied to values and
  tangents; morph weights of its bones' and its skins' meshes, one
  float track per target named node `/` target (from the mesh's
  `extras.targetNames`, else the index); pointer channels to other
  properties as float tracks named by the pointer, `/` and the
  component for more than one. `boundOut` counts what the clip holds.
- **Placement:** reported placement leaves animation above the root out
  of the clip. Folded placement composes it in: the root track is
  resampled on the clip's grid from the composed transforms, so the
  clip holds them exactly at its samples. A sample whose composed
  transform does not compose back from TRS within mani-0009's
  tolerance (a non-uniform scale above a turning root shears) refuses
  the clip at the channel, so folding never changes the motion
  unannounced.
- **Duration** is the latest key of every channel in the animation, so
  one animation's clips for several skeletons share it. The clip
  builder's refusals of keys and values come back as Data naming the
  channel.

## Consequences

Hosts build the same animation for each character in a file and bind
pointer tracks themselves. The tests evaluate glTF's Appendix C
independently in binary64 and match the clip's locals, and, with
placement folded and the basis changed, its model transforms.
