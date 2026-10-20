# mani-0021. Skinning and morph targets

Status: Accepted

## Context

Skins (mani-0005) give each mesh's joints and inverse binds, and the
palette composes them with a pose. Deforming the mesh needs each
vertex's influences, which glTF stores per mesh primitive (`JOINTS_n`,
`WEIGHTS_n`, any number of sets) and FBX per cluster (`Indexes`,
`Weights`), with FBX's skinning method and per-vertex blend between
linear and dual quaternion skinning. A glTF skin serves several meshes,
so influences cannot live in the skin. Morph targets are per-primitive
deltas applied before skinning, weighted by animated values.

## Decision

- **Influences** are container kind 6, a file per mesh primitive, bound
  to a skin by the skin's identity and joint count. Each vertex holds
  groups of four (joint, weight), as many groups as its heaviest vertex
  needs (at most 16): weights decreasing, the lower joint first between
  equals, padded with joint 0 at weight 0, renormalized to sum 1 in
  binary64 at build (a vertex without weight keeps all 0 and skinning
  leaves it as it is). Optional per vertex: FBX's blend toward dual
  quaternion skinning, and a center of rotation. Building refuses a
  weight negative or not finite, a joint outside the skin, a joint
  weighted twice for one vertex, and more than 64 weighted influences;
  opening checks every joint, weight, blend and center.
- **Methods:** linear blend, dual quaternion with two-phase scale and
  shear, FBX's blend of the two, and centers of rotation built offline,
  all over one influence format, as CPU jobs over vertex ranges of
  strided streams and as palettes for GPUs (row-major 3x4; dual
  quaternions with the scale and shear part).
- **Morph targets** are container kind 7, a file per primitive of named
  channels, each one or more targets at increasing full weights (FBX's
  in-betweens; glTF's targets are one at 1), over an attribute table
  (positions, normals, tangents, texture coordinate and color sets,
  every attribute glTF morphs), each target dense or sparse deltas for
  some of the table. Channels and targets are named; clip weight
  tracks of both formats are target weights and bind to targets by
  name. Channel weights become
  target weights by ufbx's walk (from an implicit zero shape through
  the keys, extrapolated past the ends); the job adds the weighted
  deltas in target order before skinning, then renormalizes normals and
  tangents.
- **Importers:** glTF's `JOINTS_n` and `WEIGHTS_n` sets, every rule
  the specification makes a MUST refused (a negative weight, a joint
  with two non-zero weights or past the skin, normalized weights not
  summing to 255 or 65535), float sums renormalized; glTF's morph
  targets over every attribute it morphs, channels named by
  `extras.targetNames` or the target's index so they meet the clips'
  weight tracks, a Draco-compressed primitive's too (Draco never holds
  targets, which follow its decoded vertex order); a Draco primitive's
  influences from its stream, which the library decodes itself
  (bitstream 2.2, sequential and Edgebreaker, every prediction Google's
  encoder writes, Google's decoder as the reference where the
  specification is silent), each accessor matching its decoded
  attribute as the extension asks. Before the decoder takes any memory
  for them, the stream's counts are bounded by the primitive's: its
  points by its attributes' accessor count, its faces by its indices'
  count (its vertex count without indices), and every other count by
  those or by the bytes left, so a stream makes the importer take no more
  than its primitive's accessors declare. FBX's clusters' `Indexes` and `Weights` per control
  point, as ufbx reads them (an index past the control points skipped,
  a vertex repeated in one cluster summed, a negative weight refused,
  weights renormalized as every source's are), `SkinningType` as the
  method and the skin's blend weights; FBX's BlendShape channels (6.x
  meshes' Shape children) as morph targets at FullWeights / 100,
  in-betweens and negative targets sorted, a shape after an equal
  target left out (ufbx weighs it 0 at every weight), Blender's
  per-vertex weights folded in, a repeated channel name numbered, each
  target named as its weight track.
- **Bits:** binary32 in a fixed order, the same on every platform.
- **Verification:** the ufbx differential (mani-0018) compares every
  skin's influences and blend weights, its control points skinned at
  rest and at each sampled clip time by the skin's method against
  `ufbx_get_skin_vertex_matrix`, and every set of morph targets
  (targets, names, full weights, offsets, rest weights) against ufbx's
  blend deformers; over ufbx's corpus no deviation is listed, dual
  quaternion skins with scaled bones included (both apply the scale
  before the rigid part). Fuzz targets read every format and importer;
  mutation sweeps cover the builders, jobs and importers.

## Consequences

A host switches methods per mesh without rebuilding data, and a mesh
binds to the wrong skin only by an identity collision. Sorting and
renormalizing once at build make every job independent of the
source's order. A weight track finds its target without the host
knowing which format produced it.

Measured in instructions (GCC 14, a Release build), a frame of a face
of 10,000 vertices with positions and normals interleaved, 52 sparse
targets and 2 dense, about 20 weighed: the job takes 2,110,783 at
`-O3` and 2,285,396 at `-O2`, against 1,861,146 and 2,207,347 for the
loop a host writes by hand for that one layout (strides and widths
fixed when compiled), with the same bits every frame: 1.134 and 1.035
of it. The rest is what a general job carries: strides, widths and
ranges read at run time.
