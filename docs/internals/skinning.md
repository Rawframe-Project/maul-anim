# Skinning and morph targets

A mesh is bound to some of a skeleton's bones; each frame the pose gives a palette and a job moves the vertices. Morph targets add weighted deltas before skinning. This page explains how the data is split across files and why the jobs are general strided jobs.

## Skins and palettes

A skin (kind 5) lists the bones a mesh is bound to, in the mesh's order (its joints), each with an inverse bind matrix, and records its skeleton's identity. glTF and FBX attach several meshes, each with its own list, to one hierarchy, so several skins over one skeleton are several files.

`maniComputePalette` writes entry j as the model matrix of joint j's bone times joint j's inverse bind. It refuses a stale model, a skeleton that is neither the skin's nor the pose's, and a short array; a forgotten model pass is a refusal, never an old palette. `maniPaletteDualQuaternions` gives the same palette as dual quaternions with their scale and shear part, for dual quaternion skinning on a GPU.

## Influences

A glTF skin serves several meshes, so a vertex's influences cannot live in the skin. Influences (kind 6) are a file per mesh primitive, bound to a skin by its identity and joint count. Each vertex holds groups of four (joint, weight), as many groups as its heaviest vertex needs (at most 16): weights decreasing, the lower joint first between equals, padded with joint 0 at weight 0, renormalized to sum to 1 in binary64 at build. Sorting and renormalizing once at build makes every job independent of the source's order. A vertex with no weight keeps all zeros and skinning leaves it as it is.

Optional per-vertex data: FBX's blend toward dual quaternion skinning, and a center of rotation (`maniBuildCenters`, computed offline).

## The jobs

One influence format serves every method: linear blend (`maniSkinLinear`), dual quaternion with two-phase scale and shear (`maniSkinDualQuaternion`), FBX's per-vertex blend of the two (`maniSkinBlended`), and centers of rotation. Each is a CPU job over a range of vertices of strided streams (positions, normals, tangents, directions), so a host skins its own vertex layout in place and splits a mesh across threads by ranges ([`src/skinning.c`](../../src/skinning.c)). Strides and widths are read at run time; that generality costs a few percent against a loop written by hand for one fixed layout, with the same bits.

## Morph targets

Morph targets (kind 7) are a file per primitive of named channels, each one or more targets at increasing full weights (FBX's in-betweens; glTF's targets are one at 1), over an attribute table (positions, normals, tangents, texture coordinate and color sets: every attribute glTF morphs), each target dense or sparse deltas for some of the table ([`src/morph.c`](../../src/morph.c)).

Clips carry morph weights as float tracks named by target; FBX channel weights are turned into target weights at import, following ufbx's walk from an implicit zero shape through the in-betweens, so the runtime has one weighting model whichever format the data came from. The job adds the weighted deltas in target order before skinning, then renormalizes normals and tangents.

## Verification

The ufbx differential compares every FBX skin's influences and blend weights, its control points skinned at rest and at each sampled clip time by the skin's method against ufbx's skin matrices, and every set of morph targets against ufbx's blend deformers; over ufbx's corpus no deviation is listed, dual quaternion skins with scaled bones included. Every opener and importer is fuzzed.

## Traps

- Influences are bound to a skin by identity and joint count, not to a skeleton; a mesh rebound to another skin needs its influences rebuilt.
- Changing the order in which a vertex's influences are summed changes the bits of every skinned vertex; the influences' sorted order is what makes that order fixed.
