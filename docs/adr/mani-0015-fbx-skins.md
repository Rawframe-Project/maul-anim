# mani-0015. FBX skins

Status: Accepted

## Context

An FBX skin is a deformer whose clusters each link a bone model and
hold its vertex weights and bind matrices: `Transform`, the mesh node
relative to the bone at bind, and `TransformLink`, the bone's world at
bind. Exporters omit these at times and leave a bind pose instead. The
runtime's skin is a list of skeleton bones with an inverse bind each
(mani-0004).

## Decision

- **Joints** are the skin's clusters in connection order that link a
  model, found in an opened skeleton by the model's bone name
  (refused, Bone, when missing); a model linked twice is two joints.
- **Inverse binds** are the cluster's `Transform` times the mesh
  model's geometric transform, as ufbx reads them. Without `Transform`
  (or `TransformLink`), the bone's bind world is `TransformLink`, else
  its matrix in a bind pose, else its world at rest, and the mesh
  node's world at rest is taken relative to it. Matrices that are not
  affine, too short or not invertible are refused; additive clusters
  are refused rather than bound wrongly.
- **Axes and units** apply to binds as to rests (`B M B^-1`, the
  translation scaled); placement needs no correction.
- **Vertex weights** and the skinning method are read into the
  influence format of the skinning slice, with glTF's, once it exists.

## Consequences

FBX skins bind to skeletons from the same file or from any file whose
bone names match, and agree with ufbx's binds for the differential.
