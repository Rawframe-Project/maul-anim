# mani-0005. Skins and palettes

Status: Accepted

## Context

A mesh is bound to some of a skeleton's bones, in its own order, each
with the inverse of the bone's model matrix at bind time. glTF and FBX
attach several meshes, each with its own list, to one hierarchy, and
renderers skin with one matrix per listed bone: the palette.

## Decision

- **A skin file** (container kind 5): the skeleton's identity and bone
  count, the joints (skeleton bones in the mesh's order) and an inverse
  bind matrix (3x4) per joint. Several skins over one skeleton are
  several files. A bone may be listed twice; its entries are then
  equal. The file's identity covers every section.
- **Opening** (`maniOpenSkin`) checks the sizes, every joint against
  the recorded bone count, every matrix finite and the identity.
- **The palette** (`maniComputePalette`): entry j is the model matrix
  of joint j's bone times joint j's inverse bind. It refuses a pose
  whose model is stale, a skeleton that is not the skin's or the
  pose's, and a short array (reporting the count).
- **Building** (`maniBuildSkin`) takes an opened skeleton, joints as
  bone indices and inverse binds, and refuses a bone past the skeleton
  or a matrix not finite with the joint's index.

## Consequences

A skin can never be applied to the wrong skeleton, a forgotten model
pass is a returned refusal instead of an old palette, and importers
have a place to put every mesh's binding.
