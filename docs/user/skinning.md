# Skinning and morph targets

A skin binds a mesh to some of a skeleton's bones; a mesh primitive's influences say which of those joints move each vertex and by how much. Each frame the pose gives a palette, and a job moves the vertices, or a renderer takes the palette to the GPU.

## Skins and influences

A skin lists its joints (skeleton bones, in the mesh's order), each with an inverse bind matrix. Influences are built per mesh primitive against an opened skin:

```c
const uint32_t joints[2] = {1, 2};   // the spine and the head
const maniAffine inverseBinds[2] = {
    {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0.0f, -1.2f, 0.0f}},   // undoing each joint's rest
    {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0.0f, -1.6f, 0.0f}}};
maniSkinSource skinSource = {&skeleton, joints, inverseBinds, 2};
maniBuildResult skinBuilt = maniBuildSkin(builder, &skinSource);
size_t skinSize = 0;
maniResult skinCopied = maniBuilder_CopyBuilt(builder, skinFile, skinCapacity, &skinSize);
maniSkin skin;
maniFileResult skinOpened = maniOpenSkin(skinFile, skinSize, &skin);

// Two vertices: one on the spine, one shared by the spine and the head.
const uint32_t offsets[3] = {0, 1, 3};
const uint32_t vertexJoints[3] = {0, 0, 1};
const float weights[3] = {1.0f, 0.5f, 0.5f};
maniInfluenceSource influenceSource = {&skin, offsets, vertexJoints, weights, NULL, 2};
maniBuildResult influencesBuilt = maniBuildInfluences(builder, &influenceSource);
size_t influencesSize = 0;
maniResult influencesCopied =
    maniBuilder_CopyBuilt(builder, influencesFile, influencesCapacity, &influencesSize);
maniInfluences influences;
maniFileResult influencesOpened = maniOpenInfluences(influencesFile, influencesSize, &influences);
```

The builder sorts each vertex's influences and renormalizes their weights to sum to 1, so a source may list them in any order. A vertex may have up to 64 weighted influences. The importers build skins and influences from glTF and FBX files directly.

## Skinning a frame

Each frame, the pose's model gives the palette, and a job moves the vertices:

```c
maniAffine palette[2];
maniResult paletted = maniPose_ComputePalette(pose, &skeleton, &skin, palette, 2, NULL);
const float positions[6] = {0.1f, 1.3f, 0.0f, 0.1f, 1.5f, 0.0f};
float moved[6];
const maniSkinStream stream = {positions, moved, 12, 12, maniStreamPoint};
const maniSkinJob job = {&influences, palette, NULL, 2, 0, 2, 1, &stream};
maniResult skinned = maniSkinLinear(&job);
```

The palette needs a current model (`maniPose_ComputeModel` first); a stale one is refused. Normals, directions and tangents are further streams of the same job, each with its own stride, so a program skins its own vertex layout in place. Jobs over disjoint ranges of vertices may run on several threads at once.

Methods share the one influence format, so a program switches per mesh without rebuilding data:

- `maniSkinLinear`: linear blend skinning.
- `maniSkinDualQuaternion`: dual quaternion skinning, which keeps volume at twisting joints; its palette from `maniConvertPaletteToDualQuaternions`.
- `maniSkinBlended`: a per-vertex blend of the two, as FBX skins carry it.
- Centers of rotation (`maniBuildCenters`), computed offline per vertex.

For GPU skinning, the palette (row-major 3x4) or the dual quaternion palette goes to the shader as it is.

## Morph targets

Morph targets add weighted deltas to a primitive's attributes (positions, normals, tangents, texture coordinates, colors) before skinning. A morph target file holds named channels, each one or more targets at increasing weights (FBX's in-betweens; a glTF target is one at weight 1). Clips carry the weights as float tracks named by target, so `maniClip_SampleFloats` gives what `maniApplyMorphTargets` takes, whichever format the data came from; `maniMorphTargets_ComputeTargetWeights` turns a channel's weight into its targets' when a program drives channels itself. The job adds the deltas over vertex ranges of strided streams, as the skinning jobs do, and renormalizes normals and tangents. The importers build morph target files from glTF and FBX (`maniBuildGltfMorphTargets`, `maniBuildFbxMorphTargets`).

## Not obvious

- Influences are bound to a skin, not a skeleton: several primitives of one mesh share a skin, and a skin serves every mesh that lists the same joints.
- A vertex with no weight at all is left where it is.
- The palette maps from bind space to the pose's model space. A skeleton imported with its placement reported, not folded, needs the placement applied on top.
