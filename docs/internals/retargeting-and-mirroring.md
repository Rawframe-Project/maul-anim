# Retargeting and mirroring

Retargeting carries poses from one skeleton to another whose bones have other names, other counts along the spine and limbs, other rest orientations and other proportions. Mirroring carries a skeleton's poses across a plane, swapping left and right. Both are maps built offline and opened in place, so the run-time transfer only composes rotations; both bake whole clips with the same core.

## Retargeting maps

Matching bones by name fails across different rests, and rewriting a rig's rests to a common profile loses data. So a map (kind 4) is built for two skeletons named by identity and stores, per target bone, a plan: keep its reference, follow one source bone, or sit between two neighbors along a chain at a place; a rotation weight; how its translation carries (its own, the source's change scaled, the source's itself, the pelvis's) and whether its scale carries. Both reference poses (each rest, or a pose given for it) are stored with their model rotations.

- **Chains** of any lengths on each side map their bones by mode: interpolated (each bone placed by its distance along its chain at the reference pose, and a target bone slerped between the two source bones around its place), one to one from the start or from the end, or none; extra target bones keep their reference.
- **The pelvis** carries its motion scaled by the ratio of the two pelvises' heights along an up axis, with horizontal and vertical factors and per-axis blends toward the unscaled change and toward the source's own position, as Unreal's IK Retargeter does.

### The transfer

`maniRetargetMap_RetargetPose` ([`src/retarget_pose.c`](../../src/retarget_pose.c)) gives a mapped target bone the model rotation `S R_s^-1 R_t`: its source bone's model rotation S, relative to the source's reference model rotation, applied to its own reference. That is Godot's global form, which is right across bones only one rig has, and each target bone keeps its own lengths. Its local is then taken against its parent as just made. Scaled translations are the target reference's plus the source's change times the height ratio, turned from the source parent's reference frame to the target parent's. A bone in exactly its source's frame (one source at weight 1, identical reference locals and parents) copies the source's local as it is, so a rig mapped to itself transfers bit for bit.

The caller lends scratch for a rotation per bone of both skeletons; nothing is kept between calls. A humanoid's transfer takes about 35 thousand instructions.

### Clips and graphs

`maniBuildRetargetedClip` bakes a clip of the source into a full-precision clip of the target on the same grid, each sample carried by the transfer; float tracks keep their interpolation, events and markers are copied. Root motion carries as the source's stored moves, turned between the two root bones' frames and scaled, rather than extracted again from the baked poses, which is exact at the samples and independent of the root bone's translation mode.

A graph's clip slot may name a map; the clip is then the map's source's, and every reader of the slot (clip nodes, events, sync, root motion) sees one clip carried one way. Evaluation samples the clip into a source pose and transfers it in scratch sized at binding. Playing another rig's clip live costs about three native clip samples more a frame; a crowd that shares clips bakes them.

## Mirror maps

Mirroring a pose swaps left and right bones and reflects their motion, but rigs rarely give a left bone the mirrored axes of its right partner: reflecting local rotations as they are twists such limbs. Unreal's mirror table corrects every bone through the reference pose's component-space rotations; the map here (kind 13) does the same with the corrections folded in at build.

A map pairs each bone with its partner (itself when unpaired), from explicit pairs first, then find/replace rules on names (prefix, suffix or anywhere; rules run one way, so both directions are given, and a rule's partner must pair back). Names of float tracks, events and markers are paired the same way.

With P a bone's parent's reference model rotation, R its own and m its partner, each bone keeps `A = P_b^-1 mirror(P_m)` and `B = mirror(R_m)^-1 R_b`, where mirror negates a quaternion's two vector components off the plane's normal. `maniMirrorMap_MirrorPose` ([`src/mirror_pose.c`](../../src/mirror_pose.c)) then gives bone b `A mirror(T_m)` and `A mirror(Q_m) B`: two products and a rotation a bone, no model pass, no scratch. Since mirror distributes over products, this equals Unreal's `P_b^-1 mirror(P_m Q_m R_m^-1) R_b`. It is exact on a mirror-symmetric hierarchy; the map flags every bone that breaks it (a prop under one hand) without refusing it (`maniMirrorMap_GetBoneFlags`, `maniMirrorMap_GetUnmatchedBone`). A partner's scale carries through B exactly when B takes axes to axes, and as the nearest axis scale otherwise, flagged.

`maniBuildMirroredClip` bakes a mirrored clip with the retargeting bake's core: samples mirrored, names renamed, root motion moved to its bone's partner and reflected. A graph's mirror node mirrors its input in place; the builder marks each node with the parity of mirrors above it and refuses one reached both mirrored and not, so a mirrored clip's events and sync markers are renamed and two mirrors cancel. A motion database entry flagged mirrored is its clip's span mirrored without a bake.

## The JSON form

Maps are authored by hand, so both have a text form (`maniWriteRetargetMapJson`, `maniBuildRetargetMapJson`, and the mirror pair) sharing the graph form's reader, writer and refusals ([`src/map_json.c`](../../src/map_json.c)). Bones are named, never indexed, so the text outlives a reimport that reorders bones. A reference pose lists only bones whose local differs from the rest, so a diff shows the bones that moved, and a missing reference and the rest given in full write the same text.

## Traps

- A retargeting map's transfer is a function of both references; a map built with an A pose against a T pose rig depends on those references being given, not on the rests.
- The pelvis's parent's matrix is taken from the target locals made so far in the same pass; reordering the pass breaks it.
- Mirroring is exact only on mirror-symmetric hierarchies. Flagged bones take their partner's local turn mirrored, which is right for props that follow one hand and wrong for anything that needs its world position reflected.
