# Retargeting and mirroring

A retargeting map plays one skeleton's animation on another; a mirror map plays a skeleton's animation mirrored, left for right. Both are built offline for their skeletons and work on poses, whole clips, graphs and motion databases.

## Retargeting

A map carries poses from a source skeleton to a target: bones paired by name or explicitly, chains of different lengths (a three-bone spine onto a five-bone one), the pelvis's motion scaled by the two rigs' heights, and each target bone keeping its own lengths. Rigs whose bones rest in different orientations retarget correctly, since each bone's turn is carried through both rigs' reference poses.

```c
maniRetargetMapSource retargetSource = {.source = &skeleton, .target = &tall, .matchNames = true};
maniBuildResult mapBuilt = maniBuildRetargetMap(builder, &retargetSource);
size_t mapSize = 0;
maniResult mapCopied = maniCopyBuilt(builder, mapFile, mapCapacity, &mapSize);
maniRetargetMap map;
maniFileResult mapOpened = maniOpenRetargetMap(mapFile, mapSize, &map);
maniResult carried = maniRetargetPose(&map, &skeleton, pose, &tall, tallPose, retargetScratch,
                                      maniRetargetScratchSize(&map));
```

When the rigs' rest poses differ in a way that matters (an A pose against a T pose), the map's source takes a reference pose for either side; the transfer then carries each bone's turn from that reference.

`maniBuildRetargetedClip` bakes a whole clip for the target, root motion, events and tracks included. A graph's clip slot may also name a map and play the source's clip live; that costs about three native clip samples more a frame, so a crowd that shares clips bakes them. `maniRetargetRootMotion` scales a move the same way.

## Mirroring

A mirror map pairs left and right bones, by rules on their names or explicitly, and folds in the rest pose's corrections, so rigs whose left and right bones do not have mirrored axes mirror correctly:

```c
const maniMirrorRuleSource sides[2] = {{"L", 1, maniMirrorSuffix, "R", 1},
                                       {"R", 1, maniMirrorSuffix, "L", 1}};
maniMirrorMapSource mirrorSource = {
    .skeleton = &skeleton, .rules = sides, .ruleCount = 2, .axis = maniMirrorAxisX};
maniBuildResult mirrorBuilt = maniBuildMirrorMap(builder, &mirrorSource);
size_t mirrorSize = 0;
maniResult mirrorCopied = maniCopyBuilt(builder, mirrorFile, mirrorCapacity, &mirrorSize);
maniMirrorMap mirror;
maniFileResult mirrorOpened = maniOpenMirrorMap(mirrorFile, mirrorSize, &mirror);
maniResult mirrored = maniMirrorPose(&mirror, &skeleton, pose, pose);   // in place
```

Rules run one way, so both directions are given. Names of float tracks, events and markers are paired the same way, so a mirrored clip's `left_foot_down` event becomes `right_foot_down`. Mirrored clips (`maniBuildMirroredClip`), a graph's mirror node and mirrored motion database entries use the same map.

## As text

Both maps have a JSON form (`maniWriteRetargetMapJson`, `maniBuildRetargetMapJson`, `maniWriteMirrorMapJson`, `maniBuildMirrorMapJson`) that names bones rather than numbering them, so a map lives in version control and survives a reimport that reorders bones.

## Not obvious

- A bone that breaks the rig's left-right symmetry (a prop under one hand) is mirrored as well as it can be and flagged; `maniMirrorBoneFlags` and `maniMirrorMapUnmatchedBone` find such bones.
- A rig retargeted to itself comes out bit for bit as it went in.
- A map is tied to both skeletons by identity; reimporting either with a changed rest or bone set means rebuilding the map, from its JSON text if it was written as one.
