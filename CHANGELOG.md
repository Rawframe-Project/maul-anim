# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and every data
format.

## [Unreleased]

### Added

- Two-bone inverse kinematics (mani-0029): `maniSolveTwoBone` bends a
  chain of three bones of a pose to reach a model-space goal, its bend
  toward a pole that never flips near the goal line, with an optional
  hinge, twist, softening and weight, reading and writing locals only.
- Aim and look-at (mani-0029): `maniSolveAim` turns a bone's forward
  (from an offset point) onto a goal, its up toward a pole, or spreads
  the turn over a chain of up to 8 bones by shares, within an optional
  cone.
- CCD chains (mani-0030): `maniSolveCcd` bends a chain of up to 32
  bones, named by its root and end, toward a goal by cyclic coordinate
  descent with a step bound, an iteration count and a tolerance, each
  bone within an optional cone, twist or hinge limit relative to its
  rest (`maniJointLimit`), the end optionally turned to a wanted
  rotation.
- FABRIK chains (mani-0030): `maniSolveFabrik` bends the same chains by
  forward and backward reaching, limits applied as each bone turns in
  the forward pass, an optional pole for the chain's bend.
- Chain IK graph nodes (mani-0030): CCD and FABRIK nodes solve their
  input pose toward vec3 goal and pole parameters and a quat rotation
  for the end, with limits per bone, in files, JSON and subgraphs.
- Full-body IK (mani-0031): `maniSolveBodyXpbd` bends every bone on
  the paths from a top bone to up to 32 effectors as rigid bodies with
  XPBD compliance: effector strength, rotation and position stiffness,
  limits, preferred angles, pull chains, a pinned, free or pre-pulled
  top, optional stretch, in caller scratch (`maniBodyIkScratchSize`).
- Full-body IK by selectively damped least squares (mani-0031):
  `maniSolveBodySdls` solves the same description in one least-squares
  step an iteration (Buss and Kim, the paper's clamps, a one-sided
  Jacobi decomposition whose sweeps stop the same on every platform),
  every length kept.
- Retargeting maps (mani-0032): `maniBuildRetargetMap` builds a map
  between two skeletons from bone pairs, chains of any lengths
  (interpolated, one to one, reversed), the pelvis scaled by the
  reference heights, names and a reference pose per side, planned per
  target bone; `maniOpenRetargetMap` opens it in place.
- Pose retargeting (mani-0032): `maniRetargetPose` makes a target pose
  from a source pose through a map, each mapped bone's model turn from
  its reference the source's (slerped along chains, weighted), the
  target's own lengths kept, translations scaled or carried through the
  parents' reference frames, the pelvis scaled by parts;
  `maniRetargetRootMotion` scales root motion the same way.
- Clip retargeting (mani-0032): `maniBuildRetargetedClip` bakes a clip
  of the source into a full-precision clip of the target through a
  map, each sample carried by the pose transfer, channels left at the
  rest left out, additive clips carried as differences, float and
  discrete tracks, events and markers kept, and root motion's moves
  scaled and turned into the target bone's parent's frame.
- Mirror maps (mani-0033): `maniBuildMirrorMap` pairs a skeleton's
  bones by explicit pairs and find/replace rules on their names, folds
  the reference pose into two corrections a bone so rigs whose left and
  right bones have unmirrored axes mirror to their reference, and pairs
  float track, event and marker names; `maniOpenMirrorMap` opens it,
  `maniMirrorPartner`, `maniMirrorName` and
  `maniMirrorMapUnmatchedBone` read it.
- Mirrored poses (mani-0033): `maniMirrorPose` gives each bone its
  partner's local reflected and carried through the map's corrections,
  in place or into another pose; `maniMirrorRootMotion` reflects a root
  motion move.
- Mirrored clips (mani-0033): `maniBuildMirroredClip` bakes a clip's
  mirror on its grid, additive clips mirrored from the rest mirrored,
  float tracks, discrete tracks, events and markers renamed by the
  map's name pairs, root motion moved to its bone's partner with its
  moves reflected in the model.
- Graph mirror nodes (mani-0033): `maniGraphNodeMirror` mirrors its
  input through the graph's mirror map (`maniGraphSource.mirrorMap`,
  bound from `maniGraphAssets.mirrorMap`, read by
  `maniGraphMirrorMapIdentity`); the clip, match and learned nodes
  beneath it give their moves reflected and their events and sync
  markers renamed, with `maniMirrorNameHash` for hashed names; the
  graph format is version 3.
- Mirrored motion database entries (mani-0033): `MANI_MATCH_MIRRORED`
  samples an entry's clip mirrored through the database source's
  `mirrorMap` (features exactly a mirrored bake's, float track channels
  from the partner tracks), read by
  `maniMotionDatabaseMirrorMapIdentity`; matching nodes play such
  entries mirrored; the database format is version 2.
- The JSON form of maps (mani-0034): `maniWriteRetargetMapJson` and
  `maniWriteMirrorMapJson` write a map's source as text after building
  it, bones by name and each reference pose as the bones not at their
  rest; `maniBuildRetargetMapJson` and `maniBuildMirrorMapJson` read
  the text strictly, refusing at byte offsets, and build the same file.
- Graph structure and inspection (mani-0035): `maniGraphNodeKindOf`,
  `maniGraphNodeInputCount`, `maniGraphNodeInput`, `maniGraphOutputNode`
  and `maniGraphNodeSourcePlace` (each node's place in its source, kept
  in the file: graph format version 4) draw a graph's tree;
  `maniInspectGraphNode` reads what a node did in the last update (its
  share of the output, playhead, a machine's state and transition under
  way with its fade, a one-shot's phase, an inertialization's easing, a
  matching node's entry).
- Retargeted playback in graphs (mani-0032): a clip slot may name a
  retargeting map (`maniGraphClipSource.map`) to play a clip of the
  map's source skeleton, carried at each evaluation; map slots read by
  `maniGraphMapCount`, `maniGraphMapIdentity`,
  `maniGraphMapSourceIdentity` and `maniGraphClipMap`, bound through
  `maniGraphAssets.maps` and `sourceSkeletons`; root motion carried.
- Full-body IK by SDLS 2.4 to 9 times fewer instructions an iteration
  (mani-0031): its decomposition keeps each column's length and skips
  the zero rotation rows, with the same bits.
- Full-body IK graph nodes (mani-0031): XPBD and SDLS nodes solve their
  input pose toward vec3 position and quat rotation parameters per
  effector, with bone settings, in files, JSON and subgraphs, their
  scratch in the instance's room.
- IK graph nodes (mani-0029): two-bone and aim nodes solve their input
  pose toward vec3 goal and pole parameters, their twist, soften, cone
  and weight numbers or float parameters, in files, JSON and subgraphs.
- `mani_errorState`: a builder's functions, closing a document opened
  with it, and training, restoring and destroying a learned trainer
  refuse a call back in from the object's allocator or task hooks,
  counting one misuse; `maniLearnedTrainerMisuses` reads a trainer's
  count (mani-0028).
- Graph files (mani-0023): `maniBuildGraph` builds a graph against a
  skeleton and its clips (parameters, clip slots, sync groups, and
  clip, blend, mask, additive, select, time scale and seek nodes as a
  DAG), ordering the nodes, marking poses and differences and reusing
  pose slots; `maniOpenGraph` opens one in place with every record
  checked; parameters and clip slots are read by name and index.
- Graph bindings and instances (mani-0023): `maniBindGraph` binds a
  graph's clip slots to the host's clips by identity in caller memory;
  `maniInitGraphInstance` makes an instance in caller memory that a
  `memcpy` copies; `maniSetGraphParameter` and `maniGetGraphParameter`
  set (clamped, normalized, checked) and read parameters;
  `maniSaveGraphInstance` and `maniRestoreGraphInstance` save a state
  without pointers and restore it after checking it as external bytes.
- Graph updates (mani-0023): `maniUpdateGraph` shares weights down from
  the output, moves clip playheads by speed and time scales (seeks set
  them), lists the events clip nodes pass across any number of wraps
  (counted past the buffer), and blends the clips' root motion by the
  weights the output gives it.
- Graph evaluation (mani-0023): `maniEvaluateGraph` writes an
  instance's pose through a per-thread scratch
  (`maniGraphScratchSize`): every weighed node in order into its slot
  (clips through their cursors, blends, masks and additive layers by the
  blending functions, bit for bit), unweighed nodes skipped.
- Graph sync groups and subgraphs (mani-0023): clip nodes of a sync
  group follow a leader (the declared one, else the member weighed
  most) by normalized phase, only the leader's events firing unless the
  group asks for all, by the markers common to all members when they
  repeat and share one (names matched, a follower placed where the
  leader's names and fraction are); `maniGraphNodeSubgraph` inlines
  another graph source at build, its parameters read through a map,
  its clips joined by identity and its sync groups by name; slots
  without their asset join one slot, which is refused (they could
  overrun the flattened lists).
- Graph text form (mani-0023): `maniWriteGraphJson` writes a graph
  source as JSON once it builds, names in place of indices (bones of
  mask chains included), subgraphs nested, a line per item, numbers as
  the shortest text that reads back to the same binary32.
- `maniBuildGraphJson` builds a graph from its JSON form against the
  host's clips by slot name, members strict, off values optional; texts
  that are not JSON or not the form are refused with the byte offset
  (`maniBuildReasonJson`, `maniBuildReasonForm`, `maniGraphForm`).
- Blend spaces (mani-0023): `maniGraphNodeBlendSpace1D` weighs its
  inputs by where a position falls among their points on a line,
  clamped or wrapped by a period; `maniGraphNodeBlendSpaceBands` by
  gradient bands, cartesian or polar, among points in a plane; the
  position a vec2 parameter or two numbers (`maniGraphPosition`);
  points not finite, equal or past a period refused
  (`maniBuildReasonPoint`); both in the JSON form.
- Blend spaces by triangles (mani-0023):
  `maniGraphNodeBlendSpaceTriangles` weighs the corners of the triangle
  holding the position, or outside them the ends of the nearest edge;
  the triangles are the author's (`maniGraphTrianglesNodeSource`,
  checked exactly for orientation and overlap,
  `maniBuildReasonSpaceTriangle`) or the points' Delaunay triangulation,
  made with exact integer predicates so every platform builds the same
  file; in the JSON form as `blendSpaceTriangles`.
- State machines (mani-0023): `maniGraphNodeMachine` blends its states
  (any subgraph, machines included) by weights its transitions set:
  from a state or any state (`MANI_GRAPH_ANY_STATE`), in priority order
  (no two equal among a state's candidates), when all their conditions
  hold (bool, int and float comparisons, triggers consumed on firing,
  time in state); cross-faded linearly or by smoothstep over seconds, an
  interrupted fade going on from the weights it reached; interruption
  none, by a lower priority value or by any; cooldowns; instant
  transitions chained to a limit; entry from the start or where the
  state was left. Machine state is saved and restored with the
  instance; refusals name the transition (`maniBuildReasonTransition`);
  in the JSON form as `machine`.
- State machine conditions on the current state's leading clip (the
  clip node beneath it weighed most): its phase passed in the update
  before (`maniGraphConditionPhase`, every loop below 1, once at 1 or
  more), the state finished (`maniGraphConditionFinished`: a clamped
  clip at its end, or a nested machine in one of its `finals` with no
  fade running), and an event name heard from any clip beneath the
  state (`maniGraphConditionEvent`); `maniGraphEntryMatch` enters a
  state at the leading clip's normalized phase. A machine no parent
  reaches does not step, so its triggers wait.
- One-shots (mani-0023): `maniGraphNodeOneShot` plays a shot over a
  base. A fire trigger starts the shot over and fades it in from its
  weight; it fades out when its leading clip has the fade-out's seconds
  of clip time left, or on a fade-out trigger; an abort trigger stops it
  at once. An optional mask weighs the shot per bone. Its state is saved
  and restored with the instance; in the JSON form as `oneShot`.
- Inertialization (mani-0023): `maniGraphNodeInertialize` eases out
  the jumps of its input. A machine transition faded `inertial`
  switches at once and asks the nearest such node above it for its
  duration, as the node's own trigger does; the least request of an
  update wins. Each bone's offset from the node's last output decays
  by Bollo's quintic (translation and scale by direction and length,
  rotation by axis and angle), from the speed of its last two outputs
  when they came from consecutive updates. The builder refuses an
  inertial transition with no node above it. Its state is saved and
  restored with the instance; in the JSON form as `inertialize`.
- Dead blending (mani-0023): an inertialize node whose method is
  `maniGraphDeadBlending` extrapolates its last output by its speed
  (translation, rotation as a scaled axis-angle, scale in log space),
  each axis decaying with Holden's half-life (`halfLifeScale` times the
  distance over the speed, between `halfLifeMin` and `halfLifeMax`),
  and cross-fades it into its input by a smoothstep.
- Motion databases (mani-0024): `maniBuildMotionDatabase` samples a
  skeleton's clips at a rate into frames of weighted channels (bone
  positions, velocities and facings, the root's travel and facing at
  times ahead and behind, float tracks) in the root's space, each
  channel normalized on its own, over entries that loop or clamp and
  carry cost biases, tags and a no-jump flag; `maniOpenMotionDatabase`
  opens one in place with every byte checked and the bounds over runs
  of 16 and 64 frames recomputed; channels, entries, frames, features
  and the normalization read back.
- Motion matching search (mani-0024): `maniSearchMotionDatabase` finds
  the frame of least cost exactly, Holden's bounds pruning without
  changing the result, with a continuing bias, a transition cost,
  entry biases, tag masks, and exclusions near the continuing frame,
  at clamped ends and around up to eight earlier picks.
- Motion matching nodes (mani-0024): `maniGraphNodeMatch` plays a
  database's frames while weighed, searching on a period, on a trigger, at a clamped
  entry's end and with no entry, its query the continuing frame's bone
  channels and parameters for its trajectory and float channels; a
  jump requests inertialization of the inertialize node above, which
  the build requires. Graphs list database slots (`maniGraphDatabaseSource`,
  `maniGraphDatabaseCount`, `maniGraphDatabaseIdentity`,
  `maniGraphDatabaseName`) whose clips are clip slots; `maniBindGraph`
  and `maniBuildGraphJson` take the host's databases; the node's block
  joins the saved state; subgraphs join databases by identity; the
  JSON form writes a "databases" list and "match" nodes.
- Trajectory prediction (mani-0024): `maniAdvanceTrajectory` moves a
  simulated root toward a desired velocity and rotation by Holden's
  critically damped springs in closed form; `maniTrajectoryFeatures`
  gives a sample's values for a database's trajectory channels.
- Learned motion matching models (mani-0025): `maniBuildLearnedModel`
  builds a model of a Decompressor, a Stepper and Projectors (one a
  tag filter) over a motion database's schema, with binary32 or 16-bit
  weights; `maniOpenLearnedModel` opens one in place with every record
  checked; `maniLearnedProject`, `maniLearnedStep` and
  `maniLearnedDecompress` evaluate it with the same bits on every
  target (the pose from two rotation columns, root motion, float
  tracks and events' states).
- Training learned motion matching models in the library (mani-0025):
  `maniCreateLearnedTrainer` takes a motion database, its clips and
  skeleton, the Projectors' tag filters and Holden's options;
  `maniTrainLearned` trains the Compressor and Decompressor, the
  Stepper and each Projector in turn with Holden's losses and AdamW,
  the same bits with any thread count; `maniSaveLearnedCheckpoint` and
  `maniRestoreLearnedCheckpoint` stop and resume to the same bits (a
  restored checkpoint saves back to the same sections);
  `maniBuildLearnedModelFromTrainer` builds the model.
- Learned model slots in graphs (mani-0025): `maniGraphModelSource`,
  `maniGraphModelCount`, `maniGraphModelIdentity` and
  `maniGraphModelName`; binding finds each slot's model by identity;
  the JSON form writes a "models" list.
- Learned motion matching nodes (mani-0025): `maniGraphNodeLearned`
  plays a model slot's state, stepped at the model's rate (at most
  `MANI_GRAPH_LEARNED_MAX_STEPS` steps an update, the rest of a longer
  one lost) and interpolated between steps, projected from its query parameters at
  its period or trigger and replaced past its transition cost with an
  inertialization requested above it; the Decompressor gives the pose,
  the root motion and events; saved states, subgraphs and the JSON form
  carry it.
- Spring sets (mani-0026): `maniBuildSpringSet` builds a skeleton's
  spring bones (VRMC_springBone 1.0 with its extended colliders, inside
  spheres and capsules and planes, and its cone, hinge and spherical
  limits), each joint's drag, stiffness and gravity converted from a
  frame of a reference rate (60 Hz by default) to a fixed step;
  `maniOpenSpringSet` opens one in place with every record checked
  (format 2: each joint keeps the factor a force is converted by).
  `maniColliderSource` gains the plane shape and inside colliders,
  `maniSpringJointSource` the limits, `maniSpringSetSource` the rates.
  `maniGltfReadSprings` reads both extensions (one of another version
  leaves the collider's fallback shape and the joint unlimited), a
  limit's frame carried through the options' basis change.
- Spring set slots in graphs (mani-0026): `maniGraphSpringSetSource`,
  `maniGraphSpringSetCount`, `maniGraphSpringSetIdentity` and
  `maniGraphSpringSetName`; binding finds each slot's set by identity
  through `maniGraphAssets`; the JSON form writes a "springSets" list.
  The graph file gains section 13 and a 64-byte header; graph files
  built before do not open.
- Ragdoll profiles (mani-0027): `maniBuildRagdoll` builds a skeleton's
  ragdoll for a host's physics engine: bodies on a subset of the bones
  (a frame, a sphere, capsule or box, a mass, a name), each joined to
  the body of its bone's nearest ancestor with one by a swing-twist
  joint with limits and an optional drive, its frame in the parent body
  worked out from the rest pose, and the root body's facing and up
  axes; `maniOpenRagdoll` opens one in place with every record checked
  and `maniRagdollReadBody` reads its bodies by bone.
- Ragdoll bodies and poses (mani-0027): `maniRagdollBodiesFromPose`
  places a profile's bodies where a pose puts them under the
  character's placement, with velocities from the pose before;
  `maniRagdollJointTargets` gives each joint's motor target (rotation
  and angular velocity in the parent's joint frame);
  `maniRagdollPoseFromBodies` makes a pose from the bodies' places, the
  bones without bodies keeping a reference pose's locals.
- Pose parameters in graphs (mani-0027): `maniGraphParameterPose`, a
  pose of the graph's skeleton the host sets with `maniSetGraphPose`
  (a ragdoll's, say) and reads with `maniGetGraphPose`, the rest pose
  until set; kept in the saved state and checked on restore; written
  and read in the JSON form as `"type": "pose"` with no default.
- Pose input nodes in graphs (mani-0027): `maniGraphNodePose` gives the
  pose its pose parameter holds, so a blend node lays a ragdoll over the
  animation by a weight and a mask node over part of the body; named
  `"pose"` in the JSON form; inside a subgraph it reads the outer
  parameter its own maps to.
- Getting up from a ragdoll (mani-0027): `maniRagdollGetUpPlacement`
  gives the placement on the ground under the root body, turned to look
  where the head lay face down and toward the feet face up, and whether
  the body lies face up, for the host to pick its get-up clip.
- Ragdolls from glTF (mani-0027): `maniGltfReadRagdoll` reads the
  `OMI_physics_body`, `OMI_physics_shape` and `OMI_physics_joint`
  extensions into a ragdoll source: dynamic and kinematic bodies on the
  skeleton's bones with their box, sphere or capsule colliders and
  masses, joints as twist and swing limits with their drives; what a
  profile cannot hold refused at its value with the new
  `maniImportReasonUnsupported`.
- Spring bone nodes in graphs (mani-0026): `maniGraphNodeSpring` turns
  its input's joints of a spring set slot toward their tails, stepped
  when the graph is evaluated by the time its parents gave it, at the
  placement its vec3 and quat parameters give, under the force its
  third vec3 parameter gives; its reset trigger and a seek above it drop
  the motion; its state is saved and restored with
  the instance's and checked; the JSON form names it "spring".
- Spring states (mani-0026): `maniInitSpringState` makes a character's
  spring motion in caller memory; `maniStepSprings` runs the fixed
  steps due toward an animated pose and the character's placement in
  the world and an external force there (wind), all interpolated
  between calls, with colliders, limits
  and center bones (under mirrored bones too); `maniApplySprings`
  turns the joints toward the last step's tails, moved on at their
  velocity between steps; `maniResetSprings`, `maniSaveSpringState`
  and `maniRestoreSpringState` (checked as external bytes) keep and
  move the motion. A call reads each bone once into a work area in the
  state (and a spring node's in the graph instance's room), not saved.
- Task hooks (`maniTaskHooks`, `maniEnqueueTaskFn`, `maniFinishTaskFn`):
  the host's task system, through which the library's parallel work
  will run, results never depending on how the host splits it.
- Root motion taken out at build (mani-0022): a clip source may name a
  bone (`maniRootMotionSource`), whose translation along chosen axes
  and rotation (whole, or the twist about an axis) from its rest, its
  first sample or the origin are kept beside the clip and taken out of
  the bone, optionally looped so the motion ends where it began;
  `maniClipRootMotionSource` reads the source back; compressed clips
  keep it.
- Root motion moves (mani-0022): `maniClipRootMotion` gives the move
  from a playhead time over a signed step, clamped or across any number
  of wraps either way (the period's move composed by squaring);
  `maniComposeRootMotion`, `maniBlendRootMotion` and
  `maniRootMoveIdentity` compose and blend moves.
- `maniRestoreRootMotion` puts a clip's root motion back onto its bone
  in a sampled pose (mani-0022).
- Imported clips may carry root motion and an additive basis
  (`maniImportClipOptions`, mani-0022).
- The library skeleton: the build, the family rules and tools, the
  version and result API (`maniGetVersion`, `maniResultName`) and the
  library profile.
- `maniHash64` and `MANI_HASH_INIT`, the family's frozen 64-bit hash, for
  comparing results between machines.
- Compressed clips (mani-0019): `maniCompressClip` writes a clip of
  default, constant and animated sub-tracks, segments, range reduction
  and per-segment bit rates chosen by a shell error metric; `maniOpenClip`
  and every sampler take either form, `maniSampleClipBones` samples a
  subset, and a forward cursor (`maniClipCursorSize`,
  `maniInitClipCursor`, `maniSampleClipCursor`,
  `maniSampleClipFloatsCursor`) gives random access's bits. An ACL
  differential compares sizes, errors and playback costs.
- Blending (mani-0020): `maniBlendPoses` mixes poses by layer and
  per-bone weights with the rest filling in under a threshold,
  `maniBuildMask` makes per-bone weights from chains, and
  `maniPoseIsDifference` tells a difference from a pose of the skeleton.
- Additive layers (mani-0020): `maniPoseDifference` writes a pose's
  difference from a basis and `maniAddPoses` applies differences at
  layer and per-bone weights.
- Additive clips (mani-0020): `maniClipSource.additive` names a basis
  (the rest or the first frame), sampling such a clip writes a
  difference, compression measures it on the rest, and
  `maniClipAdditiveBasis` reports the basis.
- The ozz differential (mani-0020): `tools/ozz_differential.cc` blends
  the Quaternius clips' poses with masks and additive layers by
  ozz-animation's BlendingJob and by Maul Anim, compares them and times
  both, in the weekly differential workflow.
- Blending weights run from 0 to 1 and are divided by each bone's total
  first, so blends of finite poses stay finite; differences that are
  not finite are refused, and the palette refuses a difference.
- Influences (mani-0021): `maniBuildInfluences` writes a mesh
  primitive's influences bound to a skin, sorted and renormalized in
  groups of four; `maniOpenInfluences` and its queries read them, and
  `maniSkinIdentity` gives the identity they bind to.
- Linear blend skinning (mani-0021): `maniSkinLinear` skins vertex
  ranges of strided position, normal, tangent and further streams, in
  place or not; `maniPaletteRows` writes palettes as row-major 3x4 for
  shaders.
- Dual quaternion skinning (mani-0021): `maniPaletteDualQuaternions`
  splits each palette matrix as rotation times scale and shear,
  `maniSkinDualQuaternion` skins in two phases, and `maniSkinBlended`
  mixes linear and dual quaternion skinning at FBX's per-vertex weights.
- Skinning with optimized centers of rotation (mani-0021):
  `maniBuildCenters` computes each vertex's center offline from the
  rest mesh (Le and Hodgins 2016, Eq. 4, triangles indexed by joint
  pair), and `maniSkinCenters` skins about them without the collapse of
  linear or the bulge of dual quaternion skinning.
- Morph targets (mani-0021): `maniBuildMorphTargets` builds named
  channels of dense or sparse targets (FBX in-betweens included) over
  positions, normals, tangents, texture coordinates and colors,
  `maniOpenMorphTargets` opens them, `maniMorphTargetWeights` turns
  channel weights into target weights, and `maniApplyMorphTargets` adds
  the weighted deltas to strided vertex ranges before skinning;
  `maniMorphAttributeCount` and `maniMorphAttribute` read the table.
- glTF meshes (mani-0021): `maniGltfBuildInfluences` reads a
  primitive's joint and weight sets into influences, refusing what
  glTF forbids (new import reason `maniImportReasonInfluence`);
  `maniGltfBuildMorphTargets` reads its morph targets; `maniGltfMeshCount`,
  `maniGltfPrimitiveCount`, `maniGltfPrimitiveTargetCount`,
  `maniGltfNodeMesh`, `maniGltfNodeSkin` and `maniGltfMorphWeights`
  describe meshes and their starting weights. A mesh's target count is
  now its first primitive with targets', as glTF defines it.
- Morph targets are named (mani-0021): `maniMorphTargetName` and
  `maniMorphFindTarget`; a target without a given name takes its
  channel's, with '/' and its index past the first. Clip weight tracks bind to
  targets by these names.
- FBX influences (mani-0021): `maniFbxBuildInfluences` reads a skin's
  clusters into influences a control point at a time, as ufbx reads
  them; `maniFbxSkinMethod` names the job to run (`maniSkinningMethod`:
  linear, dual quaternion or blended, from SkinningType and the
  per-vertex blend weights).
- FBX blend shapes as morph targets (mani-0021):
  `maniFbxBuildMorphTargets` builds a BlendShape deformer's channels (or
  a 6.x mesh's Shape children) as ufbx reads them, in-betweens and
  negative targets sorted, Blender's per-vertex weights folded in, each
  target named as its clip weight track (a repeated channel name
  numbered); `maniFbxMorphCount`,
  `maniFbxMorphModel`, `maniFbxMorphTargetCount` and
  `maniFbxMorphWeights` describe the sets and their rest weights.

### Changed

- `maniRetargetPose` takes a third fewer instructions (33 thousand
  for a 60-bone humanoid) with the same results, and its scratch is a
  rotation a bone: the pelvis's parent's matrix comes from the locals
  made so far instead of a matrix for every target bone (mani-0032).
- The graph file is format version 2 (mani-0032): its header's last
  word counts map slots and a clip slot's last word names its map
  slot; `maniGraph`, `maniGraphAssets` and `maniGraphClipSource` gain
  their map fields (binding takes a few bytes more).
- `maniApplySprings` (and the spring node) push the tails it moves on
  between steps out of their colliders, as the steps do: at hosts
  faster than the step rate hair no longer passes into a collider by
  up to a step's motion (mani-0026).
- Ragdoll conversions split each bone's matrix without iterating when
  its columns are orthogonal (every matrix but a sheared one) and
  normalize once a result: placing bodies from a pose takes about half
  the instructions, joint targets half, a pose from bodies a fifth
  fewer (mani-0027).
- The ragdoll documentation says what a host passes on a cut (a
  teleport, a switched clip, the first frame), bounds the angular
  velocity at half a turn a step, names each value's units and states
  every limit's range as it is checked (mani-0027).
- `maniStepSprings` and `maniResetSprings` say that the first call of a
  new or reset state takes its force for both ends and that no force
  ramps down over the call (a call of no time stops it at once)
  (mani-0026).
- `maniBindGraph` takes the graph's skeleton, clips, databases and
  models in one `maniGraphAssets`, and `maniBuildGraphJson` its named
  ones in one `maniGraphNamedAssets`, in place of their argument lists.
- A graph update no longer moves the clip nodes no parent reached and
  no sync group holds, which could not move: a state machine of 16
  states costs less than a blend of its clips (mani-0023).
- `maniGltfBuildClip` and `maniFbxBuildClip` take a
  `maniImportClipOptions` (`maniDefaultImportClipOptions`) in place of
  their sample rate and loop arguments; `maniGltfBuildClip` now refuses
  a rate not finite or negative as a misuse.
