# Inverse kinematics and physics

Inverse kinematics adjusts a pose so bones reach goals: feet on the ground, a hand on a rail, a head following a target. Spring bones sway hair and tails behind the motion. Ragdolls hand the body to the program's physics engine and back. Each solver is a function of a pose and works the same as a graph node.

## Solvers

The solvers work on a pose's locals: they compose the model frames they need themselves, write the locals of the bones they turn and leave the model stale. Goals and poles are in the pose's model space.

```c
// The hips, spine and head as a chain bending about the spine's z axis,
// its end onto a goal in model space.
const maniTwoBoneSolve reach = {.start = 0,
                                .middle = 1,
                                .end = 2,
                                .goal = {0.3f, 1.5f, 0.0f},
                                .hinge = {0.0f, 0.0f, 1.0f},
                                .soften = 1.0f,
                                .weight = 1.0f};
bool reached = false;
maniResult solved = maniSolveTwoBone(pose, &skeleton, &reach, &reached);
```

| Solver | For |
|---|---|
| `maniSolveTwoBone` | legs and arms: a two-bone chain bent toward a goal, its knee or elbow toward a pole |
| `maniSolveAim` | heads, eyes, weapons: a bone, or a chain of up to `MANI_AIM_MAX_BONES` sharing the turn, aimed at a goal, within an optional cone |
| `maniSolveCcd`, `maniSolveFabrik` | tails, spines, tentacles: chains of up to 32 bones, with cone, twist and hinge limits per bone |
| `maniSolveBodyXpbd` | a whole body: several effectors with goals at once, bones with stiffness and limits, the top pinned or free; fast enough for every frame |
| `maniSolveBodySdls` | the same description solved by damped least squares; for small parts, few iterations or work off the frame |

A pole never makes the chain flip as it crosses the goal line, and a solve on its own reached result changes nothing. Each solver takes a weight that blends its correction from the input.

Full-body solves take a description (`maniBodySolve`: a top bone, effectors with goals, weights and strengths, per-bone settings) and scratch memory of `maniBodyIkScratchSize` bytes. A 22-bone humanoid with hands, feet and head as effectors takes about 36 microseconds at 8 XPBD iterations on a desktop core; the same with SDLS takes over a millisecond.

The graph's IK nodes read their goals from vec3 and quat parameters the program sets from its world each frame. A turn limit node above them bounds how fast bones may turn, which hides the snap when a pole or goal jumps.

## Spring bones

Spring bones (hair, tails, cloth strips, the bones VRM avatars carry as `VRMC_springBone`) are a spring set built for a skeleton (`maniBuildSpringSet`, or read from a glTF file with `maniGltfReadSprings`) and a state in the program's memory (`maniSpringStateSize`, `maniInitSpringState`). Each frame `maniStepSprings` moves them against the pose's model and the character's placement in the world, and `maniApplySprings` writes them into the pose; a graph's spring node does both.

The set steps at a fixed rate, so the motion is the same at any frame rate; a set's reference rate (60 Hz by default) is the rate its content was tuned at. A wind or other force can be passed each call. The state saves and restores (`maniSaveSpringState`, `maniRestoreSpringState`); `maniResetSprings` drops the motion after a teleport.

## Ragdolls

A ragdoll profile maps bones to the bodies and joints of the program's physics engine: shapes, masses, swing-twist limits and drives. The library does no physics; it converts between poses and bodies.

- `maniRagdollBodiesFromPose` gives the bodies' placements, and with the previous pose their velocities, so an engine's kinematic bodies follow the animation.
- `maniRagdollJointTargets` gives powered joints an animation to follow.
- `maniRagdollPoseFromBodies` reads the pose back from the simulated bodies.
- `maniRagdollGetUpPlacement` places a get-up animation where the body fell, facing the right way.

A graph blends the physics pose with animation through a pose parameter (`maniSetGraphPose`) feeding a blend (full ragdoll) or a mask node (a partial ragdoll, such as an arm hit). Profiles are built from a source (`maniBuildRagdoll`) or read from glTF's `OMI_physics_*` extensions (`maniGltfReadRagdoll`).

## Not obvious

- Solvers leave the model stale; call `maniComputeModel` before reading model matrices or computing a palette.
- Masses and drive strengths pass through in the engine's own units; lengths are the skeleton's.
- On a teleport or a camera cut, pass the current placement as the previous one to `maniRagdollBodiesFromPose` (no velocity spike) and reset the springs.
