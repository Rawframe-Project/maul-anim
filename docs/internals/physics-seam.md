# Spring bones and the ragdoll seam

Two kinds of secondary motion meet animation here. Spring bones (hair, tails, cloth strips) are simulated by the library itself, as VRM defines them. Ragdolls are simulated by the host's physics engine; the library gives the seam between that engine and its poses, and never simulates bodies.

## Spring bones

A spring set (kind 11) is built for a skeleton from `VRMC_springBone-1.0` sources ([gltf-import.md](gltf-import.md#spring-bones-and-ragdolls)) or code: chains of joints, colliders (spheres, capsules, planes, inside spheres and capsules, as the extended collider extension adds), center bones, and per-joint limits (cone, hinge, spherical, from the limit extension). The builder refuses what VRM forbids: a chain whose joints are not each below the last, a joint in two chains, a center neither the first joint nor above it.

### Fixed steps, the same motion at any frame rate

VRM's reference integrates once a rendered frame: inertia carries the last frame's displacement whole while stiffness and gravity scale with the frame's time, so the motion changes with the frame rate, and content is tuned at whatever rate its author previewed. Here a set has a reference rate (60 Hz by default; the rate the content was tuned at) and a step rate, and the builder converts each joint's per-frame drag, stiffness and gravity to a step of h seconds: drag `1 - (1 - d)^(h f0)`, stiffness and gravity times `h (dh / d)`. That conversion is the exact solution, over each reference frame, of the equation the reference integrates, so the free motion is the same at any step rate and the reference's own at the default.

`maniSpringState_Step` ([`src/spring_step.c`](../../src/spring_step.c)) runs the steps due at the step rate, at most `MANI_SPRING_MAX_STEPS` a call (a longer delta loses the rest, as fixed-step physics does). Each step's inputs (heads, colliders' bones, centers, the placement, an external force such as wind) are interpolated between the last call's and this one's at the step's time, so the motion does not depend on how the host splits time. Within a step, per chain from the root: inertia, drag, stiffness toward the pose's direction, gravity and force, then the bone's length, the limit, and each collider's push followed by the length and limit again, in VRM's order.

`maniSpringState_Apply` turns each joint toward the last step's tails moved on at their velocity for the time since that step: exactly the step's result at host rates the step rate divides, continuous across steps at faster hosts, and never a step behind (interpolating between the last two steps would lag and point tails behind moving heads).

The state (`maniSpringSet_GetStateSize`) is words in caller memory without pointers, so a graph instance can hold it and a save copies it. A graph's spring node does a step and an apply on its input pose each evaluation, bit for bit as the standalone state over the same poses and deltas.

## The ragdoll seam

A ragdoll profile (kind 12, [`src/ragdoll.c`](../../src/ragdoll.c)) maps bones to physics bodies the way Jolt, Unreal, Godot and the `OMI_physics_*` glTF extensions all do: bodies on a subset of the bones, each with a frame relative to its bone, a primitive shape (sphere, capsule, box) and a mass; a swing-twist joint to the parent body (twist range, an elliptic swing cone) and an optional drive (stiffness, damping, largest torque). Joint frames are computed from the rest pose at build, so every joint is at rest in the rest pose. Lengths are the skeleton's units; masses and drives are the host engine's units, passed through, never converted, because converting them needs inertia the engine derives itself.

The calls the seam needs, and nothing more:

- `maniRagdoll_ComputeBodiesFromPose`: each body's world frame from the pose's model and the character's placement, and with the previous pose its linear and angular velocities, which an engine's kinematic bodies and motors take. On a cut the host passes what the step means: no previous pose (bodies at rest), the current placement as the previous one on a teleport (no velocity spike).
- `maniRagdoll_ComputeJointTargets`: each joint's child frame in its parent's and the relative angular velocity, the local motor targets of Jolt's and Unreal's swing-twist constraints.
- `maniRagdoll_ComputePoseFromBodies`: a pose back from the bodies; bones without a body keep a reference pose's locals.
- `maniRagdoll_ComputeGetUpPlacement`: where a get-up animation stands, from the root body and a ground the host names: face up or down by the chest's facing against the ground normal, the heading from the head-to-feet axis blended with the facing by how upright the body stands, which is continuous through upright where either alone is undefined.

A graph blends physics over animation without a ragdoll node: a pose parameter (`maniGraphInstance_SetPose`) holds the engine's pose and a pose input node feeds it to a blend (full ragdoll) or a mask (partial). The pose node holds no time, so it leads no sync group, fires no events and gives no root motion; the ragdoll's placement is the host's.

## Traps

- The spring conversion is exact only for the free motion; colliders and limits act once a step, so a tail may pass a limit by up to a step's motion until the next step.
- Spring tails keep in the center bone's space or in model space as the last step placed them, carried into each step's placement, so moving the character sways them; a teleport wants the spring node's reset trigger.
- An angular velocity from two body rotations is the shorter turn, at most half a turn a step; a bone turning faster reads as turning the other way.
- A ragdoll profile is tied to its skeleton's identity and rest pose; a skeleton with another rest pose needs its profile rebuilt.
