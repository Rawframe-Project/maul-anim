# mani-0031. Full-body IK

Status: Accepted

## Context

A character reaching with both hands while its feet stay planted, or
leaning into a push, needs several effectors solved together over a
branching skeleton, with bones that resist turning and joints that
stop. Unreal's Full Body IK node (position-based) gives animators
effector strength, bone stiffness, per-axis limits, preferred angles,
root modes and stretch; selectively damped least squares (Buss and
Kim, 2005) solves every effector in one least-squares step with no
damping constant to tune. Both of the open implementations read for
this record carry choices a stateless library cannot take: Unreal can
start from the previous solve, and Buss's code clamps each effector's
move by the last step's progress.

## Decision

- **One description:** `maniBodySolve` names a top bone; the solved
  part is the top and every bone on a path from it to an effector (at
  most `MANI_BODY_IK_MAX_BONES`, 128). Effectors (at most 32) each name
  a bone, a model-space position and rotation goal, position and
  rotation weights blending the goals from the input, a strength (0
  pulls nothing) and a pull-chain weight. Bone settings, a sparse list,
  give a rotation stiffness (1 locks the bone's local rotation), a
  position stiffness (with stretch), a limit as a chain's (mani-0030;
  the axis is the bone's one child on the part, none for a branch or a
  leaf) and preferred angles. The top is pinned, free or pre-pulled;
  stretch is on or off; an iteration count, a tolerance and a weight
  finish it. The whole description is checked before anything is
  written, and refused whole.
- **From the input, no state:** every call starts from the input pose;
  the caller lends the scratch (`maniBodyIkScratchSize`, 16-byte
  aligned), and nothing is allocated.
- **Before the solve:** preferred angles turn each bone (and the bones
  below it) by its angles times its compression, the most any of its
  effectors' goals has come closer to its origin than the effector's
  input origin, from 0 to 1, so a straight limb folds the chosen way;
  each effector with a pull-chain weight turns its chain (from the
  effector up to the nearest branch, or the top when it is free) about
  the chain's first bone by that share of the shortest arc toward the
  goal; pre-pull moves the top so each stretched effector's goal would
  lie at its reach, by the strength-weighted mean of those excesses
  along the top-to-goal lines, then holds it.
- **XPBD:** `maniSolveBodyXpbd` takes each part bone as a rigid body,
  a unit mass and an inverse inertia of 2 over the squared length to
  its farthest child (a held top fixed). Each pass, in a fixed order:
  effectors by index pull their bodies' origins and rotations with
  compliance (1 - strength) / strength; joints from the tips up hold
  each child's origin at its place on its parent (compliance 0, or 1 -
  position stiffness with stretch), moving both and turning the
  parent; stiff bones' rotations are held to their parents' with
  compliance (1 - stiffness) / stiffness; limits clamp each bone's
  local rotation. Corrections are Müller et al.'s, turns taken whole by
  rotation vectors. Without stretch every origin is placed from the
  turns again after each pass, so what a correction moved without
  turning is dropped and every length is exact. After each pass a
  locked bone takes its parent's turn exactly, so it stays rigid with
  its parent through the passes (its hard stiffness alone splits each
  correction with the bone above, which then turns within the pass),
  and it keeps its local bits.
- **Results:** each bone's local correction from its turn relative to
  its parent's, weighted from the identity as the other solvers';
  translations only for a moved top or stretched bones; "reached" is
  every effector within the tolerance at full weight. A bone whose turn
  relative to its parent is exactly none keeps its bits, so a solve on
  its own reached result changes nothing.

## Consequences

Tests on a figure of hips, spine, chest, two arms and a leg: two
hands reached within 1e-3 with every length kept and the pinned top
held; a solve on its own result unchanged bit for bit; weight 0 and
strength 0 (with a rotation goal too) leaving the pose; a locked
forearm keeping its local bits, its hand reached where the pose puts
it; with the trunk locked, one hand reached while the other is out of
reach, and the solve not reached; a cone on a forearm held, limits
wider than any turn changing no bit; a hand taking its goal rotation;
for a goal out of reach, stretch, a free top and pre-pull each
bringing the hand closer; stretching, a reached hand where the pose
puts it, position stiffness 1 keeping lengths against a compliant
effector; a free top at weight 0.5 moving exactly half as far; a
position weight of 0.5 stopping halfway; a weaker effector stopping
further against stiff bones; an effector on the held top leaving it,
the other effector still reached; a solve short of convergence (every
compliance, two stiff bones, a limit, a rotation weight) pinned bit for
bit with and without stretch, the same again on reused scratch; goals
at the top and nearly every bone locked without a NaN; every refusal.
The steps before the passes, seen through solves of strength 0 (the
passes then move nothing): a pulled chain aimed exactly at its goal
from its first bone, its shape and the other arm kept, half a pull
turning half the angle; preferred angles turning a bone by exactly its
compression times its angles, and not at all for another limb's goal,
a goal farther than the input or a locked bone; pre-pull leaving a top
whose goal is in reach and moving one by exactly the excess past the
reach; a branch bone's twist range unused; a part of 128 bones taken
and one of 129 refused.

On the tests' figure the two-hand solve's worse hand misses by 8.2e-2
after 8 passes, 3.2e-3 after 32, 5.5e-5 after 64 and 1.2e-7 after 128,
where binary32 rounding holds it. Placing the origins again each pass
was the largest gain (after 64 passes with an inverse inertia of 1
over the square: 6.5e-3 without it, 7.9e-4 with); the inverse inertia
of 2 over the square gave the rest (64 passes: 4.9e-3 at 0.5, 4.0e-4
at 3, 2.4e-3 at 4), and running the joints from the tips up a little
(128 passes, without the placing: 1.2e-3 from the top down, 9.8e-4
from the tips up). Strength and stiffness set the pose
the passes converge to; short of it the result changes with the
count, so a caller who raises the count gets a pose nearer the goals.
