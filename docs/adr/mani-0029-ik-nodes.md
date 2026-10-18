# mani-0029. Two-bone, aim and look-at

Status: Accepted

## Context

Feet planted on uneven ground, a hand on a rail and a head following a
target are the inverse kinematics nearly every character needs: a
two-bone chain bent to reach a goal, and a bone or a short chain turned
to aim at one. ozz-animation's `IKTwoBoneJob` and `IKAimJob` are the
reference most hosts tune against; their users reported two gaps, the
chain flipping as the pole crosses the goal line (ozz #187, left to the
caller in its header) and results changing when a job runs again on its
own output (#172). ozz takes model matrices; a graph node here holds
local poses.

## Decision

- **Solvers on locals:** `maniSolveTwoBone` reads a pose's locals,
  composes the model frames of the end bone's root path itself (no full
  model pass, the pose's model need not be current), writes the start
  and middle bones' corrected locals and leaves the model stale. A
  model-space turn T of a bone under a parent of model rotation P is
  the local correction P* T P, mirrored through x under a mirrored
  parent.
- **Frames:** the goal and the pole are in the pose's model space; the
  hinge is fixed to the middle bone and given in its local frame (a
  positive turn about it opening the chain), as ozz takes it. Without a
  hinge, the chain's own bend plane; a straight chain without one bends
  in the pole's plane, without a pole about the middle bone's x (or z).
- **The two turns:** the middle turns about its hinge by the wanted
  angle less the current one, the wanted angle by the law of cosines;
  the start turns the bent chain onto the goal by the shortest arc,
  then about the goal line so the bend lies in the plane of the goal
  line and the pole, then by the twist. Angles stay as cosines and
  sines: the wanted one from products of the lengths' sums and
  differences, the current one from the chain's own dot and cross
  products, every half angle from whichever of the two keeps its
  digits. No `acos`.
- **The pole never flips:** with s the sine of the pole's angle to the
  goal line, the bend plane is the pole's for s at least 0.05 and
  turns linearly back to the chain's own as s falls to 0; no pole is
  s = 0. A function of the inputs alone.
- **Softening, reach and weight as ozz's:** past a share `soften` of the
  chain's length a goal is reached only gradually, on ozz's curve;
  `reached` is a goal within that share and further than the bones'
  difference, at full weight; each correction (w at least 0) is lerped
  from the identity by the weight and normalized.
- **Edges stated:** a goal out of reach straightens the chain toward
  it, one closer than the bones' difference folds it, a goal at the
  start turns nothing at the start, a bone of no length turns nothing
  at the middle; a solve on its own result changes nothing.
- **Aim:** `maniSolveAim` takes the aiming bone's forward (unit), an
  offset point and an up axis in its local frame, and a model-space
  goal and pole. As ozz's: the forward line through the offset point is
  moved so the point aims at the goal (the forward replaced by the
  vector to where that line meets the sphere of the goal's distance;
  no turn when the offset lies further from the line than the goal),
  turned onto the goal by the shortest arc, then about the aim line so
  the up axis lies in the plane of the line and the pole (with the
  two-bone solve's blend near the line), then by the twist.
- **Look-at over a chain:** up to 8 bones, an ancestor first and the
  aiming bone last, with shares: each bone, root first, turns its share
  of the aim still needed from it down (a slerp from the identity),
  recomputed once the bones above have turned; the last turns what is
  left, so the chain ends aimed whatever the shares.
- **Cone:** a half angle in (0, pi] bends the goal onto the cone's edge
  about the input pose's forward line before the aim; `reached` is then
  false.
- **Weight:** each correction computed at full weight on the pose as
  the chain turns, then each weighted on the input pose.

## Consequences

Tests on a leg whose end is two bones below the knee: goals within
reach reached with the lengths kept and the knee toward the pole, a goal
out of reach straight toward it and a softened one short of it, a solve
on its own result unchanged, a pole swept through the goal line moving
the knee in small steps (with the blend off the same sweep jumps), the
hinge given or the bend plane's alike, a straight leg without hinge or
pole bending to reach, weight 0, a goal at the hip, the refusals.

Aim tests on a spine, neck and head: goals all around reached with the
up axis toward the pole, an eye offset's line through the goal, the
turn spread over three bones by shares (and by 1, 0, 0) ending aimed
with each bone turned, a cone stopping the forward at its edge, a pole
swept through the aim line turning the up axis continuously, weight 0,
an aim on its own result unchanged, goals at the head or nearer than
the eyes' line, the refusals.

The differential (`tools/ik_differential.cc`, weekly in CI) solves a
million random chains with ozz 0.17.0's `IKTwoBoneJob` and with this
library, poles off the goal line, goals within reach and past it,
softening, weights and twists: the bones' positions agree within 2.8e-4
of the chain's length (1.1e-3 softened, where ozz uses 12-bit
reciprocal estimates), and "reached" always. Each solver's own error is
measured too: on reachable goals the end misses by 6.5e-7 of the
chain's length here and 4.2e-5 in ozz, and the middle lies off the
pole's plane by 1.7e-6 here and 1.2e-4 in ozz, which turns to the plane
from the cosine alone. Weighted corrections of nearly half a turn (w
near 0, which way a weight turns being rounding's in both) and goals
within a twentieth of the chain's length of the start (which ozz turns
toward) are left out of the comparison. It aims a million random
bones too, against ozz's `IKAimJob` (offsets, twists, weights; poles and
up axes off the aim line): the turned rotations agree within 1.5e-3
radians, ozz's own plane error; on its own measures the forward line
misses the goal by 8.6e-7 of its distance here and 5.1e-5 in ozz, and
the up axis lies off the pole's plane by 1.7e-6 here and 4.3e-4 in ozz.
ozz weights an aim by lerping from the identity without first making w
positive (unlike its two-bone job), so a weighted turn with w below 0
goes the long way there; the comparison weights ozz's full turn as its
two-bone job does.
