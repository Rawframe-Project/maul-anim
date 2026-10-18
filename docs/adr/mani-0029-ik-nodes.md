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

## Consequences

Tests on a leg whose end is two bones below the knee: goals within
reach reached with the lengths kept and the knee toward the pole, a goal
out of reach straight toward it and a softened one short of it, a solve
on its own result unchanged, a pole swept through the goal line moving
the knee in small steps (with the blend off the same sweep jumps), the
hinge given or the bend plane's alike, a straight leg without hinge or
pole bending to reach, weight 0, a goal at the hip, the refusals.

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
toward) are left out of the comparison.
