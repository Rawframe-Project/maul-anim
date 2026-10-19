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
- **SDLS:** `maniSolveBodySdls` takes, each iteration, the Jacobian
  of every effector's position and, with a rotation weight, its
  rotation as a length (the rotation vector of its miss times its reach
  from the top, the summed offsets, no less than the mean offset), by
  three model axes at each part bone's origin (and three translations
  for a free top); rows scaled by strength, columns by one minus
  rotation stiffness (none for a locked bone or a held top), each miss
  clamped to half the mean bone length. Buss and Kim's step with the
  paper's hard clamps: for each singular triple, gamma_i = min(1, N_i /
  M_i) pi/4 bounds the largest turn of alpha_i / sigma_i v_i, and the
  sum's largest turn is bounded by pi/4 (translations unclamped, as the
  paper's translational joints). Buss's code instead scales every
  direction smoothly, never leaving one whole. The step is taken from
  the tips up, each bone's turn turning the bones below it, so a locked
  bone stays rigid with its parent exactly; limits follow from the top
  down, a limited bone's change carried to the bones below it. Stretch
  is refused: the solve keeps the part one rigid hierarchy.
- **The decomposition:** one-sided Jacobi (Hestenes) on the Jacobian's
  transpose, whose few columns are the effectors' rows: pairs rotated
  by Rutishauser's tangent, dot products in binary64, a pair skipped
  when its cosine is below sqrt(n) times binary32's unit roundoff, the
  sweeps stopping when one rotates nothing or at 30, as LAPACK's
  `sgesvj`. Each column's squared length is kept and taken again only
  after a rotation changes the column, so it is what a fresh dot
  product gives, and a pair with a zero column (the rotation rows of an
  effector without a rotation weight) is skipped without a product. Every operation is an addition, a multiplication, a
  division or a square root, each correctly rounded, in a fixed order
  with contraction off, so the stop falls on the same sweep everywhere.
- **Results:** each bone's local correction from its turn relative to
  its parent's, weighted from the identity as the other solvers';
  translations only for a moved top or stretched bones. An effector's
  miss is the larger of its distance from its goal and, wanting a
  rotation, its rotation's angle from the goal's times its reach (the
  summed offsets from the top, no less than the mean offset: the
  length SDLS scales rotation rows by); both solvers stop on it, and
  "reached" is every miss within the tolerance at full weight. Found by
  a free top as the effector, its position met and its rotation not:
  a stop on distance alone never ran an iteration. A bone whose turn
  relative to its parent is exactly none keeps its bits, so a solve on
  its own reached result changes nothing.
- **Graph nodes:** XPBD (22) and SDLS (23), one input each, a pose
  (differences refused at build and on open). Each effector's position
  goal is a vec3 parameter and its rotation goal a quat parameter or
  none, its weights numbers or float parameters clamped to [0, 1] when
  evaluated (an effector without a rotation keeps its rotation weight
  the number 0, so it has one form in a file); the top, mode, stretch,
  iterations, tolerance, strengths, pull chains and settings are fixed
  in the node, the node's weight a number or a float parameter. The
  build checks the whole description as the solve would on the
  skeleton, with the goals in place; opening, which has no parents,
  checks every word in range (a setting's limit as a chain node's) and
  a crafted file whose bones make no part passes its input. The node
  lays its effectors, its settings and the solve's scratch in the
  instance's room from a 16-byte boundary; the room is sized at
  instancing from the part the top and the effectors' bones make, so
  evaluation allocates nothing. The JSON form names bones and
  parameters by name, the mode as "pinned", "free" or "prePull", and
  settings' limits as a chain node's.

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

SDLS's tests on the same figure: two hands reached where the pose
puts them, lengths kept, the top held; a solve on its own result
unchanged; weight 0 and strength 0 leaving the pose; a locked forearm
kept, its hand reached; a cone held; a hand at its goal taking its
rotation; a free top bringing an out-of-reach hand closer; stretch
refused; goals at the top without a NaN; every bone locked keeping the
pose; a solve short of convergence pinned bit for bit. One iteration
on the left arm (three turning bones, a clamped miss) puts the hand
within 1.1e-7 of a binary64 SDLS written in the test from the paper;
with the arm hinged about z, an elbow pulled up and a hand pulled down
end at squared misses of 1.28367e-3, where a search over both hinge
angles finds 1.28367e-3 (the solve 3e-6 lower, finer than the search's
grid). The decomposition on its own: random matrices of 9 by 4 to 120
by 36 rebuilt from their factors and orthogonal within 2e-6, singular
values within 2e-6 of binary64 Jacobi's, repeated and zero columns
giving zeros without a NaN, a fixed matrix's 6 sweeps and bits pinned.

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

Timings (one core of the shared 24-core machine, gcc 14 -O3, the median
of 101 solves from the input; instruction counts, which the machine's
load leaves alone, first). The tests' figure, two hands: XPBD 21.8
thousand instructions (4.1 microseconds) at 1 iteration, 197 thousand
(43 microseconds) at 32; SDLS 116 thousand (14 microseconds) at 1, 1.31
million (157 microseconds) at 32, where it has stopped. A humanoid
(a 22-bone part from the hips, hands, feet and head as effectors, three
with rotations; 16,560 bytes of scratch): XPBD 57 thousand
instructions at 1 iteration, 165 thousand (36 microseconds) at 8, 532
thousand (117 microseconds) at 32; SDLS 1.8 million an iteration, 13.8
million (1.2 to 1.4 milliseconds) at 8. The worst description (128
bones, 32 effectors, 478,896 bytes of scratch): XPBD 493 thousand
instructions at 1 iteration, 1.8 million (0.36 milliseconds) at 16,
22.5 million (4.6 milliseconds) at 256; SDLS 54 million (5.4
milliseconds) an iteration without rotations, 1.3 seconds at 256, and
4.7 billion at 16 with every rotation row. Keeping the columns' lengths
took SDLS from 304 thousand to 116 thousand instructions an iteration
on the figure, 4.4 million to 1.8 million on the humanoid and 496
million to 54 million on the worst case (where half the rows are zero),
every bit the same; four running sums in the dot product saved under
2% of the instructions and were dropped.

The budget: a humanoid solved in 8 iterations within 0.5 milliseconds
on one core. XPBD meets it about fourteen times over, and its worst
description stays under 5 milliseconds even at 256 iterations. SDLS
misses it (an iteration of the humanoid takes about 0.2 milliseconds);
its iterations are not capped by the description's size, which would
make the count the caller gives mean less than it says, so its
documentation states its cost and the count stays the caller's budget:
SDLS for small parts, a few iterations or work off the frame, XPBD for
the frame.

The nodes' tests over the hips, spine and arm: each node's output its
input solved by its solver bit for bit, with two effectors (one with a
rotation weighted by a parameter, one without a rotation), a setting
with a cone and stiffness, a free top, a goal moved, weights clamped
from 4 and -1; both inside a subgraph reading the outer goal and
weight; the JSON form read back to the same file, a rotation weight
without a rotation, an unknown mode and settings not a list refused;
every build refusal with its reason (a scalar's at its place) and
every file refusal at its word, counts that do not match the node's
words refused there; a crafted top above no effector and an effector's
bone twice passing the input, a crafted free top still solving.
