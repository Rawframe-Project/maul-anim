# mani-0030. CCD and FABRIK chains with joint limits

Status: Accepted

## Context

Tails, spines, tentacles and fingers want a chain of more than two
bones bent toward a goal, and a knee or an elbow wants its bend held to
one axis and a range. Godot 4.6 (`CCDIK3D`, `FABRIK3D`), Unity's Chain
IK and Unreal's CCDIK and FABRIK nodes name a chain by its root and end
bones and stop at a tolerance or an iteration count; Godot limits bone
directions only, Unreal's CCDIK one angle per joint, Unity and Unreal's
FABRIK nothing. Godot's solvers stay steady by carrying the last
frame's result, which a stateless solve cannot.

## Decision

- **A chain by its ends:** `maniChainSolve` names a root and an end,
  an ancestor of it, every bone of the path between in the chain (at
  most `MANI_IK_CHAIN_MAX_BONES`, 32, the end included); the bones from
  the root to the end's parent turn so the end's origin reaches the
  goal. The solvers work on locals by the root path, as the two-bone
  solve's (mani-0029).
- **CCD:** `maniSolveCcd` sweeps from the end's parent to the root each
  iteration (Wang and Chen's order, n turns a sweep), each bone turned
  by the shortest arc taking its direction to the end onto its
  direction to the goal, cut to the step bound, then limited; the
  frames below are recomposed before the next bone. A bone whose
  origin lies on the end or the goal turns nothing.
- **FABRIK:** `maniSolveFabrik`'s backward pass places the end on the
  goal and each joint at its bone's length from the one below it; its
  forward pass, from the root, turns each bone by the shortest arc from
  its direction onto the one toward its joint's backward place (its
  twist kept), limits it and recomposes the frames below, so the next
  joint stands where the turned bone puts it and the end agrees with
  the rotations after every iteration. Limits act in the forward pass
  only, where the parent's frame is final. An optional pole turns the
  chain about the root-to-goal line first, so its bend (its middle
  joints' offsets from that line) faces the pole, blended near the line
  as the two-bone solve's.
- **Stop:** an iteration count (1 to 256) and a tolerance, checked
  before each sweep; nothing carries between updates.
- **Step bound:** each turn of a bone in a sweep cut to `step` radians
  (0 for none), the damping that keeps CCD's first turns near the end
  from curling the chain, stateless where Godot's comes from the last
  frame.
- **Joint limits:** `maniJointLimit` bounds a bone's local rotation
  relative to its rest local (the rest's rotation then a turn r), r
  split into a swing and a twist about the bone's axis, the next bone's
  rest offset: a cone bounds the swing's angle and the twist to a
  range; a hinge keeps only r's turn about its axis, within its range.
  A turn within its limit comes back unchanged. A bone whose next bone
  lies at its origin has no axis; its whole turn is its swing.
- **The end's rotation:** a wanted model rotation for the end bone,
  taken by `rotationWeight` once the chain is solved, as Unity's tip
  rotation weight and Unreal's effector rotation source.
- **Weight:** every bone's correction from its input local weighted
  from the identity, as the other solvers'.

- **Graph nodes:** CCD (20) and FABRIK (21), one input each, a pose
  (differences refused at build and on open). The goal and a FABRIK
  node's pole are vec3 parameters, the end's rotation a quat parameter
  (each pole and rotation optional); the iteration count, tolerance,
  step and limits are fixed in the node; the rotation weight and weight
  are numbers or float parameters, clamped to [0, 1] when evaluated. A
  node holds one limit per turning bone or none, each with the words
  its kind leaves unused 0, so a limit has one form in a file. The build
  checks the chain and its limits against the skeleton; opening, which
  has no parents, checks each limit as a solve would and the bones in
  range, and a crafted file whose chain is not one passes its input.
  The JSON form names bones and parameters by name and writes each limit
  as an object of its kind (none, a cone with its twist, a hinge with
  its axis and range).

## Consequences

Tests on a tail of five bones: goals within reach reached within the
tolerance with every length kept; a goal out of reach straightens the
chain toward it; cones, twists and hinges never exceeded, read back from
each bone's turn from its rest (hinged bones staying in their plane),
also on a tail along x whose bones rest turned; limits of no kind
solving as none bit for bit; the step bound bounding each bone's turn
in a sweep; a solve on its own reached result unchanged bit for bit;
the end taking the wanted rotation and half of it; weights 0 and 0.5; a
goal at the root without a NaN and the root unturned; a goal within the
tolerance leaving the pose bit for bit, one at the end reached with
tolerance 0; paths of 32 bones solved and 33 refused; every refusal.
The limits on their own: a turn within its limit unchanged bit for bit,
cones and hinges cutting as stated, a turn's negation limited as the
turn, a half-turn swing taken whole as the swing, turns just short of
it approaching that result, a bone of no axis ignoring its twist
range.

FABRIK's tests on the same tail: goals reached with lengths kept; the
joints' places after 1, 2 and 8 iterations within 2e-6 of a FABRIK in
double precision written in the test from the paper; a goal out of
reach straightening the chain; a pole on either side bending the chain
toward it, its turn coming before the first pass; a bone of length 0
left as it was; cones never exceeded; hinges that make the goal unreachable
running all 64 iterations with tolerance 0, the result finite, within
the hinges and its bits pinned (the same in every build and on every
platform CI runs); a solve on its own reached result unchanged; the
end's rotation; weight 0; the refusals.

The nodes' tests over the hips, spine and arm: each node's output its
input solved by its solver bit for bit, with a cone and a hinge, the
end's rotation by a weight parameter, weights clamped from 4 and -1, a
pole; both inside a subgraph reading the outer goal and weight; the
JSON form read back to the same file and a limit list of the wrong
length refused; every build refusal with its reason and every file
refusal at its word, 32 limits refused by the node's word count; a
crafted chain passing its input, and a crafted chain whose limits are
not one per turning bone of the bound skeleton's path passing it too
(the evaluator checks the count against the path before it reads a
limit).

The IK differential times both chains on the start, middle and end
(a three-bone chain from rest with the middle bent 0.8 radians, 256
random goals within reach, 16 iterations at most, tolerance 1e-4): a
CCD solve takes 3600 ns and a FABRIK solve 3800 ns (clang 20, -O3, one
core of the test machine under a load average near 170), each about 8
to 9 iterations on average, about 400 ns an iteration, near a two-bone
solve's 490 ns. ozz has no chain solver to time against.
