# mani-0037. The turn limit node

Status: Accepted

## Context

A pose source can jump: an IK chain's bend plane turns by up to half a
turn between two frames when its pole passes through the line from the
chain's root to its goal, however smoothly the pole moves; a machine
transition cut without a fade, a motion matching jump without an
inertialize node above it and a goal a host teleports do the same. A
host smoothing its goals bounds none of these. Godot 4.7 ships the
remedy as a modifier of its own, `LimitAngularVelocityModifier3D`:
after the modifiers before it, each chosen bone's local rotation turns
from the last frame's toward the new one by no more than a maximum
angular speed times the frame's seconds (a slerp to that fraction of
the angle), starting from the pose when it becomes active. Unity
Animation Rigging's Damped Transform lags a bone behind a source by a
weight instead: no bound, and spring bones (mani-0026) already sway.

## Decision

- **A graph node kind of its own** (`maniGraphNodeTurnLimit`): one
  input, a pose (not differences), given with each bone's local
  rotation turned from the node's last output toward the input's by at
  most its budget; translations and scales pass. IK nodes keep no
  history and stay total (mani-0029 to mani-0031).
- **Bones and weights from mask chains** (mani-0020's masks, optional):
  a bone's weight scales the speed, 0 leaving it free, every bone at 1
  without a mask. Godot's chains and its exclude switch are both a
  choice of chains and a default weight.
- **Speed** a scalar in radians a second, finite and not negative as a
  literal; a parameter's value of 0, below 0 or not a number holds the
  pose.
- **The step,** in binary32 with the library's functions: the relative
  turn `d = p^-1 q` on the shorter arc, its angle
  `a = 2 atan2(|d.xyz|, d.w)`, the budget `b = w s t` (weight, speed,
  the seconds the node's updates gave since its last evaluation, added
  as magnitudes). Within the budget the input's rotation is given as it
  is; otherwise `p (d.xyz sin(b/2) / |d.xyz|, cos(b/2))`, normalized:
  Godot's slerp to `b / a` in exact arithmetic, at about 300
  instructions a bone against the slerp's 540.
- **State** in the instance: the seconds pending and a rotation a bone,
  the last output's, normalized; all zero for a bone held nothing for,
  which takes its input as it is. The node's reset trigger, a seek
  above it and an update that does not reach it drop every bone's;
  under a subset (mani-0035) only the planned bones step, the others
  turning from what they last had when an evaluation next computes
  them. A saved state's block is checked: the time finite and not
  negative, each rotation all zero or finite and of length 1 within
  binary32's drift.
- **The graph's forms:** the file format is version 5 (version 4 files
  open as before; a reader of 4 refuses a file of 5 by version); the
  JSON kind is `turnLimit` with `speed`, `reset` and `mask`; subgraphs
  carry its parameters; the subset planner counts it among the nodes
  that write each bone from the same bone.

## Consequences

A host bounds any jump in a graph by placing one node above the
sources it distrusts, and a bone can be limited to a fraction of the
speed. Measured in a graph frame (GCC 14 `-O3`, 53 bones, two clips
blended: 31,400 instructions), a turn limit node over the blend adds
13,600 with every bone within its budget and 21,482 with every turning
bone limited: 257 and 405 a bone, its copy of the input and its update
included; its rotations are read and written in the pose's memory. Positions are not
limited (a root's travel is root motion's), and limits are local, so a
parent's turn never spends its children's budget.

Tested against binary64 angles: a jump turned by the budget on the arc,
a turn within it and a negated quaternion given bit for bit,
translations passed, weights of 0 and 0.5, a speed of 0 and below,
each reset, a missed update, summed and backward times, a subset, a
saved state restored and its block's checks, the builder's refusals,
opening's checks of the words, the JSON form, subgraphs and a format 4
file; mutants of the shorter arc, the half-angle, the weight, each
reset, the plan and the saved checks are each caught.
