# Inverse kinematics

Feet on uneven ground, a hand on a rail and a head following a target need a two-bone solve or an aim on nearly every character; tails and spines need longer chains with joint limits; a character reaching with both hands while its feet stay planted needs full-body IK. Every solver here works on a pose's locals, is stateless, and is also a graph node.

## Shared rules

- **Locals in, locals out.** A solver reads the pose's locals, composes the model frames of the bones on its root path itself (the pose's model need not be current, and no full model pass runs), writes the corrected locals of the bones it turns and leaves the model stale. A model-space turn T of a bone under a parent of model rotation P is the local correction P* T P, mirrored through x under a mirrored parent.
- **Stateless.** The result is a function of the pose and the inputs. Godot's solvers stay steady by carrying the last frame's result; a solve here cannot, so damping comes from step bounds and blends, and a node that must bound turns over time is the turn limit ([graphs.md](graphs.md#other-node-kinds)).
- **Idempotent.** A solve on its own reached result changes nothing, bit for bit: ozz-animation's users reported its jobs drifting when run on their own output.
- **Weights.** Each correction is computed at full weight, then lerped from the identity by the weight (its w made non-negative first) and normalized.
- **Frames without decomposition.** While every scale on the path is exactly 1 (most rigs), frames compose as a rotation and an origin and a bone's model rotation is read off directly; any other scale switches the path to affine matrices and the rigid decomposition.
- **Goals and poles** are in the pose's model space; axes fixed to a bone are in its local frame.

## Two-bone and aim

`maniSolveTwoBone` ([`src/ik.c`](../../src/ik.c)) turns the middle bone about its hinge by the wanted angle less the current one (law of cosines), then the start bone onto the goal by the shortest arc, then about the goal line so the bend faces the pole, then by the twist. Angles stay as cosines and sines throughout, each half angle taken from whichever of the two keeps its digits; there is no `acos`, which loses precision near straight and folded chains.

The pole never flips. With s the sine of the pole's angle to the goal line, the bend plane is the pole's for s at least 0.05 and turns linearly back to the chain's own as s falls to 0. ozz leaves this to the caller, and a pole sweeping through the goal line there flips the knee. Softening, reach and "reached" follow ozz's definitions, so hosts tuned against ozz keep their numbers.

`maniSolveAim` ([`src/ik_aim.c`](../../src/ik_aim.c)) aims a bone's forward axis through an offset point (an eye) at a goal, its up axis toward a pole with the same blend near the aim line. Over a chain of up to `MANI_AIM_MAX_BONES` (a look-at over spine, neck and head), each bone from the root turns its share of the aim still needed from it down, recomputed after the bones above turned; the last turns what is left, so the chain ends aimed whatever the shares. An optional cone bends the goal onto the cone's edge first.

Measured against ozz-animation 0.17.0 on a million random chains, the bones agree within ozz's own estimate error, and on its own measures the solve here misses the goal by 6.5e-7 of the chain's length where ozz misses by 4.2e-5. A solve takes more instructions than ozz's job alone because it composes the root path from locals; against ozz's whole cost to a host (its job on model matrices it needs computed first), the two-bone solve is about 1.1 times and the aim about 2.3 times, the rest being the binary64 arc that keeps turns exact near opposite directions.

## Chains and joint limits

`maniSolveCcd` and `maniSolveFabrik` ([`src/ik_ccd.c`](../../src/ik_ccd.c), [`src/ik_fabrik.c`](../../src/ik_fabrik.c)) bend a chain named by its root and end (at most `MANI_IK_CHAIN_MAX_BONES`, 32) toward a goal, stopping at an iteration count or a tolerance checked before each sweep.

- **CCD** sweeps from the end's parent to the root, each bone turned by the shortest arc from its direction to the end onto its direction to the goal, cut to a step bound, then limited, the frames below recomposed before the next bone. The step bound is the stateless damping that keeps CCD's first turns near the end from curling the chain.
- **FABRIK**'s backward pass places the end on the goal and each joint at its bone's length from the one below; its forward pass, from the root, turns each bone onto its backward place (twist kept), limits it and recomposes the frames below. Limits act only in the forward pass, where the parent's frame is final, so the end always agrees with the rotations. An optional pole turns the chain's bend toward it first.

A joint limit (`maniJointLimit`, [`src/ik_limit.c`](../../src/ik_limit.c)) bounds a bone's rotation relative to its rest by swing and twist about the bone's axis (the next bone's rest offset): a cone bounds the swing and a range the twist, or a hinge keeps only the turn about its axis. A turn within its limit comes back unchanged, bit for bit. A wanted end rotation can be taken by a weight once the chain is solved, as Unity's tip rotation weight.

## Full-body IK

`maniBodySolve` describes a part: a top bone and every bone on a path from it to an effector (at most 128 bones and 32 effectors). Effectors have position and rotation goals, weights and a strength; bones have rotation stiffness (1 locks), position stiffness (with stretch), limits as chains have, and preferred angles. The top is pinned, free or pre-pulled. Every call starts from the input pose; the caller lends scratch (`maniBodyIkScratchSize`). The whole description is checked before anything is written.

Two solvers take the same description:

- **XPBD** (`maniSolveBodyXpbd`, [`src/body_xpbd.c`](../../src/body_xpbd.c)): each part bone is a rigid body of unit mass and an inverse inertia of 2 over the squared length to its farthest child. Each pass, in a fixed order, effectors pull with compliance from their strength, joints from the tips up hold each child's origin on its parent, stiff bones hold their rotations to their parents', and limits clamp. Without stretch every origin is placed again from the turns after each pass, so lengths are exact. Placing the origins again was the largest gain in convergence, the inertia constant the next.
- **SDLS** (`maniSolveBodySdls`, [`src/body_sdls.c`](../../src/body_sdls.c)): Buss and Kim's selectively damped least squares over the Jacobian of every effector's position and rotation, with the paper's clamps per singular direction. The decomposition ([`src/jacobi_svd.c`](../../src/jacobi_svd.c)) is one-sided Jacobi with dot products in binary64 and the stopping rule of LAPACK's `sgesvj`; every operation is correctly rounded in a fixed order, so the stop falls on the same sweep everywhere. Stretch is refused: the solve keeps the part one rigid hierarchy.

XPBD is the one for every frame: a 22-bone humanoid with hands, feet and head as effectors takes about 165 thousand instructions at 8 iterations, well inside a frame. SDLS takes about 1.8 million instructions an iteration for the same part; it suits small parts, few iterations, or work off the frame. Its iteration count is not capped by the description's size, so the count stays the caller's budget and the header states the cost.

## Graph nodes

Two-bone, aim, CCD, FABRIK, XPBD and SDLS are graph node kinds with one pose input (differences are refused). Goals and poles are vec3 parameters, end rotations quat parameters, weights numbers or float parameters clamped when evaluated; chains, limits and settings are fixed in the node. The builder checks each chain against the skeleton; a file holds no parents, so opening checks only that bones are in range, and a crafted file whose chain is not one passes its input unsolved. Each node's output is its solver's result on its input, bit for bit.

## Traps

- Turns use a binary64 shortest arc because a binary32 half-vector arc misses by up to 2.4e-4 within a degree or so of opposite directions, wherever its switch to a perpendicular axis is placed.
- XPBD short of convergence gives a pose that depends on the iteration count; raising the count moves it nearer the goals.
- A locked bone keeps its local bits in both solvers; a change that turns locked bones by rounding breaks that test.
