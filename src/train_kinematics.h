// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Character space from a frame's Decompressor outputs, and its backward
// pass: each bone's rotation from its two columns (the first
// normalized, the third the normalized cross of it and the second, the
// second their cross), its scale given or kept, its model matrix down
// the hierarchy (the parent's times the bone's rotation, scale and
// translation, as poses compose); then each bone's model translation from
// bone 0's and its first two model columns, turned by bone 0's rotation
// undone, as the training data measures them. Binary32, in bone order
// forward and the reverse backward.

#ifndef MAUL_ANIM_SRC_TRAIN_KINEMATICS_H
#define MAUL_ANIM_SRC_TRAIN_KINEMATICS_H

#include "train_data.h"

#include <stdint.h>

// Floats a bone holds over a forward pass, for the backward pass.
#define MANI_TRAIN_KINEMATIC_FLOATS 35u

// Values a bone's character space has: its translation, then its two
// columns.
#define MANI_TRAIN_CHARACTER_FLOATS 9u

// From outputs y (raw, the Decompressor's layout): each bone's character
// values into character (MANI_TRAIN_CHARACTER_FLOATS a bone), and what
// the backward pass needs into work (MANI_TRAIN_KINEMATIC_FLOATS a bone).
void maniTrainKinematics(const maniTrainData* d, const float* y, float* work, float* character);

// Adds to yGradient the gradient by the outputs of a loss whose gradient
// by the character values is characterGradient; scratch holds 12 floats
// a bone.
void maniTrainKinematicsBackward(const maniTrainData* d, const float* work,
                                 const float* characterGradient, float* yGradient, float* scratch);

#endif // MAUL_ANIM_SRC_TRAIN_KINEMATICS_H
