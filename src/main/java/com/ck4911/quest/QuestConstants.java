// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.quest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record QuestConstants(Transform2d robotToQuest, Matrix<N3, N1> stdDevs) {}
