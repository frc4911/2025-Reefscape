// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import com.ck4911.util.FeedForwardValues;
import com.ck4911.util.PidValues;
import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record ElevatorConstants(
    int motorLeftId,
    int motorRightId,
    double sprocketRadius,
    double gearRatio,
    double homingTimeSeconds,
    double tolerance,
    double minPositionRads,
    double maxPositionRads,
    double stowPositionRotations,
    double prepareCollectPositionRotations,
    double collectPositionRotations,
    double troughPositionRotations,
    double levelTwoPositionRotations,
    double levelThreePositionRotations,
    double levelFourPositionRotations,
    PidValues feedBackValues,
    FeedForwardValues feedForwardValues) {}
