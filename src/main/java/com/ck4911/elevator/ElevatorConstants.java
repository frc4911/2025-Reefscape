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
    double corralHeight,
    double sprocketRadius,
    double gearRatio,
    double homingVolts,
    double homingVelocityThresh,
    double homingTimeSeconds,
    double tolerance,
    double minPositionRads,
    double maxPositionRads,
    double stowPositionRadians,
    double prepareCollectPositionRadians,
    double collectPositionRadians,
    double troughPositionRadians,
    double levelTwoPositionRadians,
    double levelThreePositionRadians,
    double levelFourPositionRadians,
    double profileVelocity,
    double profileAcceleration,
    double profileJerk,
    double variance,
    double debounceTimeSeconds,
    PidValues feedBackValues,
    FeedForwardValues feedForwardValues) {}
