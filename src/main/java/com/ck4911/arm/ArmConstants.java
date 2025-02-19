// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.arm;

import com.ck4911.util.FeedForwardValues;
import com.ck4911.util.PidValues;
import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record ArmConstants(
    int motorId,
    int encoderId,
    int sensorId,
    double gearRatio,
    double armEncoderOffsetRads,
    double minPositionRads,
    double maxPositionRads,
    double stowPositionDegrees,
    double collectPositionDegrees,
    double scorePositionDegrees,
    double troughPositionDegrees,
    double levelTwoAndThreePositionDegrees,
    double levelFourPositionDegrees,
    double profileVelocity,
    double profileAcceleration,
    double profileJerk,
    double variance,
    double debounceTimeSeconds,
    PidValues feedBackValues,
    FeedForwardValues feedForwardValues) {}
