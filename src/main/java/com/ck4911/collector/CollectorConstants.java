// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.collector;

import com.ck4911.util.PidValues;
import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record CollectorConstants(
    int pivotMotorId,
    int rollerMotorId,
    int sensorId,
    double gearRatio,
    PidValues pivotFeedBackValues,
    PidValues rollerFeedBackValues) {}
