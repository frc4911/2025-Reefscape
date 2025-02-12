// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.leds;

import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record LedConstants(
    int pwmPort,
    int minLoopCycleCount,
    int length,
    double strobeDuration,
    double breathDuration,
    double waveExponent,
    double waveFastCycleLength,
    double waveFastDuration,
    double waveAllianceCycleLength,
    double waveAllianceDuration,
    double autoFadeTime,
    double autoFadeMaxTime) {}
