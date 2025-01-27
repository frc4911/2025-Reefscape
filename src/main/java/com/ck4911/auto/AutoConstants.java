// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record AutoConstants(
    double xkP, double xkD, double ykP, double ykD, double thetakP, double thetakD) {}
