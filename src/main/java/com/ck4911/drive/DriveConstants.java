// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.drive;

import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record DriveConstants(
    int pigeonId,
    String canBusId,
    double wheelRadius,
    double trackWidthX,
    double trackWidthY,
    double bumperWidthX,
    double bumperWidthY,
    double maxLinearVelocity,
    double maxLinearAcceleration,
    double maxAngularVelocity,
    double maxAngularAcceleration,
    double ffkS,
    double ffkV,
    double ffkT,
    double drivekP,
    double drivekD,
    double turnkP,
    double turnkD,
    double driveReduction,
    double turnReduction,
    double drivePeakTorqueCurrent,
    double turnPeakTorqueCurrent) {

  // From SDS website
  public static double TURN_GEAR_RATIO = 150.0 / 7.0;
  public static double L1_GEAR_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
  public static double L2_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  public static double L3_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

  public double driveBaseRadius() {
    return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
  }
}
