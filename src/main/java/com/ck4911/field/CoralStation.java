// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.field;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * An enumeration of Coral stations. Includes poses that the robot should be in to collect at the
 * corresponding station.
 */
public enum CoralStation {
  LEFT(
      new Pose2d(
          new Translation2d(Meters.of(1.5355510711669922), Meters.of(7.372902870178223)),
          new Rotation2d(Degrees.of(126)))),
  RIGHT(
      new Pose2d(
          new Translation2d(Meters.of(1.5355510711669922), Meters.of(0.6715318560600281)),
          new Rotation2d(Degrees.of(-126))));

  public final Pose2d targetPose;

  CoralStation(Pose2d targetPose) {
    this.targetPose = targetPose;
  }
}
