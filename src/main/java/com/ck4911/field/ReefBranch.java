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
 * An enumeration of reef branches. Includes poses that the robot should be in to score on
 * corresponding reef branches. The naming begins with the branch on the left side of the reef face
 * facing the driver station wall and increases counterclockwise around the reef. Refer to the game
 * manual for a visual aid.
 */
public enum ReefBranch {
  A(
      new Pose2d(
          new Translation2d(Meters.of(3.1920571327209473), Meters.of(4.191031455993652)),
          new Rotation2d(Degrees.of(0)))),
  B(
      new Pose2d(
          new Translation2d(Meters.of(3.1920571327209473), Meters.of(3.8624777793884277)),
          new Rotation2d(Degrees.of(0)))),
  C(
      new Pose2d(
          new Translation2d(Meters.of(3.697187662124634), Meters.of(2.978645086288452)),
          new Rotation2d(Degrees.of(60)))),
  D(
      new Pose2d(
          new Translation2d(Meters.of(3.9764328002929688), Meters.of(2.8142290115356445)),
          new Rotation2d(Degrees.of(60)))),
  E(
      new Pose2d(
          new Translation2d(Meters.of(4.996878147125244), Meters.of(2.8142290115356445)),
          new Rotation2d(Degrees.of(120)))),
  F(
      new Pose2d(
          new Translation2d(Meters.of(5.28546667098999), Meters.of(2.978645086288452)),
          new Rotation2d(Degrees.of(120)))),
  G(
      new Pose2d(
          new Translation2d(Meters.of(5.789953708648682), Meters.of(3.8624777793884277)),
          new Rotation2d(Degrees.of(180)))),
  H(
      new Pose2d(
          new Translation2d(Meters.of(5.281809329986572), Meters.of(5.075127124786377)),
          new Rotation2d(Degrees.of(180)))),
  I(
      new Pose2d(
          new Translation2d(Meters.of(4.998748302459717), Meters.of(5.072605133056641)),
          new Rotation2d(Degrees.of(240)))),
  J(
      new Pose2d(
          new Translation2d(Meters.of(4.998748302459717), Meters.of(5.236284255981445)),
          new Rotation2d(Degrees.of(240)))),
  K(
      new Pose2d(
          new Translation2d(Meters.of(3.9764328002929688), Meters.of(5.236284255981445)),
          new Rotation2d(Degrees.of(300)))),
  L(
      new Pose2d(
          new Translation2d(Meters.of(3.6937344074249268), Meters.of(5.072605133056641)),
          new Rotation2d(Degrees.of(300))));

  public final Pose2d targetPose;

  ReefBranch(Pose2d targetPose) {
    this.targetPose = targetPose;
  }
}
