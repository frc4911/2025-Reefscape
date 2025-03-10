// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.quest;

import com.ck4911.drive.Drive;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public class QuestCalibration {
  // -- Calculate Quest Offset (copied from
  // https://github.com/FRC5010/Reefscape2025/blob/main/TigerShark2025/src/main/java/org/frc5010/common/sensors/camera/QuestNav.java#L65) --

  private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds;
  private Translation2d calculatedOffsetToRobot = new Translation2d();
  private double calculateOffsetCount = 1;

  @Inject
  QuestCalibration() {
    applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
  }

  private Translation2d calculateOffsetToRobot(Pose2d robotPose) {
    Rotation2d angle = robotPose.getRotation();
    Translation2d displacement = robotPose.getTranslation();

    double x =
        ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
            / (2 * (1 - angle.getCos()));
    double y =
        ((-angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
            / (2 * (1 - angle.getCos()));

    return new Translation2d(x, y);
  }

  public Command determineOffsetToRobotCenter(
      Drive drive, Supplier<Pose2d> robotPose, Supplier<Pose2d> questPoseSupplier) {
    return Commands.repeatingSequence(
            Commands.run(
                    () -> {
                      drive.setControl(
                          applyFieldSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0.3141)));
                    },
                    drive)
                .withTimeout(0.5),
            Commands.runOnce(
                () -> {
                  // Update current offset
                  Translation2d offset = calculateOffsetToRobot(robotPose.get());

                  calculatedOffsetToRobot =
                      calculatedOffsetToRobot
                          .times((double) calculateOffsetCount / (calculateOffsetCount + 1))
                          .plus(offset.div(calculateOffsetCount + 1));
                  calculateOffsetCount++;
                  Logger.recordOutput("QuestCalibration/CalculatedOffset", calculatedOffsetToRobot);
                }))
        .onlyIf(() -> questPoseSupplier.get().getRotation().getDegrees() > 30);
  }
}
