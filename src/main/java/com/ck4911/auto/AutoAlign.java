// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ck4911.drive.Drive;
import com.ck4911.util.LoggedTunableNumber;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dagger.assisted.Assisted;
import dagger.assisted.AssistedFactory;
import dagger.assisted.AssistedInject;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

/** A command for automatically aligning with a given pose. */
public final class AutoAlign extends Command {
  private final LoggedTunableNumber p;
  private final LoggedTunableNumber d;
  private final LoggedTunableNumber debounceTime;
  private final PIDController xController;
  private final PIDController yController;
  private final Drive drive;
  private final Pose2d targetPose;
  private final SwerveRequest.FieldCentricFacingAngle driveRequest;

  private boolean isFinished;
  private Debouncer debouncer;

  @AssistedFactory
  @Singleton
  public interface AutoAlignFactory {
    AutoAlign alignWith(Pose2d targetPose);
  }

  /**
   * A representation of poses that the robot should be in to score on corresponding reef branches. The naming begins
   * with the branch on the left side of the reef face facing the driver station wall and increases counterclockwise
   * around the reef. Refer to the game manual for a visual aid.
   */
  public enum ReefPosition {
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
                    new Translation2d(Meters.of(5.789953708648682 ), Meters.of(3.8624777793884277 )),
                    new Rotation2d(Degrees.of(180)))),
    H(
            new Pose2d(
                    new Translation2d(Meters.of(5.281809329986572 ), Meters.of(5.075127124786377 )),
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
                    new Rotation2d(Degrees.of(300))))
    ;

    private final Pose2d targetPose;

    ReefPosition(Pose2d targetPose) {
      this.targetPose = targetPose;
    }

    Pose2d getPoseTargetPose() {
      return targetPose;
    }
  }

  @AssistedInject
  AutoAlign(
      Drive drive, LoggedTunableNumber.TunableNumbers tunableNumbers, @Assisted Pose2d targetPose) {
    addRequirements(drive);
    this.drive = drive;
    this.targetPose = targetPose;
    p = tunableNumbers.create("Auto/p", 10); // move constants somewhere else
    d = tunableNumbers.create("Auto/d", 0);
    debounceTime = tunableNumbers.create("Auto/debounce", 0.3);
    debouncer = new Debouncer(debounceTime.get());
    xController = new PIDController(p.get(), 0, d.get());
    yController = new PIDController(p.get(), 0, d.get());
    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    driveRequest =
        new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);
  }

  @Override
  public void execute() {
    LoggedTunableNumber.ifChanged(hashCode(), () -> debouncer = new Debouncer(debounceTime.get()));
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          xController.setPID(p.get(), 0, d.get());
          yController.setPID(p.get(), 0, d.get());
        },
        p,
        d);
    Pose2d currentPose = drive.getState().Pose;
    drive.setControl(
        driveRequest
            .withVelocityX(xController.calculate(currentPose.getX()))
            .withVelocityY(yController.calculate(currentPose.getY()))
            .withTargetDirection(targetPose.getRotation()));
    double thetaError = targetPose.getRotation().minus(currentPose.getRotation()).getDegrees();
    Logger.recordOutput("Drive/AutoAlignErrorX", xController.getError());
    Logger.recordOutput("Drive/AutoAlignErrorY", yController.getError());
    Logger.recordOutput("Drive/AutoAlignErrorTheta", thetaError);

    // Wait until the robot is at the target pose for a certain amount of time before finishing
    isFinished =
        debouncer.calculate(
            xController.atSetpoint() && yController.atSetpoint() && Math.abs(thetaError) < 0.05);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
