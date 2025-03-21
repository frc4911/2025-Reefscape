// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import com.ck4911.drive.Drive;
import com.ck4911.field.ReefBranch;
import com.ck4911.util.LoggedTunableNumber;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dagger.assisted.Assisted;
import dagger.assisted.AssistedFactory;
import dagger.assisted.AssistedInject;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private final Field2d field;
  private final Pose2d targetPose;
  private final SwerveRequest.FieldCentricFacingAngle driveRequest;

  private boolean isFinished;
  private Debouncer debouncer;

  @AssistedFactory
  @Singleton
  public interface AutoAlignFactory {
    AutoAlign alignWith(ReefBranch reefPosition);
  }

  @AssistedInject
  AutoAlign(
      Drive drive,
      LoggedTunableNumber.TunableNumbers tunableNumbers,
      Field2d field,
      @Assisted ReefBranch reefPosition) {
    addRequirements(drive);
    this.drive = drive;
    this.targetPose = reefPosition.targetPose;
    this.field = field;
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
  public void initialize() {
    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    field.getObject("TargetPose").setPose(targetPose);
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
