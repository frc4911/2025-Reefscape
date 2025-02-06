// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import choreo.trajectory.SwerveSample;
import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.drive.Drive;
import com.ck4911.util.LoggedTunableNumber;
import com.ck4911.util.LoggedTunableNumber.TunableNumbers;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class TrajectoryFollower implements VirtualSubsystem {
  private final LoggedTunableNumber xP;
  private final LoggedTunableNumber xD;
  private final LoggedTunableNumber yP;
  private final LoggedTunableNumber yD;
  private final LoggedTunableNumber thetaP;
  private final LoggedTunableNumber thetaD;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds;
  private final Drive drive;

  @Inject
  public TrajectoryFollower(
      AutoConstants autoConstants, Drive drive, TunableNumbers tunableNumbers) {
    this.drive = drive;
    xP = tunableNumbers.create("Drive/xp", autoConstants.xFeedback().p());
    xD = tunableNumbers.create("Drive/xd", autoConstants.xFeedback().d());
    yP = tunableNumbers.create("Drive/yp", autoConstants.yFeedback().p());
    yD = tunableNumbers.create("Drive/yd", autoConstants.yFeedback().d());
    thetaP = tunableNumbers.create("Drive/thetap", autoConstants.thetaFeedback().p());
    thetaD = tunableNumbers.create("Drive/thetad", autoConstants.thetaFeedback().d());
    xController = new PIDController(xP.get(), 0, xD.get());
    yController = new PIDController(yP.get(), 0, yD.get());
    thetaController = new PIDController(thetaP.get(), 0, thetaD.get());
    pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
  }

  public void follow(SwerveSample sample) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    var pose = drive.getState().Pose;

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += xController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += yController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        thetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    drive.setControl(
        pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          xController.setP(xP.get());
          xController.setD(xD.get());
        },
        xP,
        xD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          yController.setP(yP.get());
          yController.setD(yD.get());
        },
        yP,
        yD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          thetaController.setP(thetaP.get());
          thetaController.setD(thetaD.get());
        },
        thetaP,
        thetaD);
  }
}
