// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import choreo.trajectory.SwerveSample;
import com.ck4911.drive.Drive;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import javax.inject.Inject;

public final class TrajectoryFollower {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds;
  private final Drive drive;

  @Inject
  public TrajectoryFollower(AutoConstants autoConstants, Drive drive) {
    this.drive = drive;
    xController = new PIDController(autoConstants.xkD(), 0, autoConstants.xkP());
    yController = new PIDController(autoConstants.ykD(), 0, autoConstants.ykP());
    thetaController = new PIDController(autoConstants.thetakD(), 0, autoConstants.thetakP());
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
}
