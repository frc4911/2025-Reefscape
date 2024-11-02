// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.drive.Drive;
import com.ck4911.util.LocalADStarAK;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@Singleton
public final class AutoCommandHandler implements VirtualSubsystem {
  private final LoggedDashboardChooser<Command> chooser;
  private final Drive drive;
  private double autoStart;
  private boolean autoMessagePrinted;
  private Command currentAutoCommand;

  @Inject
  public AutoCommandHandler(Drive drive, AutoConstants autoConstants) {
    this.drive = drive;

    chooser = new LoggedDashboardChooser<Command>("Auto Routine");

    // TODO: Configure AutoBuilder and drive subsystem for PathPlanner

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    setupAutos();
    setupChoreoAutos(autoConstants);
  }

  @Override
  public void periodic() {
    // Print auto duration
    if (currentAutoCommand != null) {
      if (!currentAutoCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }
  }

  private void setupAutos() {
    chooser.addDefaultOption("Nothing", Commands.none());
  }

  private void setupChoreoAutos(AutoConstants autoConstants) {
    var xController = new PIDController(autoConstants.xkD(), 0, autoConstants.xkP());
    var yController = new PIDController(autoConstants.ykD(), 0, autoConstants.ykP());
    var thetaController = new PIDController(autoConstants.thetakD(), 0, autoConstants.thetakP());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Choreo.<SwerveSample>createAutoFactory(
        drive,
        drive::getPose,
        (pose, sample) -> {
          var targetSpeeds = sample.getChassisSpeeds();
          targetSpeeds.vxMetersPerSecond += xController.calculate(pose.getX(), sample.x);
          targetSpeeds.vyMetersPerSecond += yController.calculate(pose.getY(), sample.y);
          targetSpeeds.omegaRadiansPerSecond +=
              thetaController.calculate(pose.getRotation().getRadians(), sample.heading);
          drive.runVelocity(targetSpeeds);
        },
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        new AutoFactory.AutoBindings(),
        (trajectory, starting) -> {});
  }

  public void startCurrentCommand() {
    stopCurrentCommand();
    autoStart = Timer.getFPGATimestamp();
    currentAutoCommand = chooser.get();
    if (currentAutoCommand != null) {
      currentAutoCommand.schedule();
    }
  }

  public void stopCurrentCommand() {
    if (currentAutoCommand != null) {
      currentAutoCommand.cancel();
    }
  }
}
