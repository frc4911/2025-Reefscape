// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.drive.Drive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class AutoCommandHandler implements VirtualSubsystem {
  private final AutoChooser autoChooser;
  private final Drive drive;
  private final AutoFactory autoFactory;
  private double autoStart;
  private boolean autoMessagePrinted;
  private Command currentAutoCommand;

  @Inject
  public AutoCommandHandler(Drive drive, AutoConstants autoConstants, AutoFactory autoFactory) {
    this.drive = drive;
    this.autoFactory = autoFactory;
    autoChooser = new AutoChooser();

    // Pathfinding.setPathfinder(new LocalADStarAK());
    // PathPlannerLogging.setLogActivePathCallback(
    //     (activePath) -> {
    //       Logger.recordOutput(
    //           "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    //     });
    // PathPlannerLogging.setLogTargetPoseCallback(
    //     (targetPose) -> {
    //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    //     });

    setupAutos();
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
    autoChooser.addCmd("test", () -> Commands.print("hi"));
    autoChooser.addRoutine("1m", this::exampleRoutine);
    SmartDashboard.putData("Autos", autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  private AutoRoutine exampleRoutine() {
    return autoFactory.newRoutine("1m");
  }

  // public void startCurrentCommand() {
  //   stopCurrentCommand();
  //   autoStart = Timer.getFPGATimestamp();
  //   currentAutoCommand = chooser.get();
  //   if (currentAutoCommand != null) {
  //     currentAutoCommand.schedule();
  //   }
  // }

  // public void stopCurrentCommand() {
  //   if (currentAutoCommand != null) {
  //     currentAutoCommand.cancel();
  //   }
  // }
}
