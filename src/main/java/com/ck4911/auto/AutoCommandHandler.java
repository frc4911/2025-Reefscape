// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.drive.Drive;
import com.ck4911.util.LocalADStarAK;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
  private Trigger cancelAction = new Trigger(() -> false);

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

  public void setCancelAction(Trigger cancelAction) {
    this.cancelAction = cancelAction;
  }

  private Command fullCharacterization() {
    return Commands.runOnce(SignalLogger::start)
        .andThen(drive.sysIdDynamic(Direction.kForward).until(cancelAction))
        .andThen(Commands.waitSeconds(1))
        .andThen(drive.sysIdDynamic(Direction.kReverse).until(cancelAction))
        .andThen(Commands.waitSeconds(1))
        .andThen(drive.sysIdQuasistatic(Direction.kForward).until(cancelAction))
        .andThen(Commands.waitSeconds(1))
        .andThen(drive.sysIdQuasistatic(Direction.kReverse).until(cancelAction))
        .finallyDo(SignalLogger::stop);
  }

  private void setupAutos() {
    chooser.addDefaultOption("Nothing", Commands.none());
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
