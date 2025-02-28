// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ck4911.commands.CyberCommands;
import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class AutoCommandHandler implements VirtualSubsystem {
  private final AutoChooser autoChooser;
  private final AutoFactory autoFactory;
  private final Drive drive;
  private final CyberCommands cyberCommands;
  private double autoStart;
  private boolean autoMessagePrinted;
  private Command currentAutoCommand;
  private Rotation2d startingRotation;

  @Inject
  public AutoCommandHandler(
      AutoFactory autoFactory, Drive drive, CyberCommands cyberCommands, AutoChooser autoChooser) {
    this.autoChooser = autoChooser;
    this.drive = drive;
    this.cyberCommands = cyberCommands;
    this.autoFactory = autoFactory;

    bindNamedCommands();
    addAutos();
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

  private void bindNamedCommands() {
    autoFactory.bind("Home", cyberCommands.home());
    // TODO: bind name commands so that waypoints trigger them
  }

  private void addAutos() {
    startingRotation = new Rotation2d(180);
    autoChooser.addCmd("test", () -> Commands.print("hi"));
    autoChooser.addCmd(
        "1m",
        () ->
            Commands.runOnce(
                    () ->
                        drive.resetPose(
                            new Pose2d(
                                Meters.of(7.125288963317871),
                                Meters.of(0.419444739818573),
                                startingRotation)))
                .andThen(autoFactory.trajectoryCmd("1m")));

    SmartDashboard.putData("Autos", autoChooser);
  }

  public void startCurrentCommand() {
    stopCurrentCommand();
    autoStart = Timer.getFPGATimestamp();
    currentAutoCommand = autoChooser.selectedCommandScheduler();
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
