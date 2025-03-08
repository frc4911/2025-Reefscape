// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.commands;

import com.ck4911.arm.Arm;
import com.ck4911.drive.Drive;
import com.ck4911.elevator.Elevator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class CyberCommands {
  private final Arm arm;
  private final Elevator elevator;
  private final Drive drive;

  @Inject
  public CyberCommands(Arm arm, Elevator elevator, Drive drive) {
    this.arm = arm;
    this.elevator = elevator;
    this.drive = drive;
  }

  public Command prepareForCollect() {
    return elevator
        .prepareCollect()
        .raceWith(elevator.waitUntilPrepareCollect())
        .andThen(elevator.prepareCollect().alongWith(arm.collect()));
  }

  public Command collect() {
    // TODO: safety measures (check arm position)
    return elevator
        .collect()
        .raceWith(arm.waitForCoralPresent())
        .andThen(
            elevator.passCorral().raceWith(elevator.waitUntilPrepareCollect()).andThen(stow()));
  }

  public Command score() {
    return arm.score().raceWith(arm.waitForCoralGone()).andThen(Commands.print("Done"));
  }

  public Command home() {
    return elevator.home().alongWith(arm.stow());
  }

  public Command homeWithCoral() {
    return elevator.homeWithCoral().alongWith(arm.stow());
  }

  public Command trough() {
    return elevator.trough().alongWith(arm.trough());
  }

  public Command levelTwo() {
    return elevator.levelTwo().alongWith(arm.levelTwoAndThree());
  }

  public Command levelThree() {
    return elevator.levelThree().alongWith(arm.levelTwoAndThree());
  }

  public Command levelFour() {
    return elevator.levelFour().alongWith(arm.levelFour());
  }

  public Command stow() {
    // TODO: Check if clear of corral first
    // If the elevator is too low, this could be bad
    return arm.stow().raceWith(arm.waitForStowPosition().andThen(elevator.stow()));
  }

  public Command correctPose(Pose2d pose) {
    return Commands.runOnce(
        () -> {
          drive.correctQuestPose(pose);
        });
  }

  // TODO: check with driver for which direction is prefered for zeroing
  // currently, this assumes facing away from driver station.
  public Command resetForward(Angle forwardAngle) {
    return Commands.runOnce(
        () ->
            drive.resetPose(
                new Pose2d(drive.getState().Pose.getTranslation(), new Rotation2d(forwardAngle))));
  }
}
