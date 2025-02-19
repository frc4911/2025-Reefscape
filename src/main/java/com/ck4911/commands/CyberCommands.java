// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.commands;

import com.ck4911.arm.Arm;
import com.ck4911.elevator.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class CyberCommands {
  private final Arm arm;
  private final Elevator elevator;

  @Inject
  public CyberCommands(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
  }

  public Command prepareForCollectionCommand() {
    return elevator.prepareCollect().andThen(arm.collect());
  }

  // ONLY CALL AFTER PREPARE
  public Command collectCommand() {
    return elevator.collect();
    // TODO: detect coral in arm
  }

  public Command score() {
    return arm.score();
  }

  public Command home() {
    return elevator.home().alongWith(arm.stow());
  }

  public Command trough() {
    return elevator.trough().alongWith(arm.trough());
    // return arm.trough();
  }

  public Command levelTwo() {
    // return arm.levelTwoAndThree();
    return elevator.levelTwo().alongWith(arm.levelTwoAndThree());
  }

  public Command levelThree() {
    // return elevator.levelThree();
    return elevator.levelThree().alongWith(arm.levelTwoAndThree());
  }

  public Command levelFour() {
    // return elevator.trough();
    return elevator.levelFour().alongWith(arm.levelFour());
  }

  public Command stow() {
    // TODO: move to a safe height first
    // If the elevator is too low, this could be bad
    return arm.stow().alongWith(elevator.stow());
    // return elevator.stow();
  }
}
