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
    // TODO: run these at the same time SAFELY (optional)
    return arm.collectPosition().andThen(elevator.prepareCollectPosition());
    // TODO: detect coral in corral
  }

  // ONLY CALL AFTER PREPARE
  public Command collectCommand() {
    return elevator.collectPosition();
    // TODO: detect coral in arm
  }

  public Command prepareScoreL3Command() {
    return elevator.prepareScoreL3Position().andThen(arm.prepareScoreMidPosition());
  }

  public Command stowCommand() {
    return arm.stowPosition().andThen(elevator.stowPosition());
  }
}
