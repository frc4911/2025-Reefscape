// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.characterization;

import com.ck4911.arm.Arm;
import com.ck4911.drive.Drive;
import com.ck4911.elevator.Elevator;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import javax.inject.Inject;

public final class Characterization {
  private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;

  @Inject
  Characterization(Drive drive, Elevator elevator, Arm arm) {
    this.drive = drive;
    this.elevator = elevator;
    this.arm = arm;
  }

  public Command fullDriveCharacterization(Trigger cancelAction) {
    return fullCharacterization(drive, cancelAction);
  }

  public Command fullElevatorCharaterization(Trigger cancelAction) {
    return fullCharacterization(elevator, cancelAction);
  }

  public Command fullArmCharaterization(Trigger cancelAction) {
    return fullCharacterization(arm, cancelAction);
  }

  private Command fullCharacterization(Characterizable characterizable, Trigger cancelAction) {
    return Commands.runOnce(SignalLogger::start)
        .andThen(characterizable.getSysIdRoutine().dynamic(Direction.kForward).until(cancelAction))
        .andThen(Commands.waitSeconds(1))
        .andThen(characterizable.getSysIdRoutine().dynamic(Direction.kReverse).until(cancelAction))
        .andThen(Commands.waitSeconds(1))
        .andThen(
            characterizable.getSysIdRoutine().quasistatic(Direction.kForward).until(cancelAction))
        .andThen(Commands.waitSeconds(1))
        .andThen(
            characterizable.getSysIdRoutine().quasistatic(Direction.kReverse).until(cancelAction))
        .finallyDo(SignalLogger::stop);
  }
}
