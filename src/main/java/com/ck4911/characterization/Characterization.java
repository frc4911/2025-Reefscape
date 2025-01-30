// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.characterization;

import com.ck4911.drive.Drive;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import javax.inject.Inject;

public final class Characterization {
  private final Drive drive;

  @Inject
  Characterization(Drive drive) {
    this.drive = drive;
  }

  public Command fullDriveCharacterization(Trigger cancelAction) {
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

  public Command fullElevatorCharaterization(Trigger cancelAction) {
    return Commands.none();
  }

  public Command fullArmCharaterization(Trigger cancelAction) {
    return Commands.none();
  }
}
