// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ck4911.characterization.Characterizable;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class Elevator extends SubsystemBase implements Characterizable {
  private final ElevatorIo elevatorIo;
  private final SysIdRoutine sysIdRoutine;

  @Inject
  public Elevator(ElevatorIo elevatorIo) {
    this.elevatorIo = elevatorIo;
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("Elevator_State", state.toString())),
            new SysIdRoutine.Mechanism(elevatorIo::runVolts, null, this));
  }

  @Override
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  public void moveElevevator(double rotations) {
    elevatorIo.moveElevevator(rotations);
  }
}
