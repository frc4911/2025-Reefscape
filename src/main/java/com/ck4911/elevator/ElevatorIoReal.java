// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class ElevatorIoReal implements ElevatorIo {
  private final TalonFX motor;
  private final TalonFX motorRight;
  private final VoltageOut voltageRequest;

  @Inject
  ElevatorIoReal(ElevatorConstants elevatorConstants) {
    motor = new TalonFX(elevatorConstants.motorId());
    motorRight = new TalonFX(elevatorConstants.motorRightId());
    motorRight.setControl(new Follower(elevatorConstants.motorId(), false));
    voltageRequest = new VoltageOut(0.0);
  }

  @Override
  public void moveElevevator(double rotations) {
    motor.setPosition(Rotations.of(rotations));
  }

  @Override
  public void runVolts(Voltage voltage) {
    motor.setControl(voltageRequest.withOutput(voltage.in(Volts)));
    motorRight.setControl(voltageRequest.withOutput(voltage.in(Volts)));
  }
}
