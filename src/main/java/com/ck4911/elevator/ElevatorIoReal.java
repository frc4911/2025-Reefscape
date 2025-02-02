// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class ElevatorIoReal implements ElevatorIo {
  private final TalonFX motorLeft;
  private final TalonFX motorRight;
  private final VoltageOut voltageRequest;

  @Inject
  ElevatorIoReal(ElevatorConstants elevatorConstants) {
    motorLeft = new TalonFX(elevatorConstants.motorLeftId());
    motorRight = new TalonFX(elevatorConstants.motorRightId());
    motorRight.setControl(new Follower(elevatorConstants.motorLeftId(), true));
    voltageRequest = new VoltageOut(0.0);

    TalonFXConfiguration fxConfiguration = new TalonFXConfiguration();
    fxConfiguration.Feedback.RotorToSensorRatio = elevatorConstants.gearRatio();
    motorLeft.getConfigurator().apply(fxConfiguration);
  }

  @Override
  public void moveElevevator(double rotations) {
    motorLeft.setPosition(Rotations.of(rotations));
  }

  @Override
  public void runVolts(Voltage voltage) {
    motorLeft.setControl(voltageRequest.withOutput(voltage.in(Volts)));
    motorRight.setControl(voltageRequest.withOutput(voltage.in(Volts)));
  }
}
