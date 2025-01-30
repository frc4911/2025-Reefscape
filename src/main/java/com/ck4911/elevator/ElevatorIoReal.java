// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import static edu.wpi.first.units.Units.Volts;

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
  ElevatorIoReal() {
    motorLeft = new TalonFX(0);
    motorRight = new TalonFX(0);
    voltageRequest = new VoltageOut(0.0);
  }

  @Override
  public void runVolts(Voltage voltage) {
    motorLeft.setControl(voltageRequest.withOutput(voltage.in(Volts)));
    motorRight.setControl(voltageRequest.withOutput(voltage.in(Volts)));
  }
}
