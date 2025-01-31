// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.arm;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class ArmIoReal implements ArmIo {
  private final TalonFX motor;
  private final VoltageOut voltageRequest;

  @Inject
  ArmIoReal() {
    motor = new TalonFX(0);
    voltageRequest = new VoltageOut(0.0);
  }

  @Override
  public void stop() {}

  @Override
  public void configurePid(double p, double i, double d) {}

  @Override
  public void runVolts(Voltage voltage) {
    motor.setControl(voltageRequest.withOutput(voltage.in(Volts)));
  }
}
