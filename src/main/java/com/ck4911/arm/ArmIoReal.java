// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.arm;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class ArmIoReal implements ArmIo {
  private final TalonFX motor;
  private final CANcoder armCancoder;
  private final VoltageOut voltageRequest;

  @Inject
  ArmIoReal(ArmConstants armConstants) {
    motor = new TalonFX(armConstants.motorId());
    voltageRequest = new VoltageOut(0.0);
    armCancoder = new CANcoder(armConstants.encoderId());
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(0.4));
    armCancoder.getConfigurator().apply(cancoderConfig);

    TalonFXConfiguration fxConfiguration = new TalonFXConfiguration();
    fxConfiguration.Feedback.FeedbackRemoteSensorID = armCancoder.getDeviceID();
    fxConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fxConfiguration.Feedback.RotorToSensorRatio = armConstants.gearRatio();
    motor.getConfigurator().apply(fxConfiguration);
  }

  @Override
  public void setArmPosition(Angle angle) {
    motor.setPosition(angle);
  }

  @Override
  public void stop() {
    // TODO: stop
  }

  @Override
  public void configurePid(double p, double i, double d) {
    // TODO: pass these onto the motor controller
  }

  @Override
  public void runVolts(Voltage voltage) {
    motor.setControl(voltageRequest.withOutput(voltage.in(Volts)));
  }
}
