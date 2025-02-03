// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.arm;

import static com.ck4911.util.PhoenixUtils.tryUntilOk;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import javax.inject.Inject;
import javax.inject.Named;
import javax.inject.Singleton;

@Singleton
public final class ArmIoReal implements ArmIo {
  private final ArmConstants armConstants;

  private final TalonFX motor;
  private final CANcoder cancoder;

  private final StatusSignal<Angle> internalPositionRotations;
  private final StatusSignal<Angle> encoderAbsolutePositionRotations;
  private final StatusSignal<Angle> encoderRelativePositionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final VoltageOut voltageControl;
  private final TorqueCurrentFOC currentControl;
  private final PositionTorqueCurrentFOC positionControl;

  private final TalonFXConfiguration config;

  @Inject
  ArmIoReal(ArmConstants armConstants, @Named("Bob") CANBus canbus) {
    this.armConstants = armConstants;
    motor = new TalonFX(armConstants.motorId(), canbus);
    cancoder = new CANcoder(armConstants.encoderId(), canbus);

    voltageControl = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    positionControl = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset =
        Units.radiansToRotations(armConstants.armEncoderOffsetRads());
    cancoder.getConfigurator().apply(cancoderConfig, 1.0);

    config = new TalonFXConfiguration();

    config.Slot0.kP = armConstants.feedBackValues().p();
    config.Slot0.kI = armConstants.feedBackValues().i();
    config.Slot0.kD = armConstants.feedBackValues().d();
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.MotorOutput.Inverted =
        armConstants.inverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = armConstants.gearRatio();
    config.Feedback.SensorToMechanismRatio = 1.0;

    motor.getConfigurator().apply(config, 1.0);

    internalPositionRotations = motor.getPosition();
    encoderAbsolutePositionRotations = cancoder.getAbsolutePosition();
    encoderRelativePositionRotations = cancoder.getPosition();
    velocityRps = motor.getVelocity();
    appliedVoltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    tempCelsius = motor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);

    BaseStatusSignal.setUpdateFrequencyForAll(
        500, encoderAbsolutePositionRotations, encoderRelativePositionRotations);

    // Optimize bus utilization
    motor.optimizeBusUtilization(0, 1.0);
    cancoder.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(ArmIoInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)
            .isOK();
    inputs.absoluteEncoderConnected =
        BaseStatusSignal.refreshAll(
                encoderAbsolutePositionRotations, encoderRelativePositionRotations)
            .isOK();

    inputs.positionRads = Units.rotationsToRadians(internalPositionRotations.getValueAsDouble());
    inputs.absoluteEncoderPositionRads =
        Units.rotationsToRadians(encoderAbsolutePositionRotations.getValueAsDouble())
            - armConstants.armEncoderOffsetRads(); // Negate internal offset
    inputs.relativeEncoderPositionRads =
        Units.rotationsToRadians(encoderRelativePositionRotations.getValueAsDouble())
            - armConstants.armEncoderOffsetRads();
    inputs.velocityRadsPerSec = velocityRps.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVoltage.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.torqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.tempCelcius = tempCelsius.getValue().in(Celsius);
  }

  @Override
  public void runPosition(Angle position, Current feedforward) {
    motor.setControl(positionControl.withPosition(position).withFeedForward(feedforward));
  }

  @Override
  public void runVolts(Voltage voltage) {
    motor.setControl(voltageControl.withOutput(voltage));
  }

  @Override
  public void runCurrent(Current amps) {
    motor.setControl(currentControl.withOutput(amps));
  }

  @Override
  public void setPid(double p, double i, double d) {
    config.Slot0.withKP(p).withKI(i).withKD(d);
    tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }

  @Override
  public void setFeedForward(double s, double g, double v, double a) {
    config.Slot0.withKS(s).withKS(s).withKV(v).withKA(a);
    tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    motor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void stop() {
    motor.setControl(new NeutralOut());
  }
}
