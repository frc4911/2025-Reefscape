// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import static com.ck4911.util.PhoenixUtils.tryUntilOk;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import javax.inject.Inject;
import javax.inject.Named;
import javax.inject.Singleton;

@Singleton
public final class ElevatorIoReal implements ElevatorIo {
  private final TalonFX motorLeader;
  private final TalonFX motorFollower;

  // Config
  private final TalonFXConfiguration config;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;

  private final Debouncer leaderConnectedDebouncer;
  private final Debouncer followerConnectedDebouncer;

  private final TorqueCurrentFOC torqueCurrentRequest;
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest;
  private final VoltageOut voltageRequest;

  @Inject
  ElevatorIoReal(ElevatorConstants elevatorConstants, @Named("Bob") CANBus canbus) {
    motorLeader = new TalonFX(elevatorConstants.motorLeftId(), canbus);
    motorFollower = new TalonFX(elevatorConstants.motorRightId(), canbus);

    torqueCurrentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

    leaderConnectedDebouncer = new Debouncer(0.5);
    followerConnectedDebouncer = new Debouncer(0.5);

    config = new TalonFXConfiguration();
    config.Slot0.withGravityType(GravityTypeValue.Elevator_Static);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = elevatorConstants.gearRatio();
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> motorLeader.getConfigurator().apply(config, 0.25));

    position = motorLeader.getPosition();
    velocity = motorLeader.getVelocity();
    appliedVolts = motorLeader.getMotorVoltage();
    torqueCurrent = motorLeader.getTorqueCurrent();
    supplyCurrent = motorLeader.getSupplyCurrent();
    temp = motorLeader.getDeviceTemp();
    followerAppliedVolts = motorFollower.getMotorVoltage();
    followerTorqueCurrent = motorFollower.getTorqueCurrent();
    followerSupplyCurrent = motorFollower.getSupplyCurrent();
    followerTemp = motorFollower.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, torqueCurrent, temp);
    ParentDevice.optimizeBusUtilizationForAll(motorLeader);
  }

  @Override
  public void updateInputs(ElevatorIoInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp)
            .isOK();
    boolean followerConnected =
        BaseStatusSignal.refreshAll(
                followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent, followerTemp)
            .isOK();

    inputs.leaderConnected = leaderConnectedDebouncer.calculate(connected);
    inputs.followerConnected = followerConnectedDebouncer.calculate(followerConnected);
    inputs.positionRads = position.getValue().in(Radians);
    inputs.velocityRadPerSec = velocity.getValue().in(RadiansPerSecond);
    inputs.leaderAppliedVolts = appliedVolts.getValue().in(Volts);
    inputs.leaderSupplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.leaderTorqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.leaderTempCelsius = temp.getValue().in(Celsius);
    inputs.followerAppliedVolts = followerAppliedVolts.getValue().in(Volts);
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValue().in(Amps);
    inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValue().in(Amps);
    inputs.followerTempCelsius = followerTemp.getValue().in(Celsius);
  }

  @Override
  public void runOpenLoop(double output) {
    motorLeader.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void runVolts(Voltage volts) {
    motorLeader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motorLeader.stopMotor();
  }

  @Override
  public void runPosition(Angle position, Current feedforward) {
    motorLeader.setControl(
        positionTorqueCurrentRequest.withPosition(position).withFeedForward(feedforward));
  }

  @Override
  public void setPid(double p, double i, double d) {
    config.Slot0.withKP(p).withKI(i).withKD(d);
    tryUntilOk(5, () -> motorLeader.getConfigurator().apply(config));
  }

  @Override
  public void setFeedForward(double s, double g, double v, double a) {
    config.Slot0.withKS(s).withKS(s).withKV(v).withKA(a);
    tryUntilOk(5, () -> motorLeader.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    motorLeader.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
