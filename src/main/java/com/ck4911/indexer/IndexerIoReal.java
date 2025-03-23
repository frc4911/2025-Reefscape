// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.indexer;

import static com.ck4911.util.PhoenixUtils.tryUntilOk;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class IndexerIoReal implements IndexerIo {

  private final TalonFX motor;

  private final StatusSignal<Angle> positionRads;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final VelocityTorqueCurrentFOC velocityControl;

  private final TalonFXConfiguration config;

  private final Debouncer motorConnectedDebouncer;
  private final Debouncer sensorConnectedDebouncer;
  private final LaserCan distanceSensor;

  @Inject
  IndexerIoReal(IndexerConstants constants, @Named("Bob") CANBus canbus) {
    motor = new TalonFX(constants.motorId(), canbus);

    velocityControl = new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    motorConnectedDebouncer = new Debouncer(0.5);
    sensorConnectedDebouncer = new Debouncer(0.5);

    config =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs())
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Amps.of(80.0))
                    .withPeakReverseTorqueCurrent(Amps.of(-80)))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80.0))
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(constants.gearRatio()))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                    .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)));
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 1.0));

    positionRads = motor.getPosition();
    velocityRps = motor.getVelocity();
    appliedVoltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    tempCelsius = motor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100, positionRads, velocityRps, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);

    distanceSensor = new LaserCan(constants.sensorId());
    try {
      distanceSensor.setRangingMode(LaserCanInterface.RangingMode.SHORT);
      distanceSensor.setRegionOfInterest(new LaserCanInterface.RegionOfInterest(8, 8, 6, 6));
      distanceSensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(IndexerIoInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                positionRads,
                velocityRps,
                appliedVoltage,
                torqueCurrent,
                supplyCurrent,
                tempCelsius)
            .isOK();

    inputs.motorConnected = motorConnectedDebouncer.calculate(connected);
    inputs.positionRads = positionRads.getValue().in(Radians);
    inputs.velocityRadsPerSec = velocityRps.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVoltage.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.torqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.tempCelcius = tempCelsius.getValue().in(Celsius);

    LaserCanInterface.Measurement measurement = distanceSensor.getMeasurement();
    inputs.sensorConnected = sensorConnectedDebouncer.calculate(measurement != null);
    if (inputs.sensorConnected
        && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.sensorStatus = measurement.status;
      inputs.sensorAmbient = measurement.ambient;
      inputs.sensorDistanceMillimeters = measurement.distance_mm;
    }
  }

  @Override
  public void runVelocity(AngularVelocity velocity, Current feedforward) {
    motor.setControl(velocityControl.withVelocity(velocity).withFeedForward(feedforward));
  }

  @Override
  public void setPid(double p, double i, double d) {
    config.Slot0.withKP(p).withKI(i).withKD(d);
    tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }
}
