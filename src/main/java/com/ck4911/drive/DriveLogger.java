// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

/** A shim to read signals from devices in the generated swerve code and pass them to our logs */
final class DriveLogger {
  private static final double debounceTime = 0.5;
  private final SwerveModuleIoInputsAutoLogged inputs = new SwerveModuleIoInputsAutoLogged();

  private final StatusSignal<Angle> encoderPositionRotations;

  private final StatusSignal<Angle> drivePositionRadians;
  private final StatusSignal<AngularVelocity> driveVelocityRps;
  private final StatusSignal<Voltage> driveAppliedVoltage;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Current> driveTorqueCurrent;
  private final StatusSignal<Temperature> driveTempCelsius;

  private final StatusSignal<Angle> steerPositionRadians;
  private final StatusSignal<AngularVelocity> steerVelocityRps;
  private final StatusSignal<Voltage> steerAppliedVoltage;
  private final StatusSignal<Current> steerSupplyCurrent;
  private final StatusSignal<Current> steerTorqueCurrent;
  private final StatusSignal<Temperature> steerTempCelsius;

  private final String name;
  private final Debouncer driveConnectedDebouncer;
  private final Debouncer steerConnectedDebouncer;
  private final Debouncer encoderConnectedDebouncer;

  DriveLogger(String name, SwerveModule<TalonFX, TalonFX, CANcoder> module) {
    this.name = name;
    encoderPositionRotations = module.getEncoder().getAbsolutePosition();

    drivePositionRadians = module.getDriveMotor().getPosition();
    driveVelocityRps = module.getDriveMotor().getVelocity();
    driveAppliedVoltage = module.getDriveMotor().getMotorVoltage();
    driveSupplyCurrent = module.getDriveMotor().getSupplyCurrent();
    driveTorqueCurrent = module.getDriveMotor().getTorqueCurrent();
    driveTempCelsius = module.getDriveMotor().getDeviceTemp();

    steerPositionRadians = module.getSteerMotor().getPosition();
    steerVelocityRps = module.getSteerMotor().getVelocity();
    steerAppliedVoltage = module.getSteerMotor().getMotorVoltage();
    steerSupplyCurrent = module.getSteerMotor().getSupplyCurrent();
    steerTorqueCurrent = module.getSteerMotor().getTorqueCurrent();
    steerTempCelsius = module.getSteerMotor().getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        drivePositionRadians,
        driveVelocityRps,
        driveAppliedVoltage,
        driveSupplyCurrent,
        driveTorqueCurrent,
        driveTempCelsius,
        steerPositionRadians,
        steerVelocityRps,
        steerAppliedVoltage,
        steerSupplyCurrent,
        steerTorqueCurrent,
        steerTempCelsius);
    BaseStatusSignal.setUpdateFrequencyForAll(500, encoderPositionRotations);
    ParentDevice.optimizeBusUtilizationForAll(
        module.getDriveMotor(), module.getSteerMotor(), module.getEncoder());

    driveConnectedDebouncer = new Debouncer(debounceTime);
    steerConnectedDebouncer = new Debouncer(debounceTime);
    encoderConnectedDebouncer = new Debouncer(debounceTime);
  }

  public void updateInputs() {
    boolean driveMotorConnected =
        BaseStatusSignal.refreshAll(
                drivePositionRadians,
                driveVelocityRps,
                driveAppliedVoltage,
                driveSupplyCurrent,
                driveTorqueCurrent,
                driveTempCelsius)
            .isOK();

    boolean turnMotorConnected =
        BaseStatusSignal.refreshAll(
                steerPositionRadians,
                steerVelocityRps,
                steerAppliedVoltage,
                steerSupplyCurrent,
                steerTorqueCurrent,
                steerTempCelsius)
            .isOK();
    boolean encoderConnected = BaseStatusSignal.refreshAll(encoderPositionRotations).isOK();

    inputs.driveMotorConnected = driveConnectedDebouncer.calculate(driveMotorConnected);
    inputs.drivePositionRad = drivePositionRadians.getValue().in(Radians) / TunerConstants.kDriveGearRatio;
    inputs.driveVelocityRadPerSec = driveVelocityRps.getValue().in(RadiansPerSecond) / TunerConstants.kDriveGearRatio;
    inputs.driveAppliedVolts = driveAppliedVoltage.getValue().in(Volts);
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValue().in(Amps);
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValue().in(Amps);
    inputs.driveTempCelcius = driveTempCelsius.getValue().in(Celsius);

    inputs.turnMotorConnected = steerConnectedDebouncer.calculate(turnMotorConnected);
    inputs.turnPositionRad = steerPositionRadians.getValue().in(Radians) / TunerConstants.kSteerGearRatio;
    inputs.turnVelocityRadPerSec = steerVelocityRps.getValue().in(RadiansPerSecond) / TunerConstants.kDriveGearRatio;
    inputs.turnAppliedVolts = steerAppliedVoltage.getValue().in(Volts);
    inputs.turnSupplyCurrentAmps = steerSupplyCurrent.getValue().in(Amps);
    inputs.turnTorqueCurrentAmps = steerTorqueCurrent.getValue().in(Amps);
    inputs.turnTempCelcius = steerTempCelsius.getValue().in(Celsius);

    inputs.encoderConnected = encoderConnectedDebouncer.calculate(encoderConnected);
    inputs.encoderPositionRads = encoderPositionRotations.getValue().in(Radians);

    Logger.processInputs("Drive/Module" + name, inputs);
  }

  public double getDrivePositionRadians() {
    return inputs.drivePositionRad;
  }
}
