// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.collector;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;

import javax.inject.Inject;
import javax.inject.Named;

public final class CollectorIoReal implements CollectorIo {
  private final CollectorConstants collectorConstants;

  private final TalonFX pivotMotor;
  private final TalonFX rollerMotor;


  private final StatusSignal<AngularVelocity> pivotVelocityRps;
  private final StatusSignal<Voltage> pivotAppliedVoltage;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<Current> pivotTorqueCurrent;
  private final StatusSignal<Temperature> pivotTempCelsius;


  private final StatusSignal<AngularVelocity> rollerVelocityRps;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<Current> rollerSupplyCurrent;
  private final StatusSignal<Current> rollerTorqueCurrent;
  private final StatusSignal<Temperature> rollerTempCelsius;

  @Inject
  CollectorIoReal(CollectorConstants collectorConstants, @Named("Bob") CANBus canbus) {
    this.collectorConstants = collectorConstants;
    pivotMotor = new TalonFX(collectorConstants.pivotMotorId(), canbus);
    rollerMotor = new TalonFX(collectorConstants.rollerMotorId(), canbus);

    pivotVelocityRps = pivotMotor.getVelocity();
    pivotAppliedVoltage = pivotMotor.getMotorVoltage();
    pivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
    pivotTempCelsius = pivotMotor.getDeviceTemp();

    rollerVelocityRps = rollerMotor.getVelocity();
    rollerAppliedVoltage = rollerMotor.getMotorVoltage();
    rollerSupplyCurrent = rollerMotor.getSupplyCurrent();
    rollerTorqueCurrent = rollerMotor.getTorqueCurrent();
    rollerTempCelsius = rollerMotor.getDeviceTemp();
  }
}
