// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.drive;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIo {
  @AutoLog
  public static class SwerveModuleIoInputs {
    public boolean driveMotorConnected;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTorqueCurrentAmps = 0.0;
    public double driveTempCelcius;

    public boolean encoderConnected;
    public double encoderPositionRads = 0.0;

    public boolean turnMotorConnected;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnTorqueCurrentAmps = 0.0;
    public double turnTempCelcius;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveModuleIoInputs inputs) {}
}
