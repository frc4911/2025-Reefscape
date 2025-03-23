// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.collector;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface CollectorIo {
  @AutoLog
  class CollectorIoInputs {
    public boolean pivotMotorConnected = true;
    public boolean rollerMotorConnected = true;

    public double pivotPositionRads;
    public double pivotVelocityRadsPerSec;
    public double pivotAppliedVolts;
    public double pivotSupplyCurrentAmps;
    public double pivotTorqueCurrentAmps;
    public double pivotTempCelcius;

    public double rollerPositionRads;
    public double rollerVelocityRadsPerSec;
    public double rollerAppliedVolts;
    public double rollerSupplyCurrentAmps;
    public double rollerTorqueCurrentAmps;
    public double rollerTempCelcius;

    public boolean sensorConnected;
    public int sensorStatus;
    public int sensorAmbient;
    public int sensorDistanceMillimeters;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(CollectorIo.CollectorIoInputs inputs) {}

  default void runRollerVelocity(AngularVelocity velocity) {}

  default void runPivotPosition(Angle position, Current feedforward) {}

  /** Set feed back constants. */
  default void setPivotPid(double p, double i, double d) {}

  default void setPivotFeedForward(double s, double g, double v, double a) {}

  public default void setRollerPid(double p, double i, double d) {}
}
