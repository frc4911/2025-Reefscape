// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIo {
  @AutoLog
  class ElevatorIoInputs {
    public boolean leaderConnected = true;
    public boolean followerConnected = true;
    public double positionRads = 0.0;
    public double velocityRadPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderTorqueCurrentAmps = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;
  }

  default void updateInputs(ElevatorIoInputs inputs) {}

  default void runOpenLoop(double output) {}

  public default void runVolts(Voltage voltage) {}

  default void stop() {}

  /** Run elevator output shaft to positionRad with additional feedforward output */
  default void runPosition(Angle position, Current feedforward) {}

  default void setPid(double p, double i, double d) {}

  default void setFeedForward(double s, double g, double v, double a) {}

  default void setBrakeMode(boolean enabled) {}
}
