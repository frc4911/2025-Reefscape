// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIo {
  @AutoLog
  public static class ArmIoInputs {
    public double armPositionRad;
    public double armVelocityRadPerSec;
    public double armAppliedVolts;
    public double armCurrentAmps;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIoInputs inputs) {}

  public default void setArmOutput(double percent) {}

  /** Run the arm at the specified voltage. */
  public default void setArmVoltage(double volts) {}

  /** Run closed loop to the specified position. */
  public default void setArmPosition(double position, double ffVolts) {}

  /** Stop aimer in open loop. */
  public default void stop() {}

  /** Set feed back constants. */
  public default void configurePid(double p, double i, double d) {}

  /** Set feed forward constants. */
  public default void configureFeedForward(double p, double i, double d) {}

  public default void configureLimits(double forwardLimit, double backwardLimit) {}
}
