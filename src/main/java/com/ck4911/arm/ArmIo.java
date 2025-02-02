// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
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
  public default void runVolts(Voltage voltage) {}

  /** Stop aimer in open loop. */
  public default void stop() {}

  /** Set feed back constants. */
  public default void configurePid(double p, double i, double d) {}

  public default void setArmPosition(Angle angle) {}
}
