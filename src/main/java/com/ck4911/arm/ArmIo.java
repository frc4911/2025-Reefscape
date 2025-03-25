// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.arm;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIo {
  @AutoLog
  public static class ArmIoInputs {
    public boolean motorConnected = true;
    public boolean absoluteEncoderConnected = true;

    public double positionRads;
    public double absoluteEncoderPositionRads;
    public double velocityRadsPerSec;
    public double appliedVolts;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelcius;

    public boolean sensorConnected;
    public int sensorStatus;
    public int sensorAmbient;
    public int sensorDistanceMillimeters;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIoInputs inputs) {}

  /** Run the arm at the specified voltage. */
  public default void runVolts(Voltage voltage) {}

  public default void runPosition(Angle position, Current feedforward) {}

  public default void runCurrent(Current amps) {}

  default void setBrakeMode(boolean enabled) {}

  /** Stop aimer in open loop. */
  public default void stop() {}

  /** Set feed back constants. */
  public default void setPid(double p, double i, double d) {}

  public default void setFeedForward(double s, double g, double v, double a) {}

  public default void setProfile(
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk) {}
}
