// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIo {
  @AutoLog
  class IndexerIoInputs {
    public boolean motorConnected = true;

    public double positionRads;
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
  default void updateInputs(IndexerIo.IndexerIoInputs inputs) {}

  default void runVelocity(AngularVelocity velocity, Current feedforward) {}

  /** Set feed back constants. */
  default void setPid(double p, double i, double d) {}
}
