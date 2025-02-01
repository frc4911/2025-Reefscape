// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIo {
  public default void runVolts(Voltage voltage) {}

  public default void moveElevevator(double rotations) {}
}
