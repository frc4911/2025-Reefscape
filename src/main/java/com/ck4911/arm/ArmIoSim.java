// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.arm;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class ArmIoSim implements ArmIo {

  @Inject
  ArmIoSim(ArmConstants constants) {}

  @Override
  public void stop() {}

  @Override
  public void setPid(double p, double i, double d) {}
}
