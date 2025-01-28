// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class Arm extends SubsystemBase {

  private final ArmIo armIo;
  private final ArmIoInputsAutoLogged inputs = new ArmIoInputsAutoLogged();

  @Inject
  public Arm(ArmConstants constants, ArmIo armIo) {
    super();
    this.armIo = armIo;
  }
}
