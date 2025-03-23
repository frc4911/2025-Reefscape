// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.collector;

import com.ck4911.util.LoggedTunableNumber;
import com.ck4911.util.LoggedTunableNumber.TunableNumbers;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class Collector extends SubsystemBase {

  private final CollectorIo collectorIo;
  //    private final CollectorIoInputsAutoLogged inputs = new CollectorIoInputsAutoLogged();
  private final LoggedTunableNumber pP;
  private final LoggedTunableNumber pI;
  private final LoggedTunableNumber pD;
  private final LoggedTunableNumber rP;
  private final LoggedTunableNumber rI;
  private final LoggedTunableNumber rD;
  private final CollectorConstants constants;

  @Inject
  public Collector(
      CollectorConstants constants, CollectorIo collectorIo, TunableNumbers tunableNumbers) {
    super();
    this.collectorIo = collectorIo;
    this.constants = constants;

    pP = tunableNumbers.create("Collector/pP", constants.pivotFeedBackValues().p());
    pI = tunableNumbers.create("Collector/pI", constants.pivotFeedBackValues().i());
    pD = tunableNumbers.create("Collector/pD", constants.pivotFeedBackValues().d());

    rP = tunableNumbers.create("Collector/rP", constants.rollerFeedBackValues().p());
    rI = tunableNumbers.create("Collector/rI", constants.rollerFeedBackValues().i());
    rD = tunableNumbers.create("Collector/rD", constants.rollerFeedBackValues().d());
  }

  @Override
  public void periodic() {}
}
