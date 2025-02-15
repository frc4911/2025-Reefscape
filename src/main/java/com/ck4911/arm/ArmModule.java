// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.arm;

import com.ck4911.Constants.Mode;
import com.ck4911.util.FeedForwardValues;
import com.ck4911.util.PidValues;
import dagger.Module;
import dagger.Provides;
import javax.inject.Provider;

@Module
public interface ArmModule {

  @Provides
  static ArmIo providesArmIo(
      Mode mode, Provider<ArmIoReal> realProvider, Provider<ArmIoSim> simProvider) {
    switch (mode) {
        // case REAL:
        // return realProvider.get();
      case SIM:
        return simProvider.get();
      default:
        return new ArmIo() {};
    }
  }

  @Provides
  static ArmConstants provideArmConstants() {
    // TODO: Fill in the constants
    return ArmConstantsBuilder.builder()
        .motorId(10)
        .encoderId(11)
        .sensorId(12)
        .inverted(false)
        .gearRatio(1)
        .armEncoderOffsetRads(1)
        .collectPositionDegrees(1)
        .troughPositionDegrees(1)
        .levelTwoAndThreePositionDegrees(1)
        .levelFourPositionDegrees(1)
        .feedBackValues(new PidValues(0, 0, 0))
        .feedForwardValues(new FeedForwardValues(0, 0, 0, 0))
        .build();
  }
}
