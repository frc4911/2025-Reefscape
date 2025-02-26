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
      case REAL:
        return realProvider.get();
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
        .gearRatio(9)
        .variance(.1)
        .debounceTimeSeconds(.25)
        .armEncoderOffsetRads(-1.83)
        .minPositionRads(-Math.PI / 2.0)
        .maxPositionRads(Math.PI / 2.0)
        .stowPositionDegrees(60)
        .collectPositionDegrees(-90)
        .scorePositionDegrees(0)
        .troughPositionDegrees(0)
        .levelTwoAndThreePositionDegrees(57)
        .levelFourPositionDegrees(55)
        .coralDetectionDistanceMillimeters(50.0)
        .feedBackValues(new PidValues(1500, 0, 100))
        .feedForwardValues(new FeedForwardValues(0, 0, 0, 0))
        .build();
  }
}
