// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import com.ck4911.Constants.Mode;
import com.ck4911.util.FeedForwardValues;
import com.ck4911.util.PidValues;
import dagger.Module;
import dagger.Provides;
import javax.inject.Provider;

@Module
public interface ElevatorModule {

  @Provides
  static ElevatorIo providesArmIo(
      Mode mode, Provider<ElevatorIoReal> realProvider, Provider<ElevatorIoSim> simProvider) {
    switch (mode) {
      case REAL:
        return realProvider.get();
      case SIM:
        return simProvider.get();
      default:
        return new ElevatorIo() {};
    }
  }

  @Provides
  static ElevatorConstants provideElevatorConstants() {
    // TODO: Fill in the constants
    return ElevatorConstantsBuilder.builder()
        .motorLeftId(21)
        .motorRightId(22)
        .corralHeight(13.5)
        .sprocketRadius(1.7567 / 2.0)
        .gearRatio(4.0 * 3.0)
        .variance(.01)
        .debounceTimeSeconds(0.25)
        .homingTimeSeconds(0.25)
        .homingVolts(-1.5)
        .homingVelocityThresh(5.0)
        .tolerance(0.01) // 1%
        .minPositionRads(0)
        .maxPositionRads(0)
        .stowPositionRadians(5)
        .collectPositionRadians(9.24)
        .prepareCollectPositionRadians(16.5)
        .troughPositionRadians(7)
        .levelTwoPositionRadians(10)
        .levelThreePositionRadians(20)
        .levelFourPositionRadians(35)
        .feedBackValues(new PidValues(64, 0, 5))
        .feedForwardValues(new FeedForwardValues(0, 0, 0, 0))
        .build();
  }
}
