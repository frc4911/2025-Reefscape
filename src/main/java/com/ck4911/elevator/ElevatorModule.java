// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import com.ck4911.Constants.Mode;
import dagger.Module;
import dagger.Provides;
import javax.inject.Provider;

@Module
public interface ElevatorModule {

  @Provides
  static ElevatorIo providesArmIo(
      Mode mode, Provider<ElevatorIoReal> realProvider, Provider<ElevatorIoSim> simProvider) {
    switch (mode) {
        //   case REAL:
        //     return realProvider.get();
      case SIM:
        return simProvider.get();
      default:
        return new ElevatorIo() {};
    }
  }

  @Provides
  static ElevatorConstants provideElevatorConstants() {
    // TODO: Fill in the constants
    return new ElevatorConstants(20, 21, 1.0, null, null);
  }
}
