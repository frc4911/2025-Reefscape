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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import javax.inject.Named;
import javax.inject.Provider;
import javax.inject.Singleton;

@Module
public interface ElevatorModule {

  @Provides
  static ElevatorIo providesArmIo(
      Mode mode, Provider<ElevatorIoReal> realProvider, Provider<ElevatorIoSim> simProvider) {
    return switch (mode) {
      case REAL -> realProvider.get();
      case SIM -> simProvider.get();
      default -> new ElevatorIo() {};
    };
  }

  @Singleton
  @Provides
  @Named("Elevator")
  static MechanismLigament2d providesElevatorMechanism() {
    Mechanism2d mechanism2d = new Mechanism2d(3, 3);
    MechanismRoot2d root = mechanism2d.getRoot("Chassis", 1.5, 0.15);
    SmartDashboard.putData("Mechanism", mechanism2d);
    return root.append(
        new MechanismLigament2d("Elevator", 0, 90, 6, new Color8Bit(Color.kIndianRed)));
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
