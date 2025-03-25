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
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import javax.inject.Named;
import javax.inject.Provider;
import javax.inject.Singleton;

@Module
public interface ArmModule {

  @Provides
  static ArmIo providesArmIo(
      Mode mode, Provider<ArmIoReal> realProvider, Provider<ArmIoSim> simProvider) {
    return switch (mode) {
      case REAL -> realProvider.get();
      case SIM -> simProvider.get();
      default -> new ArmIo() {};
    };
  }

  @Singleton
  @Provides
  @Named("Arm")
  static MechanismLigament2d providesArmMechanism(@Named("Elevator") MechanismLigament2d elevator) {
    return elevator.append(
        new MechanismLigament2d("Arm", 0.5, 90, 6, new Color8Bit(Color.kAquamarine)));
  }

  @Provides
  static ArmConstants provideArmConstants() {
    // TODO: Fill in the constants
    return ArmConstantsBuilder.builder()
        .motorId(10)
        .encoderId(11)
        .sensorId(1)
        .gearRatio(25)
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
        .coralDetectionDistanceMillimeters(20.0)
        .coralScoreDistanceMillimeters(90.0)
        .feedBackValues(new PidValues(500, 0, 75))
        .feedForwardValues(new FeedForwardValues(0, 0, 0, 0))
        .build();
  }
}
