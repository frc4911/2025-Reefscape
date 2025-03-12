// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.robot;

import com.ck4911.Constants.Mode;
import com.ctre.phoenix6.CANBus;
import dagger.Module;
import dagger.Provides;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import javax.inject.Named;
import javax.inject.Singleton;

@Module
public interface RobotModule {

  @Provides
  static @Named("RobotName") String providesRobotName() {
    return "ReefScapeRobotName";
  }

  @Provides
  static @Named("TuningMode") boolean providesTuningMode() {
    // TODO: toggle this off for competition
    return true;
  }

  @Provides
  static @Named("LoopPeriod") double providesLoopPeriod() {
    return 0.02;
  }

  @Provides
  @Singleton
  static @Named("rio") CANBus providesRioBus() {
    return new CANBus("rio");
  }

  @Provides
  @Singleton
  static @Named("Bob") CANBus providesCanivoreBus() {
    return new CANBus("Bob");
  }

  @Provides
  static Mode providesMode() {
    return Mode.REAL;
  }

  @Provides
  static CommandScheduler providesCommandScheduler() {
    return CommandScheduler.getInstance();
  }
}
