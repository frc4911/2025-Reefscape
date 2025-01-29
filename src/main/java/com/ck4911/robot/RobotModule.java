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
  public static @Named("RobotName") String providesRobotName() {
    return "ReefScapeRobotName";
  }

  @Provides
  public static @Named("TuningMode") boolean providesTuningMode() {
    // TODO: toggle this for competition
    return true;
  }

  @Provides
  public static @Named("LoopPeriod") double providesLoopPeriod() {
    return 0.02;
  }

  @Provides
  @Singleton
  public static @Named("rio") CANBus providesRioBus() {
    return new CANBus("rio");
  }

  @Provides
  @Singleton
  public static @Named("Bob") CANBus providesCanivoreBus() {
    return new CANBus("Bob");
  }

  @Provides
  public static Mode providesMode() {
    return Mode.REAL;
  }

  @Provides
  public static CommandScheduler providesCommandScheduler() {
    return CommandScheduler.getInstance();
  }
}
