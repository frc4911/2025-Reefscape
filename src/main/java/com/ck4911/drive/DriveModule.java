// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.drive;

import com.ck4911.Constants.Mode;
import com.ck4911.vision.VisionConsumer;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntKey;
import dagger.multibindings.IntoMap;
import dagger.multibindings.Multibinds;
import java.util.Map;
import javax.inject.Provider;

@Module
public interface DriveModule {

  @Provides
  public static GyroIO providesGyroIO(Mode mode, Provider<GyroIO> realProvider) {
    switch (mode) {
      case REAL:
        // TODO: enable this when gyro is set up
        // return realProvider.get();
        return new GyroIO() {};
      case SIM:
      default:
        return new GyroIO() {};
    }
  }

  // FL, FR, BL, BR
  @Provides
  @IntoMap
  @IntKey(0) // FL
  public static ModuleIO providesFrontLeftModule(Mode mode) {
    switch (mode) {
      case REAL:
        // TODO: enable this when swerve is set up
        // return new ModuleIOTalonFX(0);
        return new ModuleIO() {};
      case SIM:
        return new ModuleIOSim();
      default:
        return new ModuleIO() {};
    }
  }

  @Provides
  @IntoMap
  @IntKey(1) // FR
  public static ModuleIO providesFrontRightModule(Mode mode) {
    switch (mode) {
      case REAL:
        // TODO: enable this when swerve is set up
        // return new ModuleIOTalonFX(1);
        return new ModuleIO() {};
      case SIM:
        return new ModuleIOSim();
      default:
        return new ModuleIO() {};
    }
  }

  @Provides
  @IntoMap
  @IntKey(2) // BL
  public static ModuleIO providesBackLeftModule(Mode mode) {
    switch (mode) {
      case REAL:
        // TODO: enable this when swerve is set up
        // return new ModuleIOTalonFX(2);
        return new ModuleIO() {};
      case SIM:
        return new ModuleIOSim();
      default:
        return new ModuleIO() {};
    }
  }

  @Provides
  @IntoMap
  @IntKey(3) // BR
  public static ModuleIO providesBackRightModule(Mode mode) {
    switch (mode) {
      case REAL:
        // TODO: enable this when swerve is set up
        // return new ModuleIOTalonFX(3);
        return new ModuleIO() {};
      case SIM:
        return new ModuleIOSim();
      default:
        return new ModuleIO() {};
    }
  }

  @Multibinds
  public abstract Map<Integer, Module> providesModules();

  @Binds
  public VisionConsumer bindsVisionConsumer(Drive drive);
}
