// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.drive;

import com.ck4911.vision.VisionConsumer;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;
import javax.inject.Singleton;

@Module
public interface DriveModule {

  @Provides
  @Singleton
  public static Drive providesDrive() {
    return TunerConstants.createDrivetrain();
  }

  @Binds
  public VisionConsumer bindsVisionConsumer(Drive drive);
}
