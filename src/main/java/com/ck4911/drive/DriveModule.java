// Copyright (c) 2025 FRC 4911
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
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Consumer;
import java.util.function.Supplier;

@Module
public interface DriveModule {

  @Binds
  public VisionConsumer bindsVisionConsumer(Drive drive);

  @Provides
  public static Supplier<Pose2d> providesPoseSupplier(Drive drive) {
    return () -> drive.getState().Pose;
  }

  @Provides
  public static Consumer<Pose2d> providesPoseConsumer(Drive drive) {
    return drive::resetPose;
  }
}
