// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ck4911.Constants.Mode;
import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.drive.Drive;
import com.ck4911.util.PidValues;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Consumer;
import java.util.function.Supplier;
import javax.inject.Singleton;

@Module
public interface AutoModule {
  @Provides
  @Singleton
  public static AutoConstants provideAutoConstants(Mode mode) {
    return AutoConstantsBuilder.builder()
        .feedback(new PidValues(10.0, 0, 0))
        .thetaFeedback(new PidValues(7.0, 0, 0))
        .build();
  }

  @Binds
  @IntoSet
  public VirtualSubsystem bindsAutoCommandHandler(AutoCommandHandler autoCommandHandler);

  @Provides
  @Singleton
  public static AutoFactory provideAutoFactory(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> poseConsumer,
      TrajectoryFollower trajectoryFollower,
      Drive drive) {
    return new AutoFactory(poseSupplier, poseConsumer, trajectoryFollower::follow, true, drive);
  }

  @Provides
  @Singleton
  public static AutoChooser provideAutoChooser() {
    return new AutoChooser();
  }

  @Binds
  @IntoSet
  public VirtualSubsystem bindsTrajectoryFollower(TrajectoryFollower trajectoryFollower);
}
