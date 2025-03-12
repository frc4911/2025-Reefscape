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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Consumer;
import java.util.function.Supplier;
import javax.inject.Singleton;

@Module
public interface AutoModule {
  @Provides
  @Singleton
  static AutoConstants provideAutoConstants(Mode mode) {
    return AutoConstantsBuilder.builder()
        .feedback(new PidValues(10.0, 0, 0))
        .thetaFeedback(new PidValues(7.0, 0, 0))
        .build();
  }

  @Binds
  @IntoSet
  VirtualSubsystem bindsAutoCommandHandler(AutoCommandHandler autoCommandHandler);

  @Provides
  @Singleton
  static AutoFactory provideAutoFactory(
          Supplier<Pose2d> poseSupplier,
          Consumer<Pose2d> poseConsumer,
          TrajectoryFollower trajectoryFollower,
          Drive drive) {
    return new AutoFactory(
        poseSupplier,
        poseConsumer,
        trajectoryFollower::follow,
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red,
        drive);
  }

  @Provides
  @Singleton
  static AutoChooser provideAutoChooser() {
    return new AutoChooser();
  }

  @Binds
  @IntoSet
  VirtualSubsystem bindsTrajectoryFolloer(TrajectoryFollower trajectoryFollower);
}
