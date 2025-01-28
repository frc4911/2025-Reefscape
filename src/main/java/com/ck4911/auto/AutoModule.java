// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import choreo.auto.AutoFactory;
import com.ck4911.Constants.Mode;
import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.drive.Drive;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

@Module
public interface AutoModule {
  @Provides
  public static AutoConstants provideAutoConstants(Mode mode) {
    return AutoConstantsBuilder.builder().xkD(10.0).ykD(10.0).thetakD(7.0).build();
  }

  @Binds
  @IntoSet
  public VirtualSubsystem bindsAutoCommandHandler(AutoCommandHandler autoCommandHandler);

  @Provides
  public static AutoFactory provideAutoFactory(TrajectoryFollower trajectoryFollower, Drive drive) {
    return new AutoFactory(
        () -> drive.getState().Pose,
        drive::resetPose,
        trajectoryFollower::follow,
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red,
        drive);
  }
}
