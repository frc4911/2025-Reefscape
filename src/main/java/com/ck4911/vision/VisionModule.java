// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.vision;

import com.ck4911.commands.VirtualSubsystem;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;
import dagger.multibindings.Multibinds;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Map;

@Module
public interface VisionModule {

  @Provides
  public static AprilTagFieldLayout providesFieldLayout() {
    return AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  }

  @Provides
  public static InterpolatingMatrixTreeMap<Double, N3, N1> providesMeasurementStdDevDistanceMap() {
    InterpolatingMatrixTreeMap<Double, N3, N1> map = new InterpolatingMatrixTreeMap<>();
    map.put(1.0, VecBuilder.fill(1.0, 1.0, 1.0));
    map.put(8.0, VecBuilder.fill(10.0, 10.0, 10.0));
    return map;
  }

  @Multibinds
  public abstract Map<String, VisionIO> providesEmptyMap();

  @Binds
  @IntoSet
  public VirtualSubsystem bindsVision(Vision vision);
}
