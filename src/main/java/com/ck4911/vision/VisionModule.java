// Copyright (c) 2025 FRC 4911
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
import dagger.multibindings.IntoMap;
import dagger.multibindings.IntoSet;
import dagger.multibindings.StringKey;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

@Module
public interface VisionModule {

  @Provides
  public static AprilTagFieldLayout providesFieldLayout() {
    return AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  }

  @Provides
  @IntoMap
  @StringKey("FrontRight")
  public static VisionIO providesVisionIos(AprilTagFieldLayout aprilTagFieldLayout) {
    return new VisionIOPhotonVision("FrontRight", new Transform3d(), aprilTagFieldLayout);
  }

  @Binds
  @IntoSet
  public VirtualSubsystem bindsVision(Vision vision);

  @Provides
  public static VisionConstants providesVisionConstants() {
    return VisionConstantsBuilder.builder()
        .maxAmbiguity(0.3)
        .maxZError(0.75)
        .linearStdDevBaseline(0.02)
        .angularStdDevBaseline(0.06)
        .cameraStdDevFactors(
            new double[] {
              1.0, // Camera 0
              1.0 // Camera 1
            })
        .linearStdDevMegatag2Factor(0.5)
        .angularStdDevMegatag2Factor(Double.POSITIVE_INFINITY)
        .build();
  }

  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };
}
