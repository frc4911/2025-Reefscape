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
import dagger.multibindings.IntoSet;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

@Module
public interface VisionModule {

  @Provides
  public static AprilTagFieldLayout providesFieldLayout() {
    return AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  }

  @Provides
  public static List<CameraConfig> providesVisionIos(
      AprilTagFieldLayout aprilTagFieldLayout, Set<CameraConstants> cameraConstants) {
    return cameraConstants.stream()
        .map(
            (constants) ->
                new CameraConfig(
                    constants,
                    new VisionIoInputsAutoLogged(),
                    new VisionIoPhotonVision(
                        constants.name(), constants.robotToCamera(), aprilTagFieldLayout),
                    new Alert(
                        "Vision camera " + constants.name() + " is disconnected.",
                        Alert.AlertType.kWarning)))
        .collect(Collectors.toList());
  }

  // TODO: add more cameras
  @Provides
  @IntoSet
  public static CameraConstants providesCameraFrontRight() {
    return CameraConstantsBuilder.builder()
        .name("front_right")
        .robotToCamera(new Transform3d())
        .cameraStdDevFactor(1.0)
        .build();
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
        .linearStdDevMegatag2Factor(0.5)
        .angularStdDevMegatag2Factor(Double.POSITIVE_INFINITY)
        .build();
  }
}
