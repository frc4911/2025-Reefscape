// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ck4911.commands.VirtualSubsystem;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

@Module
public interface VisionModule {
  // Note: These are absolute locations. For directions, refer to:
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
  // Note: 11.36 is the X and Y distance of each swerve azimuth axis from robot center
  // Distance forward of center
  double SWERVE_MOUNTED_CAMERA_OFFSET_X = 11.36 - 1.00;
  // Distance left or right of center
  double SWERVE_MOUNTED_CAMERA_OFFSET_Y = 11.36 + 0.25;
  // Distance from floor
  double SWERVE_MOUNTED_CAMERA_OFFSET_Z = 6.0 + 2.24;
  // Angle tilted back
  double SWERVE_MOUNTED_CAMERA_PITCH = 61.875 - 90.0;
  // Angle turned outward
  double SWERVE_MOUNTED_CAMERA_YAW = 34.709;

  @Provides
  static AprilTagFieldLayout providesFieldLayout() {
    return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
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

  @Provides
  @IntoSet
  static CameraConstants providesCameraFrontRight() {
    return CameraConstantsBuilder.builder()
        .name("front_right")
        .robotToCamera(
            new Transform3d(
                new Translation3d(
                    Inches.of(SWERVE_MOUNTED_CAMERA_OFFSET_X),
                    Inches.of(-SWERVE_MOUNTED_CAMERA_OFFSET_Y),
                    Inches.of(SWERVE_MOUNTED_CAMERA_OFFSET_Z)),
                new Rotation3d(
                        Degrees.zero(), Degrees.of(SWERVE_MOUNTED_CAMERA_PITCH), Degrees.zero())
                    .rotateBy(
                        new Rotation3d(
                            Degrees.zero(),
                            Degrees.zero(),
                            Degrees.of(-SWERVE_MOUNTED_CAMERA_YAW)))))
        .cameraStdDevFactor(1.0)
        .build();
  }

  @Provides
  @IntoSet
  static CameraConstants providesCameraFrontLeft() {
    return CameraConstantsBuilder.builder()
        .name("front_left")
        .robotToCamera(
            new Transform3d(
                new Translation3d(
                    Inches.of(SWERVE_MOUNTED_CAMERA_OFFSET_X),
                    Inches.of(SWERVE_MOUNTED_CAMERA_OFFSET_Y),
                    Inches.of(SWERVE_MOUNTED_CAMERA_OFFSET_Z)),
                new Rotation3d(
                        Degrees.zero(), Degrees.of(SWERVE_MOUNTED_CAMERA_PITCH), Degrees.zero())
                    .rotateBy(
                        new Rotation3d(
                            Degrees.zero(),
                            Degrees.zero(),
                            Degrees.of(SWERVE_MOUNTED_CAMERA_YAW)))))
        .cameraStdDevFactor(1.0)
        .build();
  }

  @Binds
  @IntoSet
  VirtualSubsystem bindsVision(Vision vision);

  @Provides
  static VisionConstants providesVisionConstants() {
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
