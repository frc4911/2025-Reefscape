// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.vision;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

@Module
public interface VisionModule {
  // Note: These are absolute locations. For directions, refer to:
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
  // Note: 11.36 is the X and Y distance of each swerve azimuth axis from robot center
  Distance SWERVE_MOUNTED_CAMERA_OFFSET_X = Inches.of(11.36 - .82);
  Distance SWERVE_MOUNTED_CAMERA_OFFSET_Y = Inches.of(11.36 - 1.06);
  Distance SWERVE_MOUNTED_CAMERA_OFFSET_Z = Inches.of(6.0 + 2.24);
  Angle SWERVE_MOUNTED_CAMERA_PITCH = Radians.of(0.490873852123);
  Angle SWERVE_MOUNTED_CAMERA_YAW = Radians.of(1.0471975512);

  @Provides
  public static AprilTagFieldLayout providesFieldLayout() {
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

  // TODO: add more cameras
  @Provides
  @IntoSet
  public static CameraConstants providesCameraFrontRight() {
    return CameraConstantsBuilder.builder()
        .name("front_right")
        .robotToCamera(
            new Transform3d(
                new Translation3d(
                    SWERVE_MOUNTED_CAMERA_OFFSET_X,
                    SWERVE_MOUNTED_CAMERA_OFFSET_Y.unaryMinus(),
                    SWERVE_MOUNTED_CAMERA_OFFSET_Z),
                new Rotation3d(
                    Degrees.zero(),
                    SWERVE_MOUNTED_CAMERA_PITCH.unaryMinus(),
                    SWERVE_MOUNTED_CAMERA_YAW.unaryMinus())))
        .cameraStdDevFactor(1.0)
        .build();
  }

  @Provides
  @IntoSet
  public static CameraConstants providesCameraFrontLeft() {
    return CameraConstantsBuilder.builder()
        .name("front_left")
        .robotToCamera(
            new Transform3d(
                new Translation3d(
                    SWERVE_MOUNTED_CAMERA_OFFSET_X,
                    SWERVE_MOUNTED_CAMERA_OFFSET_Y.unaryMinus(),
                    SWERVE_MOUNTED_CAMERA_OFFSET_Z),
                new Rotation3d(
                    Degrees.zero(), SWERVE_MOUNTED_CAMERA_PITCH, SWERVE_MOUNTED_CAMERA_YAW)))
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
