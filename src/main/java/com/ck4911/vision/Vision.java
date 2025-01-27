// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.vision;

import com.ck4911.commands.VirtualSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.stream.Collectors;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public final class Vision implements VirtualSubsystem {
  private static final double MAXIMUM_ACCEPTED_MEASUREMENT_Z = 1.0;

  private final AprilTagFieldLayout fieldLayout;
  private final VisionConsumer visionConsumer;
  private final InterpolatingMatrixTreeMap<Double, N3, N1> measurementStdDevDistanceMap;
  private final Map<String, VisionIO> iosMap;
  private final Map<String, VisionIOInputsAutoLogged> visionInputs;

  @Inject
  public Vision(
      AprilTagFieldLayout fieldLayout,
      Map<String, VisionIO> iosMap,
      VisionConsumer visionConsumer,
      InterpolatingMatrixTreeMap<Double, N3, N1> measurementStdDevDistanceMap) {
    this.fieldLayout = fieldLayout;
    this.visionConsumer = visionConsumer;
    this.measurementStdDevDistanceMap = measurementStdDevDistanceMap;
    this.iosMap = iosMap;

    visionInputs =
        iosMap.keySet().stream()
            .collect(Collectors.toMap(key -> key, key -> new VisionIOInputsAutoLogged()));
  }

  @Override
  public void periodic() {
    iosMap.keySet().stream()
        .forEach(
            key -> {
              var inputs = visionInputs.get(key);
              iosMap.get(key).updateInputs(inputs);
              Logger.processInputs("Vision/" + key, inputs);

              Pose3d pose = inputs.estimatedPose;
              double timestamp = inputs.timestampSeconds;

              // Skip inputs that haven't updated
              if (!inputs.poseUpdated) {
                return;
              }

              // Skip measurements that are not with in the field boundary
              if (pose.getX() < 0.0
                  || pose.getX() > fieldLayout.getFieldLength()
                  || pose.getY() < 0.0
                  || pose.getY() > fieldLayout.getFieldWidth()) {
                return;
              }
              // Skip measurements that are floating
              if (pose.getZ() > MAXIMUM_ACCEPTED_MEASUREMENT_Z) {
                return;
              }

              // Compute the standard deviation to use based on the distance to the closest tag
              OptionalDouble closestTagDistance =
                  Arrays.stream(inputs.targetIds)
                      .mapToObj(fieldLayout::getTagPose)
                      .filter(Optional::isPresent)
                      .mapToDouble(
                          tagPose ->
                              tagPose.get().getTranslation().getDistance(pose.getTranslation()))
                      .min();
              // If for some reason we were unable to calculate the distance to the closest tag,
              // assume we
              // are infinitely far away
              Matrix<N3, N1> stdDevs =
                  measurementStdDevDistanceMap.get(closestTagDistance.orElse(Double.MAX_VALUE));

              visionConsumer.accept(timestamp, pose.toPose2d(), stdDevs);
            });
  }
}
