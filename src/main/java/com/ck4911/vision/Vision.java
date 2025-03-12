// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.vision;

import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.vision.VisionIO.PoseObservationType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public final class Vision implements VirtualSubsystem {
  private final VisionConsumer consumer;
  private final List<VisionIO> io;
  private final List<VisionIOInputsAutoLogged> inputs;
  private final List<Alert> disconnectedAlerts;
  private final AprilTagFieldLayout aprilTagLayout;
  private final VisionConstants constants;

  @Inject
  public Vision(
      VisionConsumer consumer,
      VisionConstants constants,
      AprilTagFieldLayout aprilTagLayout,
      Map<String, VisionIO> ios) {
    this.consumer = consumer;
    this.constants = constants;
    this.aprilTagLayout = aprilTagLayout;
    this.io = new ArrayList<>();
    this.inputs = new ArrayList<>();
    this.disconnectedAlerts = new ArrayList<>();
    for (Entry<String, VisionIO> entry : ios.entrySet()) {
      this.io.add(entry.getValue());
      inputs.add(new VisionIOInputsAutoLogged());
      new Alert("Vision camera " + entry.getKey() + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs.get(cameraIndex).latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.size(); i++) {
      io.get(i).updateInputs(inputs.get(i));
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs.get(i));
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.size(); cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts.get(cameraIndex).set(!inputs.get(cameraIndex).connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs.get(cameraIndex).tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs.get(cameraIndex).poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > constants.maxAmbiguity()) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > constants.maxZError() // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = constants.linearStdDevBaseline() * stdDevFactor;
        double angularStdDev = constants.angularStdDevBaseline() * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= constants.linearStdDevMegatag2Factor();
          angularStdDev *= constants.angularStdDevMegatag2Factor();
        }
        if (cameraIndex < constants.cameraStdDevFactors().length) {
          linearStdDev *= constants.cameraStdDevFactors()[cameraIndex];
          angularStdDev *= constants.cameraStdDevFactors()[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }
}
