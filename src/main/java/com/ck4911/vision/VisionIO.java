// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public interface VisionIO {
  public final PhotonCamera camera = new PhotonCamera("grant's_spy_camera");
  public final Transform3d robotToCam =
      new Transform3d(
          new Translation3d(0.5, 0.0, 0.5),
          new Rotation3d(
              0, 0,
              0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center.
  public final PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          VisionModule.providesFieldLayout(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);

  @AutoLog
  class VisionIOInputs {
    public Pose3d estimatedPose = new Pose3d();
    public Pose2d estimatedPose2d = new Pose2d();
    public double timestampSeconds;
    public int[] targetIds;
    public boolean poseUpdated;
  }

  public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update(camera.getLatestResult());
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
