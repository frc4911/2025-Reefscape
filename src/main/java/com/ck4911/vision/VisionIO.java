// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  class VisionIOInputs {
    public Pose3d estimatedPose = new Pose3d();
    public Pose2d estimatedPose2d = new Pose2d();
    public double timestampSeconds;
    public int[] targetIds;
    public boolean connected;
    public TargetObservation latestTargetObservation;
    public boolean poseUpdated;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}
