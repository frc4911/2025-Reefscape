// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  class VisionIOInputs {
    public Pose3d estimatedPose = new Pose3d();
    public Pose2d estimatedPose2d = new Pose2d();
    public double timestampSeconds;
    public int[] targetIds;
    public boolean poseUpdated;
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
