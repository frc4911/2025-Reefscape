// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestIo {
  @AutoLog
  public static class QuestIoInputs {
    public boolean connected = false;

    // These are with relative with offsets applied
    public Pose2d questPose = new Pose2d();
    public Pose2d robotPose = new Pose2d();

    public double timestamp = 0;
    public double timestampDelta = 0;
    public double batteryLevel = 0;
  }

  public default void updateInputs(QuestIoInputs inputs) {}

  /** Sets supplied pose as origin of all calculations */
  public default void resetPose(Pose2d pose) {}

  public default void updateRobotToQuest(Transform2d robotToQuest) {}
}
