// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.quest;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import javax.inject.Inject;

public class QuestIoReal implements QuestIo {

  // Configure Network Tables topics (questnav/...) to communicate with the Quest
  private final NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  private final NetworkTable nt4Table = nt4Instance.getTable("questnav");

  // Quest "output"
  private final IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();
  private final DoubleArrayPublisher questResetPose = nt4Table.getDoubleArrayTopic("resetpose").publish();

  // Quest "input"
  private final IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private final DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0);
  private final FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private final FloatArraySubscriber questEulerAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private final DoubleSubscriber questBattery = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0);

  private final Debouncer connectedDebouncer;
  private final QuestConstants constants;
  private Transform2d robotToQuest = new Transform2d();

  @Inject
  public QuestIoReal(QuestConstants constants) {
    this.constants = constants;
    connectedDebouncer = new Debouncer(.5);
  }

  public void updateInputs(QuestIoInputs inputs) {
    inputs.questPose = getQuestPose();
    inputs.robotPose = getRobotPose();

    double timestamp = questTimestamp.get();
    inputs.timestampDelta = timestamp - inputs.timestamp;
    inputs.timestamp = timestamp;

    inputs.batteryLevel = questBattery.get();

    // The timestamp delta is calculated between the current and last robot loop
    // The delta is zero if the new measurement is from the same time as the last measurement,
    // meaning we have not received new data and as such can assume the quest is not connected
    inputs.connected = connectedDebouncer.calculate(inputs.timestampDelta != 0);
    completeQuestPose();
  }

  public double getRawX() {
    return questPosition.get()[2];
  }

  public double getRawY() {
    return -questPosition.get()[0];
  }

  public double getRawZ() {
    return questPosition.get()[1];
  }

  public static double stabilize(double angle) {
    return angle >= 180
        ? angle - ((int) ((angle - 180) / 360)) * 360 - 360
        : angle <= -180 ? angle - ((int) ((angle + 180) / 360)) * 360 + 360 : angle;
  }

  public double getRawPitch() {
    return stabilize(questEulerAngles.get()[0]);
  }

  public double getRawYaw() {
    return stabilize(-questEulerAngles.get()[1]);
  }

  public double getRawRoll() {
    return stabilize(questEulerAngles.get()[2]);
  }

  public Pose2d getQuestPose() {
    return new Pose2d(
        new Translation2d(getRawX(), getRawY()),
        new Rotation2d(Units.degreesToRadians(getRawYaw())));
  }

  public Pose2d getRobotPose() {
    return getQuestPose().plus(robotToQuest.inverse());
  }

  public void completeQuestPose() {
    if (questMiso.get() == 98 || questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  @Override
  public void updateRobotToQuest(Transform2d robotToQuest) {
    this.robotToQuest = robotToQuest;
  }

  @Override
  public void resetPose(Pose2d robotPose) {
    Pose2d correctedPose = robotPose.plus(robotToQuest);
    if (questMiso.get() != 98) {
      questResetPose.set(
          new double[] {
            correctedPose.getX(), correctedPose.getY(), correctedPose.getRotation().getDegrees()
          });
      questMosi.set(2);
    }
  }
}
