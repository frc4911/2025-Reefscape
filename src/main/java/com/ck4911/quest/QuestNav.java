// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.quest;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.util.Alert;
import com.ck4911.util.Alert.AlertType;
import com.ck4911.util.LoggedTunableNumber;
import com.ck4911.util.LoggedTunableNumber.TunableNumbers;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public final class QuestNav implements VirtualSubsystem {
  private final Alert disconnectAlert = new Alert("Quest is disconnected", AlertType.WARNING);
  private final Alert batteryAlert = new Alert("Quest battery is low", AlertType.WARNING);

  private final LoggedTunableNumber questOffsetAngleDegrees;
  private final LoggedTunableNumber questOffsetXInches;
  private final LoggedTunableNumber questOffsetYInches;

  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  private final NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  private final NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private final IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private final IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();
  // Format: [X, Y, Rotation] in FRC field coordinates
  private final DoubleArrayPublisher questResetPose =
      nt4Table.getDoubleArrayTopic("resetpose").publish();

  // Subscribe to the Network Tables questnav data topics
  private final DoubleSubscriber questTimestamp =
      nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private final FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private final FloatArraySubscriber questQuaternion =
      nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
  private final FloatArraySubscriber questEulerAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private final DoubleSubscriber questBatteryPercent =
      nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  private Debouncer disconnectDebouncer = new Debouncer(5);

  // Position of the quest on the robot
  private Transform2d robotToQuest;
  private Transform2d fieldOffset = new Transform2d();

  @Inject
  QuestNav(QuestConstants questConstants, TunableNumbers tunableNumbers) {
    questOffsetAngleDegrees =
        tunableNumbers.create("Quest/offsetAngleDegrees", questConstants.offsetAngleDegrees());
    questOffsetXInches =
        tunableNumbers.create("Quest/offsetXInches", questConstants.offsetXInches());
    questOffsetYInches =
        tunableNumbers.create("Quest/offsetYInches", questConstants.offsetYInches());

    updateRobotToQuest();
    cleanUpQuestNavMessages();
  }

  private void updateRobotToQuest() {
    robotToQuest =
        new Transform2d(
            Inches.of(questOffsetXInches.get()),
            Inches.of(questOffsetYInches.get()),
            new Rotation2d(Degrees.of(questOffsetAngleDegrees.get())));
  }

  public void poseCorrection(Pose2d initialPose) {
    fieldOffset = initialPose.minus(getRobotPose());
  }

  public Pose2d getFieldPose() {
    return getRobotPose().plus(fieldOffset);
  }

  public Pose2d getRobotPose() {
    return getQuestPose().transformBy(robotToQuest.inverse());
  }

  public Pose2d getQuestPose() {
    var eulerAngles = questEulerAngles.get();
    var rotation = Rotation2d.fromDegrees(-Math.IEEEremainder(eulerAngles[1], 360d));

    var questnavPosition = questPosition.get();
    var translation = new Translation2d(questnavPosition[2], -questnavPosition[0]);
    return new Pose2d(translation, rotation);
  }

  public void resetPose(Pose2d pose) {
    questResetPose.accept(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
  }

  private double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  private boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  private Quaternion getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
  }

  private double timestamp() {
    return questTimestamp.get();
  }

  // Clean up questnav subroutine messages after processing on the headset
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        this::updateRobotToQuest,
        questOffsetAngleDegrees,
        questOffsetXInches,
        questOffsetYInches);

    disconnectAlert.set(disconnectDebouncer.calculate(!connected()));
    batteryAlert.set(getBatteryPercent() < 25);

    Logger.recordOutput("Drive/QuestPose", getQuestPose());
    Logger.recordOutput("Drive/RobotPose", getRobotPose());
    Logger.recordOutput("Drive/OculusQuaternion", getQuaternion());
    Logger.recordOutput("Drive/FieldPose", getFieldPose());
  }
}
