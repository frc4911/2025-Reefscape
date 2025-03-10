// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.quest;

import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.drive.Drive;
import com.ck4911.util.Alert;
import com.ck4911.util.Alert.AlertType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public final class QuestNav implements VirtualSubsystem {
  private final QuestIo io;
  private final QuestIoInputsAutoLogged inputs = new QuestIoInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Quest Disconnected!", AlertType.WARNING);
  private final Alert lowBatteryAlert =
      new Alert("Quest Battery is Low! (<25%)", AlertType.WARNING);

  private final QuestCalibration calibration;

  @Inject
  public QuestNav(QuestIo io, QuestCalibration calibration) {
    this.io = io;
    this.calibration = calibration;
    resetPose(new Pose2d());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("QuestNav", inputs);

    disconnectedAlert.set(!inputs.connected);
    lowBatteryAlert.set(inputs.batteryLevel < 25 && inputs.connected);

    // if (DriverStation.isEnabled() && Constants.currentMode == Constants.Mode.REAL) {
    //   BobotState.offerQuestMeasurement(new TimestampedPose(inputs.robotPose, inputs.timestamp));
    // }
  }

  public void resetPose(Pose2d pose) {
    io.resetPose(pose);
  }

  public Command calibrateCommand(Drive drive) {
    return calibration.determineOffsetToRobotCenter(
        drive, () -> inputs.robotPose, () -> inputs.questPose);
  }
}
