// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.commands;

import com.ck4911.auto.AutoAlign;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@Singleton
public final class DashboardCommands implements VirtualSubsystem {
  private final CyberCommands cyberCommands;
  private final AutoAlign.AutoAlignFactory factory;

  private AutoAlign.ReefPosition currentReefPosition = AutoAlign.ReefPosition.A;
  private ReefLevel currentReefLevel = ReefLevel.LEVEL_4;

  public enum ReefLevel {
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    LEVEL_4
  }

  @Inject
  DashboardCommands(CyberCommands cyberCommands, AutoAlign.AutoAlignFactory factory) {
    this.cyberCommands = cyberCommands;
    this.factory = factory;
  }

  public void addAllReefPositions() {
    SmartDashboard.putData("BRANCH A", setCurrentBranchCommand(AutoAlign.ReefPosition.A));
    SmartDashboard.putData("BRANCH B", setCurrentBranchCommand(AutoAlign.ReefPosition.B));
    SmartDashboard.putData("BRANCH C", setCurrentBranchCommand(AutoAlign.ReefPosition.C));
    SmartDashboard.putData("BRANCH D", setCurrentBranchCommand(AutoAlign.ReefPosition.D));
    SmartDashboard.putData("BRANCH E", setCurrentBranchCommand(AutoAlign.ReefPosition.E));
    SmartDashboard.putData("BRANCH F", setCurrentBranchCommand(AutoAlign.ReefPosition.F));
    SmartDashboard.putData("BRANCH G", setCurrentBranchCommand(AutoAlign.ReefPosition.G));
    SmartDashboard.putData("BRANCH H", setCurrentBranchCommand(AutoAlign.ReefPosition.H));
    SmartDashboard.putData("BRANCH I", setCurrentBranchCommand(AutoAlign.ReefPosition.I));
    SmartDashboard.putData("BRANCH J", setCurrentBranchCommand(AutoAlign.ReefPosition.J));
    SmartDashboard.putData("BRANCH K", setCurrentBranchCommand(AutoAlign.ReefPosition.K));
    SmartDashboard.putData("BRANCH L", setCurrentBranchCommand(AutoAlign.ReefPosition.L));
  }

  public void addAllReefLevels() {
    LoggedDashboardChooser<ReefLevel> levelChooser = new LoggedDashboardChooser<>("Reef Level");
    levelChooser.addDefaultOption("LEVEL 4", ReefLevel.LEVEL_4);
    levelChooser.addDefaultOption("LEVEL 3", ReefLevel.LEVEL_3);
    levelChooser.addDefaultOption("LEVEL 2", ReefLevel.LEVEL_2);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Drive/CurrentReefPosition", currentReefPosition.name());
    Logger.recordOutput("Drive/CurrentReefLevel", currentReefPosition.name());
  }

  public Command goToCurrentReefPosition() {
    return factory.alignWith(currentReefPosition);
  }

  public Command goToCurrentReefLevel() {
    return switch (currentReefLevel) {
      case LEVEL_2 -> cyberCommands.levelTwo();
      case LEVEL_3 -> cyberCommands.levelThree();
      case LEVEL_4 -> cyberCommands.levelFour();
      default -> Commands.none();
    };
  }

  private Command setCurrentBranchCommand(AutoAlign.ReefPosition reefPosition) {
    return Commands.runOnce(() -> currentReefPosition = reefPosition);
  }

  private Command setCurrentLevelCommand(ReefLevel reefLevel) {
    return Commands.runOnce(() -> currentReefLevel = reefLevel);
  }
}
