// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.robot;

import com.ck4911.auto.AutoCommandHandler;
import com.ck4911.commands.CyberCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import javax.inject.Inject;

public class RobotContainer {

  private final AutoCommandHandler autoCommandHandler;
  private final CyberCommands cyberCommands;
  private final CommandScheduler commandScheduler;

  @Inject
  public RobotContainer(
      AutoCommandHandler autoCommandHandler,
      CyberCommands cyberCommands,
      CommandScheduler commandScheduler) {
    this.autoCommandHandler = autoCommandHandler;
    this.cyberCommands = cyberCommands;
    this.commandScheduler = commandScheduler;
  }

  /** Updates dashboard data. */
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void autonomousInit() {
    System.out.println("AutoOn");
    autoCommandHandler.startCurrentCommand();
    commandScheduler.schedule(cyberCommands.home());
  }

  public void teleopInit() {
    System.out.println("Teleop");
    autoCommandHandler.stopCurrentCommand();
    commandScheduler.schedule(cyberCommands.home());
  }
}
