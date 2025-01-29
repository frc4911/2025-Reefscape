// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.robot;

import com.ck4911.auto.AutoCommandHandler;
import com.ck4911.control.ControllerBinding;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javax.inject.Inject;

public class RobotContainer {

  private final AutoCommandHandler autoCommandHandler;
  private final ControllerBinding controllerBinding;

  @Inject
  public RobotContainer(
      AutoCommandHandler autoCommandHandler, ControllerBinding controllerBinding) {
    this.autoCommandHandler = autoCommandHandler;
    this.controllerBinding = controllerBinding;
  }

  /** Updates dashboard data. */
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void autonomousInit() {
    autoCommandHandler.startCurrentCommand();
  }

  public void teleopInit() {
    autoCommandHandler.stopCurrentCommand();
  }
}
