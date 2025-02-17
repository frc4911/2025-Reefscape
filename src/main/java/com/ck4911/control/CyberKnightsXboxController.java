// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

final class CyberKnightsXboxController implements CyberKnightsController {
  private final CommandXboxController controller;

  CyberKnightsXboxController(int port) {
    controller = new CommandXboxController(port);
  }

  @Override
  public GenericHID getHID() {
    return controller.getHID();
  }

  @Override
  public Trigger leftBumper() {
    return controller.leftBumper();
  }

  @Override
  public Trigger rightBumper() {
    return controller.rightBumper();
  }

  @Override
  public Trigger leftStick() {
    return controller.leftStick();
  }

  @Override
  public Trigger rightStick() {
    return controller.rightStick();
  }

  @Override
  public Trigger leftTrigger() {
    return controller.leftTrigger();
  }

  @Override
  public Trigger rightTrigger() {
    return controller.rightTrigger();
  }

  @Override
  public double getLeftX() {
    return controller.getLeftX();
  }

  @Override
  public double getRightX() {
    return controller.getRightX();
  }

  @Override
  public double getLeftY() {
    return controller.getLeftY();
  }

  @Override
  public double getRightY() {
    return controller.getRightY();
  }

  @Override
  public Trigger a() {
    return controller.a();
  }

  @Override
  public Trigger b() {
    return controller.b();
  }

  @Override
  public Trigger x() {
    return controller.x();
  }

  @Override
  public Trigger y() {
    return controller.y();
  }

  @Override
  public Trigger povUp() {
    return controller.povUp();
  }

  @Override
  public Trigger povRight() {
    return controller.povRight();
  }

  @Override
  public Trigger povDown() {
    return controller.povDown();
  }

  @Override
  public Trigger povLeft() {
    return controller.povLeft();
  }
}
