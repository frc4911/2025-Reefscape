// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.robot;

import com.ck4911.arm.ArmModule;
import com.ck4911.auto.AutoModule;
import com.ck4911.commands.CommandsModule;
import com.ck4911.control.ControlModule;
import com.ck4911.drive.DriveModule;
import com.ck4911.elevator.ElevatorModule;
import com.ck4911.leds.LedModule;
import com.ck4911.vision.VisionModule;
import dagger.Component;
import javax.inject.Singleton;

@Component(
    modules = {
      ArmModule.class,
      AutoModule.class,
      ControlModule.class,
      CommandsModule.class,
      DriveModule.class,
      ElevatorModule.class,
      LedModule.class,
      RobotModule.class,
      VisionModule.class,
    })
@Singleton
public interface RobotComponent {
  public CyberKnightsRobot robot();
}
