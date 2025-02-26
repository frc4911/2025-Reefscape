// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.control;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ck4911.commands.CyberCommands;
import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.control.Controller.Role;
import com.ck4911.drive.Drive;
import com.ck4911.drive.TunerConstants;
import com.ck4911.util.Alert;
import com.ck4911.util.Alert.AlertType;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class ControllerBinding implements VirtualSubsystem {
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);

  private final CyberKnightsController driver;
  private final CyberKnightsController operator;
  private final Drive drive;
  private final CyberCommands cyberCommands;

  // kSpeedAt12Volts desired top speed
  private LinearVelocity maxSpeed = TunerConstants.kSpeedAt12Volts;
  // 3/4 of a rotation per second max angular velocity
  private AngularVelocity maxAngularSpeed = RotationsPerSecond.of(0.75);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric driveRequest =
      // Add a 10% deadband
      // Use open-loop control for drive motors
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed.times(0.1))
          .withRotationalDeadband(maxAngularSpeed.times(0.1))
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  @Inject
  public ControllerBinding(
      Drive drive,
      @Controller(Role.DRIVER) CyberKnightsController driver,
      @Controller(Role.OPERATOR) CyberKnightsController operator,
      CyberCommands cyberCommands) {
    this.drive = drive;
    this.driver = driver;
    this.operator = operator;
    this.cyberCommands = cyberCommands;

    setupControls();
  }

  @Override
  public void periodic() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    // operatorDisconnected.set(
    //     !DriverStation.isJoystickConnected(operator.getHID().getPort())
    //         || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));
  }

  private void setupControls() {
    drive.setDefaultCommand(
        drive.applyRequest(
            () -> {
              // TODO: swap these after pigeon fix
              double x = driver.getLeftX();
              double y = -driver.getLeftX();
              double theta = -driver.getRightX();
              if (driver.leftTrigger().getAsBoolean()) {
                x = x * .1;
                y = y * .1;
                theta = theta * .1;
              }
              return driveRequest
                  .withVelocityX(maxSpeed.times(x))
                  .withVelocityY(maxSpeed.times(y))
                  .withRotationalRate(maxAngularSpeed.times(theta));
            }));

    // driver.a().onTrue(characterization.fullDriveCharacterization(driver.x()));
    // driver.y().onTrue(characterization.fullArmCharaterization(driver.x()));
    // driver.leftBumper().onTrue(null);
    // driver.b().onTrue(Commands.runOnce(() -> arm.setAngle(Degrees.of(0)), arm));
    // driver.a().onTrue(cyberCommands.levelTwo()); // actually arm L2/l3
    // driver.b().onTrue(cyberCommands.trough()); // actually arm trough
    // driver.y().onTrue(cyberCommands.levelFour()); // actually elevator trough
    // driver.x().onTrue(cyberCommands.levelThree()); // actually elevator l3
    operator.leftBumper().onTrue(cyberCommands.home());
    operator.povUp().onTrue(cyberCommands.prepareForCollect());
    operator.povDown().onTrue(cyberCommands.collect());
    operator.povLeft().onTrue(cyberCommands.stow());
    operator.rightTrigger().onTrue(cyberCommands.score());
    operator.b().onTrue(cyberCommands.levelThree());

    operator.x().onTrue(cyberCommands.levelTwo());
    operator.a().onTrue(cyberCommands.trough());
    operator.y().onTrue(cyberCommands.levelFour());

    driver.leftTrigger().onTrue(cyberCommands.collect());
  }

  public void setDriverRumble(boolean enabled) {
    driver.getHID().setRumble(RumbleType.kBothRumble, enabled ? 1 : 0);
  }

  public void setOperatorRumble(boolean enabled) {
    operator.getHID().setRumble(RumbleType.kBothRumble, enabled ? 1 : 0);
  }
}
