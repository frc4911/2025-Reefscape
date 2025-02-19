// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.control;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ck4911.arm.Arm;
import com.ck4911.characterization.Characterization;
import com.ck4911.commands.CyberCommands;
import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.control.Controller.Role;
import com.ck4911.drive.Drive;
import com.ck4911.drive.TunerConstants;
import com.ck4911.elevator.Elevator;
import com.ck4911.util.Alert;
import com.ck4911.util.Alert.AlertType;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final Arm arm;
  private final Elevator elevator;
  private final Characterization characterization;
  private final CyberCommands cyberCommands;

  // kSpeedAt12Volts desired top speed
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // 3/4 of a rotation per second max angular velocity
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric driveRequest =
      // Add a 10% deadband
      // Use open-loop control for drive motors
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  @Inject
  public ControllerBinding(
      Drive drive,
      Arm arm,
      Elevator elevator,
      @Controller(Role.DRIVER) CyberKnightsController driver,
      @Controller(Role.OPERATOR) CyberKnightsController operator,
      Characterization characterization,
      CyberCommands cyberCommands) {
    this.drive = drive;
    this.arm = arm;
    this.elevator = elevator;
    this.driver = driver;
    this.operator = operator;
    this.characterization = characterization;
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
            () ->
                // Drive forward with negative Y (forward)
                // Drive left with negative X (left)
                // Drive counterclockwise with negative X (left)
                driveRequest
                    .withVelocityX(-driver.getLeftY())
                    .withVelocityY(-driver.getLeftX())
                    .withRotationalRate(-driver.getRightX())));

    // driver.a().onTrue(characterization.fullDriveCharacterization(driver.x()));
    // driver.y().onTrue(characterization.fullArmCharaterization(driver.x()));
    // driver.leftBumper().onTrue(null);
    // driver.b().onTrue(Commands.runOnce(() -> arm.setAngle(Degrees.of(0)), arm));
    // driver.a().onTrue(cyberCommands.levelTwo()); // actually arm L2/l3
    // driver.b().onTrue(cyberCommands.trough()); // actually arm trough
    // driver.y().onTrue(cyberCommands.levelFour()); // actually elevator trough
    // driver.x().onTrue(cyberCommands.levelThree()); // actually elevator l3
    driver.x().onTrue(elevator.home());
  }

  public void setDriverRumble(boolean enabled) {
    driver.getHID().setRumble(RumbleType.kBothRumble, enabled ? 1 : 0);
  }

  public void setOperatorRumble(boolean enabled) {
    operator.getHID().setRumble(RumbleType.kBothRumble, enabled ? 1 : 0);
  }

  private Command level1Score() {
    /* Elevator to trough position
    Arm to trough Position */
    return Commands.none();
  }

  private Command level2Prep() {
    /* Elevator to L2 position
    Arm to L2&3 Position */
    return Commands.none();
  }

  private Command level2Score() {
    // Arm stow
    return Commands.none();
  }

  private Command level3Prep() {
    /* Elevator to L3 position
    Arm to L2&3 Position */
    return Commands.none();
  }

  private Command level3Score() {
    // Arm stow
    return Commands.none();
  }

  private Command level4Prep() {
    /* Elevator to L4 position
    Arm to L4 Position */
    return Commands.none();
  }

  private Command level4Score() {
    // Arm stow
    return Commands.none();
  }

  private Command collect() {
    // wait until coral detected
    return Commands.none();
  }

  private Command climbShallow() {
    // elevator down
    return Commands.none();
  }

  private Command climbDeep() {
    /* climb mechanism on
    if pigeon ever reads pitch or roll >= |(tbd) deg|:
    climb mechanism down/set robot on floor */
    return Commands.none();
  }
}
