// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.control;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ck4911.commands.CyberCommands;
import com.ck4911.commands.DashboardCommands;
import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.control.Controller.Role;
import com.ck4911.drive.Drive;
import com.ck4911.drive.TunerConstants;
import com.ck4911.field.ReefLevel;
import com.ck4911.util.Alert;
import com.ck4911.util.Alert.AlertType;
import com.ck4911.util.LoggedTunableNumber;
import com.ck4911.util.LoggedTunableNumber.TunableNumbers;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class ControllerBinding implements VirtualSubsystem {
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);

  private final LoggedTunableNumber deadband;
  private final LoggedTunableNumber sniperScale;
  private final CyberKnightsController driver;
  private final CyberKnightsController operator;
  private final Drive drive;
  private final CyberCommands cyberCommands;
  private final DashboardCommands dashboardCommands;

  // kSpeedAt12Volts desired top speed
  private final LinearVelocity maxSpeed = TunerConstants.kSpeedAt12Volts;
  // 3/4 of a rotation per second max angular velocity
  private final AngularVelocity maxAngularSpeed = RotationsPerSecond.of(0.75);

  // TODO: experiment with DriveRequestType.Velocity
  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed.times(0.1))
          .withRotationalDeadband(maxAngularSpeed.times(0.1))
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // TODO: experiment with this
  // .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  @Inject
  public ControllerBinding(
      ControlConstants constants,
      Drive drive,
      @Controller(Role.DRIVER) CyberKnightsController driver,
      @Controller(Role.OPERATOR) CyberKnightsController operator,
      CyberCommands cyberCommands,
      DashboardCommands dashboardCommands,
      TunableNumbers tunableNumbers) {
    this.drive = drive;
    this.driver = driver;
    this.operator = operator;
    this.cyberCommands = cyberCommands;
    this.dashboardCommands = dashboardCommands;
    deadband = tunableNumbers.create("Controller/deadband", constants.deadband());
    sniperScale = tunableNumbers.create("Controller/sniperScale", constants.sniperScale());
    updateDeadband(deadband.get());
    setupControls();
    dashboardCommands.addAllReefLevels();
    dashboardCommands.addAllReefPositions();
  }

  @Override
  public void periodic() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operator.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));

    LoggedTunableNumber.ifChanged(hashCode(), () -> updateDeadband(deadband.get()), deadband);
  }

  private void updateDeadband(double deadband) {
    driveRequest
        .withDeadband(maxSpeed.times(deadband))
        .withRotationalDeadband(maxAngularSpeed.times(deadband));
  }

  private void setupControls() {
    drive.setDefaultCommand(
        drive.applyRequest(
            () -> {
              double x = -driver.getLeftY();
              double y = -driver.getLeftX();
              double theta = -driver.getRightX();
              if (driver.leftTrigger().getAsBoolean()) {
                x = x * sniperScale.get();
                y = y * sniperScale.get();
                theta = theta * sniperScale.get();
              }
              return driveRequest
                  .withVelocityX(maxSpeed.times(x))
                  .withVelocityY(maxSpeed.times(y))
                  .withRotationalRate(maxAngularSpeed.times(theta));
            }));

    operator.leftBumper().debounce(1.0).onTrue(cyberCommands.home());
    operator.rightBumper().debounce(1.0).onTrue(cyberCommands.homeWithCoral());

    operator.povUp().onTrue(cyberCommands.prepareForCollect());
    operator.povDown().onTrue(cyberCommands.collect());
    operator.povLeft().onTrue(cyberCommands.stow());
    operator.rightTrigger().onTrue(cyberCommands.score());

    //    operator.a().onTrue(cyberCommands.reefLevel(ReefLevel.LEVEL_1));
    operator.b().onTrue(cyberCommands.reefLevel(ReefLevel.LEVEL_3));
    operator.x().onTrue(cyberCommands.reefLevel(ReefLevel.LEVEL_2));
    operator.y().onTrue(cyberCommands.reefLevel(ReefLevel.LEVEL_4));

    driver.leftBumper().onTrue(cyberCommands.prepareForCollect());
    driver.x().whileTrue(drive.applyRequest(() -> brake));
    // This is a "long press"; it will only zero if the button is held down for a few seconds
    driver
        .y()
        .debounce(1.0)
        .onTrue(
            cyberCommands
                .resetForward(Degrees.of(0))
                .alongWith(
                    Commands.runOnce(() -> setDriverRumble(true))
                        .withTimeout(1.5)
                        .andThen(() -> setDriverRumble(false))));

    driver.rightBumper().onTrue(dashboardCommands.goToCurrentReefLevel());
    // TODO: test this out before enabling it
    //    driver.rightTrigger().onTrue(dashboardCommands.goToCurrentReefPosition());
  }

  public void setDriverRumble(boolean enabled) {
    driver.getHID().setRumble(RumbleType.kBothRumble, enabled ? 1 : 0);
  }

  public void setOperatorRumble(boolean enabled) {
    operator.getHID().setRumble(RumbleType.kBothRumble, enabled ? 1 : 0);
  }
}
