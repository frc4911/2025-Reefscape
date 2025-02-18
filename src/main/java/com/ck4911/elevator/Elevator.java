// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ck4911.characterization.Characterizable;
import com.ck4911.util.Alert;
import com.ck4911.util.LoggedTunableNumber;
import com.ck4911.util.LoggedTunableNumber.TunableNumbers;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public final class Elevator extends SubsystemBase implements Characterizable {
  private final ElevatorIo elevatorIo;
  private final ElevatorConstants constants;
  private final ElevatorIoInputsAutoLogged inputs = new ElevatorIoInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;
  private final LoggedTunableNumber p;
  private final LoggedTunableNumber i;
  private final LoggedTunableNumber d;
  private final LoggedTunableNumber s;
  private final LoggedTunableNumber g;
  private final LoggedTunableNumber v;
  private final LoggedTunableNumber a;
  private final LoggedTunableNumber velocity;
  private final LoggedTunableNumber acceleration;
  private final LoggedTunableNumber jerk;
  private final LoggedTunableNumber variance;
  private final LoggedTunableNumber debounceTime;
  private final LoggedTunableNumber homingTimeSeconds;
  private final LoggedTunableNumber homingVolts;
  private final LoggedTunableNumber homingVelocityThresh;
  private final Alert leaderDisonnected;
  private final Alert followerDisconnected;

  private double homedPositionRads = 0;

  @Inject
  public Elevator(
      ElevatorIo elevatorIo, ElevatorConstants constants, TunableNumbers tunableNumbers) {
    this.elevatorIo = elevatorIo;
    this.constants = constants;
    // TODO: adjust these values
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(0.1),
                Volts.of(0.1), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger clas
                (state) -> SignalLogger.writeString("Elevator_State", state.toString())),
            new SysIdRoutine.Mechanism(elevatorIo::runVolts, null, this));

    p = tunableNumbers.create("Elevator/p", constants.feedBackValues().p());
    i = tunableNumbers.create("Elevator/i", constants.feedBackValues().i());
    d = tunableNumbers.create("Elevator/d", constants.feedBackValues().d());
    s = tunableNumbers.create("Elevator/s", constants.feedForwardValues().s());
    g = tunableNumbers.create("Elevator/g", constants.feedForwardValues().g());
    v = tunableNumbers.create("Elevator/v", constants.feedForwardValues().v());
    a = tunableNumbers.create("Elevator/a", constants.feedForwardValues().a());
    velocity = tunableNumbers.create("Elevator/ProfileVelocity", constants.profileVelocity());
    acceleration =
        tunableNumbers.create("Elevator/ProfileAcceleration", constants.profileAcceleration());
    jerk = tunableNumbers.create("Elevator/ProfileJerk", constants.profileJerk());
    debounceTime = tunableNumbers.create("Arm/DebounceTime", constants.debounceTimeSeconds());
    variance = tunableNumbers.create("Arm/Variance", constants.variance());
    homingVolts = tunableNumbers.create("Elevator/HomingVolts", constants.homingVolts());
    homingVelocityThresh =
        tunableNumbers.create("Elevator/HomingVelocityThresh", constants.homingVelocityThresh());
    homingTimeSeconds =
        tunableNumbers.create("Elevator/HomingTimeSecs", constants.homingTimeSeconds());

    elevatorIo.setPid(p.get(), i.get(), d.get());
    elevatorIo.setFeedForward(s.get(), g.get(), v.get(), a.get());
    elevatorIo.setProfile(
        RotationsPerSecond.of(velocity.get()),
        RotationsPerSecondPerSecond.of(acceleration.get()),
        RotationsPerSecondPerSecond.per(Second).of(jerk.get()));

    leaderDisonnected = new Alert("Elevator lead motor disconnected!", Alert.AlertType.WARNING);
    followerDisconnected =
        new Alert("Elevator follow motor disconnected!", Alert.AlertType.WARNING);
  }

  @Override
  public void periodic() {
    elevatorIo.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    leaderDisonnected.set(!inputs.leaderConnected);
    followerDisconnected.set(!inputs.followerConnected);

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> elevatorIo.setPid(p.get(), i.get(), d.get()), p, i, d);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> elevatorIo.setFeedForward(s.get(), g.get(), v.get(), a.get()),
        s,
        g,
        v,
        a);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            elevatorIo.setProfile(
                RotationsPerSecond.of(velocity.get()),
                RotationsPerSecondPerSecond.of(acceleration.get()),
                RotationsPerSecondPerSecond.per(Second).of(jerk.get())),
        velocity,
        acceleration,
        jerk);
    checkLimits();
  }

  @Override
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  public void setAngle(Angle angle) {
    // TODO: Use Alerts and also cap angles beyond limits
    // if (angle.compareTo(Radians.of(constants.maxPositionRads())) > 0) {
    //   System.out.println("ERROR: angle above max");
    //   return;
    // }
    // if (angle.compareTo(Radians.of(constants.minPositionRads())) < 0) {
    //   System.out.println("ERROR: angle below min");
    //   return;
    // }
    // TODO: calculate feed forward
    elevatorIo.runPosition(angle, Amps.of(0));
  }

  public Angle getAngle() {
    return Radians.of(inputs.positionRads);
  }

  public Distance getPosition() {
    Distance position =
        Meters.of(constants.sprocketRadius() * (inputs.positionRads - homedPositionRads));
    Logger.recordOutput("Elevator/MeasuredHeightMeters", position.baseUnitMagnitude());
    return position;
  }

  private Command goTo(Angle angle) {
    Debouncer debouncer = new Debouncer(debounceTime.get());
    return Commands.run(() -> setAngle(angle), this)
        .until(() -> debouncer.calculate(getAngle().isNear(angle, variance.get())));
  }

  public Command stow() {
    return goTo(Radians.of(constants.stowPositionRadians()));
  }

  public Command prepareCollect() {
    return goTo(Radians.of(constants.prepareCollectPositionRadians()));
  }

  public Command collect() {
    return goTo(Radians.of(constants.collectPositionRadians()));
  }

  public Command trough() {
    return goTo(Radians.of(constants.troughPositionRadians()));
  }

  public Command levelTwo() {
    return goTo(Radians.of(constants.levelTwoPositionRadians()));
  }

  public Command levelThree() {
    return goTo(Radians.of(constants.levelThreePositionRadians()));
  }

  public Command levelFour() {
    return goTo(Radians.of(constants.levelFourPositionRadians()));
  }

  // A command to gently move the elevator to the home position and mark it.
  public Command home() {
    return new Command() {
      Debouncer homingDebouncer;

      @Override
      public void initialize() {
        homingDebouncer = new Debouncer(homingTimeSeconds.get());
      }

      @Override
      public void execute() {
        elevatorIo.runVolts(Volts.of(homingVolts.get()));
      }

      @Override
      public boolean isFinished() {
        return homingDebouncer.calculate(
            Math.abs(inputs.velocityRadPerSec) <= homingVelocityThresh.get());
      }

      @Override
      public void end(boolean interrupted) {
        if (!interrupted) {
          homedPositionRads = inputs.positionRads;
        }
      }
    };
  }

  private void checkLimits() {
    // TODO: check position limits (upper and lower)
  }
}
