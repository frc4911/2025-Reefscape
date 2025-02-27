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
  private final LoggedTunableNumber corralHeight;
  private final LoggedTunableNumber a;
  private final LoggedTunableNumber velocity;
  private final LoggedTunableNumber acceleration;
  private final LoggedTunableNumber jerk;
  private final LoggedTunableNumber variance;
  private final LoggedTunableNumber debounceTime;
  private final LoggedTunableNumber homingTimeSeconds;
  private final LoggedTunableNumber homingVolts;
  private final LoggedTunableNumber homingVelocityThresh;
  private final LoggedTunableNumber stowPositionRadians;
  private final LoggedTunableNumber prepareCollectPositionRadians;
  private final LoggedTunableNumber collectPositionRadians;
  private final LoggedTunableNumber troughPositionRadians;
  private final LoggedTunableNumber levelTwoPositionRadians;
  private final LoggedTunableNumber levelThreePositionRadians;
  private final LoggedTunableNumber levelFourPositionRadians;
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
    corralHeight = tunableNumbers.create("Elevator/CorralHeight", constants.corralHeight());
    homingTimeSeconds =
        tunableNumbers.create("Elevator/HomingTimeSecs", constants.homingTimeSeconds());
    stowPositionRadians =
        tunableNumbers.create("Elevator/StowPosition", constants.stowPositionRadians());
    prepareCollectPositionRadians =
        tunableNumbers.create(
            "Elevator/PrepareCollectPosition", constants.prepareCollectPositionRadians());
    collectPositionRadians =
        tunableNumbers.create("Elevator/CollectPosition", constants.collectPositionRadians());
    troughPositionRadians =
        tunableNumbers.create("Elevator/TroughPosition", constants.troughPositionRadians());
    levelTwoPositionRadians =
        tunableNumbers.create("Elevator/LevelTwoPosition", constants.levelTwoPositionRadians());
    levelThreePositionRadians =
        tunableNumbers.create("Elevator/LevelThreePosition", constants.levelThreePositionRadians());
    levelFourPositionRadians =
        tunableNumbers.create("Elevator/LevelFourPosition", constants.levelFourPositionRadians());

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
    Logger.recordOutput("Elevator/Home", homedPositionRads);
    Logger.recordOutput("Elevator/CurrentPosition", getAngle().baseUnitMagnitude());

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
    elevatorIo.runPosition(angle.plus(Radians.of(homedPositionRads)), Amps.of(0));
    Logger.recordOutput("Elevator/SetAngle", angle.plus(Radians.of(homedPositionRads)));
  }

  public Angle getAngle() {
    return Radians.of(inputs.positionRads - homedPositionRads);
  }

  public Distance getPosition() {
    Distance position =
        Meters.of(constants.sprocketRadius() * (inputs.positionRads - homedPositionRads));
    Logger.recordOutput("Elevator/MeasuredHeightMeters", position.baseUnitMagnitude());
    return position;
  }

  private Command goTo(LoggedTunableNumber number) {
    return Commands.run(() -> setAngle(Radians.of(number.get())), this);
  }

  public Command waitForPrepareCollectPosition() {
    return waitUntilAbove(Radians.of(constants.prepareCollectPositionRadians() * 0.9));
  }

  public Command waitForCorralClearance() {
    // TODO: use a different constant for corral clearance
    return waitUntilAbove(Radians.of(constants.prepareCollectPositionRadians() * 0.9));
  }

  public Command waitForCollect() {
    // TODO: use the sensor instead/as well
    return waitUntilBelow(Radians.of(constants.collectPositionRadians()));
  }

  private Command waitUntilAbove(Angle angle) {
    Debouncer debouncer = new Debouncer(debounceTime.get());
    Logger.recordOutput("Elevator/waitForAngleAngle", angle.baseUnitMagnitude());
    return Commands.waitUntil(
        () -> {
          boolean done =
              debouncer.calculate(getAngle().baseUnitMagnitude() > angle.baseUnitMagnitude());
          Logger.recordOutput("Elevator/waitForAngleGet", getAngle().baseUnitMagnitude());
          return done;
        });
  }

  private Command waitUntilBelow(Angle angle) {
    Debouncer debouncer = new Debouncer(debounceTime.get());
    Logger.recordOutput("Elevator/waitForAngleAngle", angle.baseUnitMagnitude());
    return Commands.waitUntil(
        () -> {
          boolean done =
              debouncer.calculate(getAngle().baseUnitMagnitude() < angle.baseUnitMagnitude());
          Logger.recordOutput("ArElevatorm/waitForAngleGet", getAngle().baseUnitMagnitude());
          return done;
        });
  }

  public Command stow() {
    return goTo(stowPositionRadians);
  }

  public Command prepareCollect() {
    return goTo(prepareCollectPositionRadians);
  }

  public Command collect() {
    return goTo(collectPositionRadians);
  }

  public Command passCorral() {
    return goTo(corralHeight);
  }

  public Command trough() {
    return goTo(troughPositionRadians);
  }

  public Command levelTwo() {
    return goTo(levelTwoPositionRadians);
  }

  public Command levelThree() {
    return goTo(levelThreePositionRadians);
  }

  public Command levelFour() {
    return goTo(levelFourPositionRadians);
  }

  // A command to gently move the elevator to the home position and mark it.
  public Command home() {
    return new Command() {
      private Debouncer homingDebouncer;
      private boolean homed;

      @Override
      public void initialize() {
        homingDebouncer = new Debouncer(homingTimeSeconds.get());
        homed = false;
      }

      @Override
      public void execute() {
        elevatorIo.runVolts(Volts.of(homingVolts.get()));
        homed =
            homingDebouncer.calculate(
                Math.abs(inputs.velocityRadPerSec) <= homingVelocityThresh.get());
      }

      @Override
      public boolean isFinished() {
        return homed;
      }

      @Override
      public void end(boolean interrupted) {
        if (homed && !interrupted) {
          homedPositionRads = inputs.positionRads;
        }
      }
    };
  }

  private void checkLimits() {
    // TODO: check position limits (upper and lower)
  }

  public Command waitUntilAboveCorral() {
    // TODO: monitor height and wait for actual position
    return Commands.waitSeconds(0.5);
  }
}
