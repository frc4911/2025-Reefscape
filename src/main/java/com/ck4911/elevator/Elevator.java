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
import com.ck4911.field.ReefLevel;
import com.ck4911.util.Alert;
import com.ck4911.util.LoggedTunableNumber;
import com.ck4911.util.LoggedTunableNumber.TunableNumbers;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import javax.inject.Inject;
import javax.inject.Named;
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
  private final MechanismLigament2d elevatorMechanism;

  private double homedPositionRads = 0;

  @Inject
  public Elevator(
      ElevatorIo elevatorIo,
      ElevatorConstants constants,
      @Named("Elevator") MechanismLigament2d elevatorMechanism,
      TunableNumbers tunableNumbers) {
    this.elevatorIo = elevatorIo;
    this.constants = constants;
    this.elevatorMechanism = elevatorMechanism;
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
    debounceTime = tunableNumbers.create("Elevator/DebounceTime", constants.debounceTimeSeconds());
    variance = tunableNumbers.create("Elevator/Variance", constants.variance());
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
    leaderDisonnected = new Alert("Elevator lead motor disconnected!", Alert.AlertType.WARNING);
    followerDisconnected =
        new Alert("Elevator follow motor disconnected!", Alert.AlertType.WARNING);

    elevatorIo.setPid(p.get(), i.get(), d.get());
    elevatorIo.setFeedForward(s.get(), g.get(), v.get(), a.get());
    elevatorIo.setProfile(
        RotationsPerSecond.of(velocity.get()),
        RotationsPerSecondPerSecond.of(acceleration.get()),
        RotationsPerSecondPerSecond.per(Second).of(jerk.get()));
  }

  @Override
  public void periodic() {
    elevatorIo.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Distance position =
        Meters.of(constants.sprocketRadius() * (inputs.positionRads - homedPositionRads));

    elevatorMechanism.setLength(.2 + position.baseUnitMagnitude());

    Logger.recordOutput("Elevator/Home", homedPositionRads);
    Logger.recordOutput("Elevator/CurrentPosition", getCurrentAngle().baseUnitMagnitude());
    Logger.recordOutput("Elevator/MeasuredHeightMeters", position.baseUnitMagnitude());

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

  public Angle getCurrentAngle() {
    return Radians.of(inputs.positionRads - homedPositionRads);
  }

  private Angle getAngleForReefLevel(ReefLevel reefLevel) {
    LoggedTunableNumber number =
        switch (reefLevel) {
          case LEVEL_1 -> troughPositionRadians;
          case LEVEL_2 -> levelTwoPositionRadians;
          case LEVEL_3 -> levelThreePositionRadians;
          case LEVEL_4 -> levelFourPositionRadians;
        };
    return Radians.of(number.get());
  }

  private Command goTo(LoggedTunableNumber number) {
    return Commands.run(() -> setAngle(Radians.of(number.get())), this);
  }

  private Command goTo(LoggedTunableNumber number, boolean endWhenFinished) {
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

  private Command waitFor(Angle angle) {
    Debouncer debouncer = new Debouncer(debounceTime.get());
    double error = variance.get();
    return Commands.waitUntil(
            () -> {
              boolean done =
                      debouncer.calculate(
                              getCurrentAngle().div(angle).g .baseUnitMagnitude() > angle.baseUnitMagnitude());
              Logger.recordOutput("Elevator/waitForAngleGet", getCurrentAngle().baseUnitMagnitude());
              return done;
            });
  }

  private Command waitUntilAbove(Angle angle) {
    Debouncer debouncer = new Debouncer(debounceTime.get());
    Logger.recordOutput("Elevator/waitForAngleAngle", angle.baseUnitMagnitude());
    return Commands.waitUntil(
        () -> {
          boolean done =
              debouncer.calculate(
                  getCurrentAngle().baseUnitMagnitude() > angle.baseUnitMagnitude());
          Logger.recordOutput("Elevator/waitForAngleGet", getCurrentAngle().baseUnitMagnitude());
          return done;
        });
  }

  private Command waitUntilBelow(Angle angle) {
    Debouncer debouncer = new Debouncer(debounceTime.get());
    Logger.recordOutput("Elevator/waitForAngleAngle", angle.baseUnitMagnitude());
    return Commands.waitUntil(
        () -> {
          boolean done =
              debouncer.calculate(
                  getCurrentAngle().baseUnitMagnitude() < angle.baseUnitMagnitude());
          Logger.recordOutput("ArElevatorm/waitForAngleGet", getCurrentAngle().baseUnitMagnitude());
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

  public Command reefLevel(ReefLevel reefLevel) {
    return Commands.run(() -> setAngle(getAngleForReefLevel(reefLevel)), this);
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

  public Command homeWithCoral() {
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
          homedPositionRads = inputs.positionRads - 7.05;
        }
      }
    };
  }

  public Command waitUntilPrepareCollect() {
    // TODO: monitor height and wait for actual position
    return Commands.waitSeconds(0.5);
  }
}
