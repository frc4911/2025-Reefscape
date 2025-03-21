// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public final class Arm extends SubsystemBase implements Characterizable {

  private final ArmIo armIo;
  private final ArmIoInputsAutoLogged inputs = new ArmIoInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;
  private final LoggedTunableNumber p;
  private final LoggedTunableNumber i;
  private final LoggedTunableNumber d;
  private final LoggedTunableNumber s;
  private final LoggedTunableNumber g;
  private final LoggedTunableNumber v;
  private final LoggedTunableNumber a;
  private final LoggedTunableNumber troughPositionDegrees;
  private final LoggedTunableNumber levelTwoAndThreePositionDegrees;
  private final LoggedTunableNumber levelFourPositionDegrees;
  private final LoggedTunableNumber velocity;
  private final LoggedTunableNumber acceleration;
  private final LoggedTunableNumber jerk;
  private final LoggedTunableNumber debounceTime;
  private final LoggedTunableNumber coralDetectionDistance;
  private final LoggedTunableNumber coralScoreDistance;
  private final Alert motorDisconnected;
  private final Alert encoderDisconnected;
  private final ArmConstants constants;

  @Inject
  public Arm(ArmConstants constants, ArmIo armIo, TunableNumbers tunableNumbers) {
    super();
    this.armIo = armIo;
    this.constants = constants;

    // TODO: adjust these values
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(0.1),
                Volts.of(0.1), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("Arm_State", state.toString())),
            new SysIdRoutine.Mechanism(armIo::runVolts, null, this));
    p = tunableNumbers.create("Arm/p", constants.feedBackValues().p());
    i = tunableNumbers.create("Arm/i", constants.feedBackValues().i());
    d = tunableNumbers.create("Arm/d", constants.feedBackValues().d());
    s = tunableNumbers.create("Arm/s", constants.feedForwardValues().s());
    g = tunableNumbers.create("Arm/g", constants.feedForwardValues().g());
    v = tunableNumbers.create("Arm/v", constants.feedForwardValues().v());
    a = tunableNumbers.create("Arm/a", constants.feedForwardValues().a());
    troughPositionDegrees =
        tunableNumbers.create("Arm/TroughPosition", constants.troughPositionDegrees());
    levelTwoAndThreePositionDegrees =
        tunableNumbers.create(
            "Arm/LevelTwoThreePosition", constants.levelTwoAndThreePositionDegrees());
    levelFourPositionDegrees =
        tunableNumbers.create("Arm/LevelFourPosition", constants.levelFourPositionDegrees());
    velocity = tunableNumbers.create("Arm/ProfileVelocity", constants.profileVelocity());
    acceleration =
        tunableNumbers.create("Arm/ProfileAcceleration", constants.profileAcceleration());
    jerk = tunableNumbers.create("Arm/ProfileJerk", constants.profileJerk());
    debounceTime = tunableNumbers.create("Arm/DebounceTime", constants.debounceTimeSeconds());
    coralDetectionDistance =
        tunableNumbers.create("Arm/CoralDetectMm", constants.coralDetectionDistanceMillimeters());
    coralScoreDistance =
        tunableNumbers.create("Arm/CoralScoreMm", constants.coralScoreDistanceMillimeters());
    armIo.setPid(p.get(), i.get(), d.get());
    armIo.setFeedForward(s.get(), g.get(), v.get(), a.get());
    armIo.setProfile(
        RotationsPerSecond.of(velocity.get()),
        RotationsPerSecondPerSecond.of(acceleration.get()),
        RotationsPerSecondPerSecond.per(Second).of(jerk.get()));

    motorDisconnected = new Alert("Arm motor disconnected!", Alert.AlertType.WARNING);
    encoderDisconnected = new Alert("Arm absolute encoder disconnected!", Alert.AlertType.WARNING);
  }

  @Override
  public void periodic() {
    armIo.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    motorDisconnected.set(!inputs.motorConnected);
    encoderDisconnected.set(!inputs.absoluteEncoderConnected);

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> armIo.setPid(p.get(), i.get(), d.get()), p, i, d);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> armIo.setFeedForward(s.get(), g.get(), v.get(), a.get()), s, g, v, a);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            armIo.setProfile(
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
    armIo.runPosition(angle, Amps.of(0));
  }

  public Angle getAngle() {
    return Radians.of(inputs.absoluteEncoderPositionRads);
  }

  private Command goTo(Angle angle) {
    return Commands.run(() -> setAngle(angle), this);
  }

  private Angle getAngleForReefLevel(ReefLevel reefLevel) {
    LoggedTunableNumber number =
        switch (reefLevel) {
          case LEVEL_1 -> troughPositionDegrees;
          case LEVEL_2, LEVEL_3 -> levelTwoAndThreePositionDegrees;
          case LEVEL_4 -> levelFourPositionDegrees;
        };
    return Degrees.of(number.get());
  }

  private Command waitUntilAbove(Angle angle) {
    Debouncer debouncer = new Debouncer(debounceTime.get());
    Logger.recordOutput("Arm/waitForAngleAbove", angle.baseUnitMagnitude());
    return Commands.waitUntil(
        () -> {
          boolean done =
              debouncer.calculate(getAngle().baseUnitMagnitude() > angle.baseUnitMagnitude());
          Logger.recordOutput("Arm/waitForAngleGet", getAngle().baseUnitMagnitude());
          return done;
        });
  }

  private Command waitUntilBelow(Angle angle) {
    Debouncer debouncer = new Debouncer(debounceTime.get());
    Logger.recordOutput("Arm/waitForAngleBelow", angle.baseUnitMagnitude());
    return Commands.waitUntil(
        () -> {
          boolean done =
              debouncer.calculate(getAngle().baseUnitMagnitude() < angle.baseUnitMagnitude());
          Logger.recordOutput("Arm/waitForAngleGet", getAngle().baseUnitMagnitude());
          return done;
        });
  }

  public Command waitForCoralPresent() {
    // TODO: configure the sensor range with GrappleHook
    // then, validate the coralDetectionDistance
    if (inputs.sensorConnected) {
      // Wait until the coral is present for a minimum amount of time
      Debouncer sensorDebouncer = new Debouncer(0.100);
      return Commands.waitUntil(
          () ->
              sensorDebouncer.calculate(
                  inputs.sensorDistanceMillimeters < coralDetectionDistance.get()));
    } else {
      return Commands.waitSeconds(.4);
    }
  }

  public Command waitForCoralGone() {
    // TODO: validate
    if (inputs.sensorConnected) {
      // Wait until the coral is gone for a minimum amount of time
      Debouncer sensorDebouncer = new Debouncer(0.100);
      return Commands.waitUntil(
          () -> {
            // sensorDebouncer.calculate(
            System.out.println(inputs.sensorDistanceMillimeters > coralScoreDistance.get());
            return inputs.sensorDistanceMillimeters > coralScoreDistance.get();
          });
    } else {
      return Commands.waitUntil(() -> false);
    }
  }

  public Command waitForCollectPosition() {
    return waitUntilBelow(Degrees.of(constants.collectPositionDegrees() + 3.0));
  }

  public Command waitForStowPosition() {
    return waitUntilAbove(Degrees.of(constants.stowPositionDegrees() - 10.0));
  }

  public Command stow() {
    return goTo(Degrees.of(constants.stowPositionDegrees()));
  }

  public Command collect() {
    return goTo(Degrees.of(constants.collectPositionDegrees()));
  }

  public Command score() {
    return goTo(Degrees.of(0));
  }

  public Command reefLevel(ReefLevel reefLevel) {
    return Commands.run(() -> setAngle(getAngleForReefLevel(reefLevel)), this);
  }
}
