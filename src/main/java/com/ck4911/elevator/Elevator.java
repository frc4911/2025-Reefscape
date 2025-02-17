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
import static edu.wpi.first.units.Units.Rotations;
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
  private final LoggedTunableNumber tolerance;
  private final LoggedTunableNumber homingTimeSeconds;
  private final Alert leaderDisonnected;
  private final Alert followerDisconnected;

  private Debouncer homingDebouncer;

  private double homedPosition = 0;

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
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("Elevator_State", state.toString())),
            new SysIdRoutine.Mechanism(elevatorIo::runVolts, null, this));

    p = tunableNumbers.create("Elevator/p", constants.feedBackValues().p());
    i = tunableNumbers.create("Elevator/i", constants.feedBackValues().i());
    d = tunableNumbers.create("Elevator/d", constants.feedBackValues().d());
    s = tunableNumbers.create("Elevator/s", constants.feedForwardValues().s());
    g = tunableNumbers.create("Elevator/g", constants.feedForwardValues().g());
    v = tunableNumbers.create("Elevator/v", constants.feedForwardValues().v());
    a = tunableNumbers.create("Elevator/a", constants.feedForwardValues().a());
    tolerance = tunableNumbers.create("Elevator/Tolerance", constants.tolerance());
    homingTimeSeconds =
        tunableNumbers.create("Elevator/HomingTimeSecs", constants.homingTimeSeconds());

    homingDebouncer = new Debouncer(homingTimeSeconds.get());

    elevatorIo.setPid(p.get(), i.get(), d.get());
    elevatorIo.setFeedForward(s.get(), g.get(), v.get(), a.get());

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
        () -> homingDebouncer = new Debouncer(homingTimeSeconds.get()),
        homingTimeSeconds);
    checkLimits();
  }

  @Override
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  public void setAngle(Angle angle) {
    // TODO: Use Alerts and also cap angles beyond limits
    if (angle.compareTo(Radians.of(constants.maxPositionRads())) > 0) {
      System.out.println("ERROR: angle above max");
      return;
    }
    if (angle.compareTo(Radians.of(constants.minPositionRads())) < 0) {
      System.out.println("ERROR: angle below min");
      return;
    }
    // TODO: calculate feed forward
    elevatorIo.runPosition(angle, Amps.of(0));
  }

  public Angle getAngle() {
    return Radians.of(inputs.positionRads);
  }

  public Distance getPosition() {
    Distance position =
        Meters.of(constants.sprocketRadius() * (inputs.positionRads - homedPosition));
    Logger.recordOutput("Elevator/MeasuredHeightMeters", position.baseUnitMagnitude());
    return position;
  }

  private Command goTo(Angle angle) {
    return Commands.runOnce(() -> setAngle(angle), this)
        .andThen(Commands.waitUntil(() -> getAngle().isNear(angle, tolerance.get())));
  }

  public Command stow() {
    return goTo(Rotations.of(constants.stowPositionRotations()));
  }

  public Command prepareCollect() {
    return goTo(Rotations.of(constants.prepareCollectPositionRotations()));
  }

  public Command collect() {
    return goTo(Rotations.of(constants.collectPositionRotations()));
  }

  public Command trough() {
    return goTo(Rotations.of(constants.troughPositionRotations()));
  }

  public Command levelTwo() {
    return goTo(Rotations.of(constants.levelTwoPositionRotations()));
  }

  public Command levelThree() {
    return goTo(Rotations.of(constants.levelThreePositionRotations()));
  }

  public Command levelFour() {
    return goTo(Rotations.of(constants.levelFourPositionRotations()));
  }

  // TODO: a command to gently move the elevator to the home position and mark it.
  public Command home() {
    return Commands.none();
  }

  private void checkLimits() {
    // TODO: check position limits (upper and lower)
  }
}
