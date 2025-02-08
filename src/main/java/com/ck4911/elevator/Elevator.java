// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ck4911.characterization.Characterizable;
import com.ck4911.util.Alert;
import com.ck4911.util.LoggedTunableNumber;
import com.ck4911.util.LoggedTunableNumber.TunableNumbers;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public final class Elevator extends SubsystemBase implements Characterizable {
  private final ElevatorIo elevatorIo;
  private final ElevatorIoInputsAutoLogged inputs = new ElevatorIoInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;
  private final LoggedTunableNumber p;
  private final LoggedTunableNumber i;
  private final LoggedTunableNumber d;
  private final LoggedTunableNumber s;
  private final LoggedTunableNumber g;
  private final LoggedTunableNumber v;
  private final LoggedTunableNumber a;
  private final Alert leaderDisonnected;
  private final Alert followerDisconnected;

  @Inject
  public Elevator(
      ElevatorIo elevatorIo, ElevatorConstants constants, TunableNumbers tunableNumbers) {
    this.elevatorIo = elevatorIo;
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
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
  }

  @Override
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }
}
