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
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ck4911.characterization.Characterizable;
import com.ck4911.util.Alert;
import com.ck4911.util.LoggedTunableNumber;
import com.ck4911.util.LoggedTunableNumber.TunableNumbers;
import com.ctre.phoenix6.SignalLogger;
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
    armIo.setPid(p.get(), i.get(), d.get());
    armIo.setFeedForward(s.get(), g.get(), v.get(), a.get());

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
  }

  @Override
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  public void setAngle(Angle angle) {
    // TODO: calculate feed forward
    armIo.runPosition(angle, Amps.of(0));
  }

  public Angle getAngle() {
    return Radians.of(inputs.absoluteEncoderPositionRads);
  }

  private Command goTo(Angle angle) {
    return Commands.runOnce(() -> setAngle(angle), this)
        .andThen(Commands.waitUntil(() -> getAngle().isNear(angle, .01)));
  }

  public Command stowPosition() {
    return goTo(Degrees.of(constants.stowPositionDegrees()));
  }

  public Command collectPosition() {
    return goTo(Degrees.of(constants.collectPositionDegrees()));
  }

  public Command prepareScoreL1Position() {
    return goTo(Degrees.of(constants.troughPositionDegrees()));
  }

  public Command prepareScoreMidPosition() {
    return goTo(Degrees.of(constants.levelTwoAndThreePositionDegrees()));
  }

  public Command prepareScoreL4Position() {
    return goTo(Degrees.of(constants.levelFourPositionDegrees()));
  }
}
