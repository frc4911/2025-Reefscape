// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ck4911.util.Alert;
import com.ck4911.util.LoggedTunableNumber;
import com.ck4911.util.LoggedTunableNumber.TunableNumbers;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public final class Indexer extends SubsystemBase {

  private final IndexerIo indexerIo;
  private final IndexerIoInputsAutoLogged inputs = new IndexerIoInputsAutoLogged();
  private final LoggedTunableNumber p;
  private final LoggedTunableNumber i;
  private final LoggedTunableNumber d;
  private final LoggedTunableNumber velocityRadiansPerSecond;
  private final LoggedTunableNumber coralDetectionDistance;
  private final Alert motorDisconnected;

  @Inject
  Indexer(IndexerConstants constants, IndexerIo indexerIo, TunableNumbers tunableNumbers) {
    this.indexerIo = indexerIo;
    p = tunableNumbers.create("Indexer/p", constants.feedBackValues().p());
    i = tunableNumbers.create("Indexer/i", constants.feedBackValues().i());
    d = tunableNumbers.create("Indexer/d", constants.feedBackValues().d());
    velocityRadiansPerSecond =
        tunableNumbers.create(
            "Indexer/velocityRadiansPerSecond", constants.velocityRadiansPerSecond());
    coralDetectionDistance =
        tunableNumbers.create(
            "Indexer/CoralDetectMm", constants.coralDetectionDistanceMillimeters());
    motorDisconnected = new Alert("Indexer motor disconnected!", Alert.AlertType.WARNING);
  }

  @Override
  public void periodic() {
    indexerIo.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    motorDisconnected.set(!inputs.motorConnected);

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> indexerIo.setPid(p.get(), i.get(), d.get()), p, i, d);
  }

  public void setVelocity(AngularVelocity angularVelocity) {
    indexerIo.runVelocity(angularVelocity, Amps.of(0));
  }

  public Command indexAndHold() {
    return Commands.run(
            () -> setVelocity(RadiansPerSecond.of(velocityRadiansPerSecond.get())), this)
        .raceWith(waitForCoralPresent());
  }

  public Command feed() {
    return Commands.run(
            () -> setVelocity(RadiansPerSecond.of(velocityRadiansPerSecond.get())), this)
        .raceWith(waitForCoralGone());
  }

  public Command waitForCoralPresent() {
    Debouncer sensorDebouncer = new Debouncer(0.100);
    return Commands.waitUntil(
        () ->
            sensorDebouncer.calculate(
                inputs.sensorDistanceMillimeters < coralDetectionDistance.get()));
  }

  public Command waitForCoralGone() {
    // Wait until the coral is gone for a minimum amount of time
    Debouncer sensorDebouncer = new Debouncer(0.100);
    return Commands.waitUntil(
        () -> {
          return inputs.sensorDistanceMillimeters > coralDetectionDistance.get();
        });
  }
}
