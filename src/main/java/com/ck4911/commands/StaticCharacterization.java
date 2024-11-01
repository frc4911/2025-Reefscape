// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.commands;

import com.ck4911.util.LoggedTunableNumber;
import com.ck4911.util.LoggedTunableNumber.TunableNumbers;
import dagger.assisted.Assisted;
import dagger.assisted.AssistedFactory;
import dagger.assisted.AssistedInject;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public final class StaticCharacterization extends Command {
  private final LoggedTunableNumber currentRampFactor;
  private final LoggedTunableNumber minVelocity;
  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  @AssistedFactory
  public interface StaticCharacterizationFactory {
    StaticCharacterization create(
        Subsystem subsystem,
        DoubleConsumer characterizationInputConsumer,
        DoubleSupplier velocitySupplier);
  }

  @AssistedInject
  public StaticCharacterization(
      TunableNumbers tunableNumbers,
      @Assisted Subsystem subsystem,
      @Assisted DoubleConsumer characterizationInputConsumer,
      @Assisted DoubleSupplier velocitySupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;

    currentRampFactor = tunableNumbers.create("StaticCharacterization/CurrentRampPerSec", 1.0);
    minVelocity = tunableNumbers.create("StaticCharacterization/MinStaticVelocity", 0.1);

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.get();
    inputConsumer.accept(currentInput);
  }

  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.get();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Static Characterization output: " + currentInput + " amps");
    inputConsumer.accept(0);
  }
}
