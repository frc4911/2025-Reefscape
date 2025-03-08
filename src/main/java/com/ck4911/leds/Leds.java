// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.leds;

import com.ck4911.commands.VirtualSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.List;
import java.util.Optional;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class Leds implements VirtualSubsystem {

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean endgameAlert = false;
  public boolean autoFinished = false;
  public double autoFinishedTime = 0.0;
  public boolean lowBatteryAlert = false;
  public boolean demoMode = false;

  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kGold;
  private Color secondaryDisabledColor = Color.kDarkBlue;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // Constants
  private final LedConstants constants;

  @Inject
  public Leds(LedConstants constants) {
    this.constants = constants;
    leds = new AddressableLED(constants.pwmPort());
    buffer = new AddressableLEDBuffer(constants.length());
    leds.setLength(constants.length());
    leds.setData(buffer);
    leds.start();
    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (Leds.this) {
                breath(Color.kWhite, Color.kBlack, System.currentTimeMillis() / 1000.0);
                leds.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  @Override
  public synchronized void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      allianceColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(Color.kGold);
      secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : Color.kDarkBlue;
    }

    // Update auto state
    if (DriverStation.isDisabled()) {
      autoFinished = false;
    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < constants.minLoopCycleCount()) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // Select LED mode
    breath(Color.kRed, Color.kWhite); // Default to CyberKnights colors
    if (estopped) {
      solid(Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (lastEnabledAuto
          && Timer.getFPGATimestamp() - lastEnabledTime < constants.autoFadeMaxTime()) {
        // Auto fade
        solid(
            1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / constants.autoFadeTime()),
            Color.kGreen);
      } else if (lowBatteryAlert) {
        // Low battery
        breath(Color.kOrangeRed, Color.kYellow);
      } else {
        // Default pattern
        wave(
            allianceColor,
            secondaryDisabledColor,
            constants.waveAllianceCycleLength(),
            constants.waveAllianceDuration());
      }
    } else if (DriverStation.isAutonomous()) {
      wave(
          Color.kGold,
          Color.kDarkBlue,
          constants.waveFastCycleLength(),
          constants.waveFastDuration());
      if (autoFinished) {
        double fullTime =
            (double) constants.length()
                / constants.waveFastCycleLength()
                * constants.waveFastDuration();
        solid((Timer.getFPGATimestamp() - autoFinishedTime) / fullTime, Color.kGreen);
      }
    } else { // Enabled
      if (endgameAlert) {
        strobe(Color.kRed, Color.kGold, constants.strobeDuration());
      } else {
        rainbow(constants.waveFastCycleLength(), constants.waveFastDuration());
      }
    }

    // Update LEDs
    leds.setData(buffer);
  }

  private void solid(Color color) {
    if (color != null) {
      for (int i = 0; i < constants.length(); i++) {
        buffer.setLED(i, color);
      }
    }
  }

  private void solid(double percent, Color color) {
    for (int i = 0; i < MathUtil.clamp(constants.length() * percent, 0, constants.length()); i++) {
      buffer.setLED(i, color);
    }
  }

  private void strobe(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(c1On ? c1 : c2);
  }

  private void strobe(Color color, double duration) {
    strobe(color, Color.kBlack, duration);
  }

  private void breath(Color c1, Color c2) {
    breath(c1, c2, Timer.getFPGATimestamp());
  }

  private void breath(Color c1, Color c2, double timestamp) {
    double x =
        ((timestamp % constants.breathDuration()) / constants.breathDuration()) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }

  private void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < constants.length(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < constants.length(); i++) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), constants.waveExponent()) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), constants.waveExponent()) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
    }
  }

  private void stripes(List<Color> colors, int stripeLength, double duration) {
    int offset =
        (int) (Timer.getFPGATimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = 0; i < constants.length(); i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }
}
