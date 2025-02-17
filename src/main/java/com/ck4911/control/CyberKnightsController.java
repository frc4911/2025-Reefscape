// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A wrapper interface for {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}
 * implementations. Oddly, these concrete classes don't share an interface despite sharing many
 * identical methods for joysticks, buttons, and D-pads. This wrapper provides a way to swap
 * different controller brands without larger control binding re-writes or duplicative code.
 */
public interface CyberKnightsController {

  public static CyberKnightsController createForBrand(int port, Brand brand) {
    switch (brand) {
      case STADIA:
        return new CyberKnightsStadiaController(port);
      case XBOX:
        return new CyberKnightsXboxController(port);
      default:
        throw new IllegalArgumentException("No implementation found for brand: " + brand);
    }
  }

  public enum Brand {
    XBOX,
    STADIA;
  }

  public GenericHID getHID();

  public Trigger leftBumper();

  public Trigger rightBumper();

  public Trigger leftStick();

  public Trigger rightStick();

  public Trigger leftTrigger();

  public Trigger rightTrigger();

  public double getLeftX();

  public double getRightX();

  public double getLeftY();

  public double getRightY();

  public Trigger a();

  public Trigger b();

  public Trigger x();

  public Trigger y();

  public Trigger povUp();

  public Trigger povRight();

  public Trigger povDown();

  public Trigger povLeft();
}
