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

  static CyberKnightsController createForBrand(int port, Brand brand) {
    switch (brand) {
      case STADIA:
        return new CyberKnightsStadiaController(port);
      case XBOX:
        return new CyberKnightsXboxController(port);
      default:
        throw new IllegalArgumentException("No implementation found for brand: " + brand);
    }
  }

  enum Brand {
    XBOX,
    STADIA
  }

  GenericHID getHID();

  Trigger leftBumper();

  Trigger rightBumper();

  Trigger leftStick();

  Trigger rightStick();

  Trigger leftTrigger();

  Trigger rightTrigger();

  double getLeftX();

  double getRightX();

  double getLeftY();

  double getRightY();

  Trigger a();

  Trigger b();

  Trigger x();

  Trigger y();

  Trigger povUp();

  Trigger povRight();

  Trigger povDown();

  Trigger povLeft();
}
