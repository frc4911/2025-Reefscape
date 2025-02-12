// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public final class PhoenixUtils {

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      StatusCode statusCode = command.get();
      if (statusCode.isOK()) {
        break;
      }
    }
  }

  private PhoenixUtils() {}
}
