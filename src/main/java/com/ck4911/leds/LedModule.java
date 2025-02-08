// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.leds;

import com.ck4911.commands.VirtualSubsystem;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;

@Module
public interface LedModule {
  @Binds
  @IntoSet
  public VirtualSubsystem bindsLeds(Leds leds);

  @Provides
  public static LedConstants providesLedConstants() {
    return LedConstantsBuilder.builder()
        .pwmPort(0)
        .minLoopCycleCount(10)
        .length(12)
        .strobeDuration(0.1)
        .breathDuration(1.0)
        .waveExponent(0.4)
        .waveFastCycleLength(25.0)
        .waveFastDuration(0.25)
        .waveAllianceCycleLength(15.0)
        .waveAllianceDuration(2.0)
        .autoFadeTime(2.5) // 3s nominal
        .autoFadeMaxTime(5.0) // Return to normal
        .build();
  }
}
