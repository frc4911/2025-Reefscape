// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import com.ck4911.Constants.Mode;
import com.ck4911.commands.VirtualSubsystem;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;

@Module
public interface AutoModule {
  @Provides
  public static AutoConstants provideAutoConstants(Mode mode) {
    return AutoConstantsBuilder.builder().xkD(10.0).ykD(10.0).thetakD(7.0).build();
  }

  @Binds
  @IntoSet
  public VirtualSubsystem bindsAutoCommandHandler(AutoCommandHandler autoCommandHandler);
}
