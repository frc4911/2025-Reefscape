// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.commands;

import dagger.Module;
import dagger.multibindings.Multibinds;
import java.util.Set;

@Module
public interface CommandsModule {

  // Ensure the set is always there
  @Multibinds
  public abstract Set<VirtualSubsystem> providesEmptySet();
}
