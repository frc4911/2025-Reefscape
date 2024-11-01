// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.commands;

import dagger.Module;
import dagger.Provides;
import dagger.multibindings.ElementsIntoSet;
import java.util.Collections;
import java.util.Set;

@Module
public interface CommandsModule {

  // Ensure the set is always there
  @Provides
  @ElementsIntoSet
  public static Set<VirtualSubsystem> providesEmpty() {
    return Collections.emptySet();
  }
}
