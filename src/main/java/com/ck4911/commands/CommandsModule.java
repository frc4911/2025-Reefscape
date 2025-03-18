// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.commands;

import dagger.Binds;
import dagger.Module;
import dagger.multibindings.IntoSet;
import dagger.multibindings.Multibinds;
import java.util.Set;

@Module
public interface CommandsModule {

  // Ensure the set is always there
  @Multibinds
  public abstract Set<VirtualSubsystem> providesEmptySet();

  @Binds
  @IntoSet
  public VirtualSubsystem bindsDashboardCommands(DashboardCommands dashboardCommands);
}
