// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.quest;

import com.ck4911.commands.VirtualSubsystem;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;

@Module
public interface QuestModule {

  @Provides
  public static QuestConstants providesQuestConstants() {
    return QuestConstantsBuilder.builder()
        .offsetAngleDegrees(-29.216252f)
        .offsetXInches(0)
        .offsetYInches(0)
        .build();
  }

  @Binds
  @IntoSet
  public VirtualSubsystem bindsQuestNav(QuestNav questNav);
}
