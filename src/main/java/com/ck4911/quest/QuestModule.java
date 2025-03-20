// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.quest;

import com.ck4911.Constants.Mode;
import com.ck4911.commands.VirtualSubsystem;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;
import edu.wpi.first.math.geometry.Transform2d;
import javax.inject.Provider;

@Module
public interface QuestModule {

  @Provides
  static QuestIo providesQuestIo(Mode mode, Provider<QuestIoReal> realProvider) {
    switch (mode) {
      case REAL:
        return realProvider.get();
      default:
        return new QuestIo() {};
    }
  }

  @Provides
  public static QuestConstants providesQuestConstants() {
    return QuestConstantsBuilder.builder().robotToQuest(new Transform2d()).build();
  }

  @Binds
  @IntoSet
  public VirtualSubsystem bindsQuestNav(QuestNav questNav);
}
