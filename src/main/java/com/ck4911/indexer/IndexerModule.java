// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.indexer;

import com.ck4911.Constants;
import com.ck4911.util.PidValues;
import dagger.Module;
import dagger.Provides;
import javax.inject.Provider;

@Module
public interface IndexerModule {
  @Provides
  static IndexerIo providesIndexerIo(Constants.Mode mode, Provider<IndexerIoReal> realProvider) {
    return switch (mode) {
      case REAL -> realProvider.get();
      default -> new IndexerIo() {};
    };
  }

  @Provides
  static IndexerConstants provideIndexerConstants() {
    return IndexerConstantsBuilder.builder().feedBackValues(new PidValues(0, 0, 0)).build();
  }
}
