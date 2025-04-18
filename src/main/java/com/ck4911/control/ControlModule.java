// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.control;

import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.control.Controller.Role;
import com.ck4911.control.CyberKnightsController.Brand;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;
import javax.inject.Singleton;

@Module
public interface ControlModule {
  @Provides
  public static ControlConstants provideControlConstants() {
    return ControlConstantsBuilder.builder()
        .deadband(0.1)
        .sniperScale(0.1)
        .driverPort(0)
        .operatorPort(1)
        .build();
  }

  @Provides
  @Controller(Role.DRIVER)
  public static Brand provideDriverControllerBrand() {
    return Brand.STADIA;
  }

  @Provides
  @Controller(Role.OPERATOR)
  public static Brand provideOperatorControllerBrand() {
    return Brand.STADIA;
  }

  @Singleton
  @Provides
  @Controller(Role.DRIVER)
  public static CyberKnightsController provideDriverController(
      @Controller(Role.DRIVER) Brand brand, ControlConstants constants) {
    return CyberKnightsController.createForBrand(constants.driverPort(), brand);
  }

  @Singleton
  @Provides
  @Controller(Role.OPERATOR)
  public static CyberKnightsController provideOperatorController(
      @Controller(Role.OPERATOR) Brand brand, ControlConstants constants) {
    return CyberKnightsController.createForBrand(constants.operatorPort(), brand);
  }

  @Binds
  @IntoSet
  public VirtualSubsystem bindsControllerBinding(ControllerBinding controllerBinding);
}
