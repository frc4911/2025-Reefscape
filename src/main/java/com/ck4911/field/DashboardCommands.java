// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.field;

import com.ck4911.commands.CyberCommands;
import com.ck4911.commands.VirtualSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@Singleton
public final class DashboardCommands implements VirtualSubsystem {
  private final CyberCommands cyberCommands;
  private final Field2d field;
  private final AutoAlign.AutoAlignFactory factory;
  private final LoggedDashboardChooser<ReefLevel> levelChooser =
      new LoggedDashboardChooser<>("Reef Level");
  private final LoggedDashboardChooser<ReefBranch> branchChooser =
      new LoggedDashboardChooser<>("Reef Branch");

  @Inject
  DashboardCommands(
      CyberCommands cyberCommands, Field2d field, AutoAlign.AutoAlignFactory factory) {
    this.cyberCommands = cyberCommands;
    this.field = field;
    this.factory = factory;
  }

  public void addAllReefPositions() {
    branchChooser.addDefaultOption("BRANCH A", ReefBranch.A);
    branchChooser.addOption("BRANCH B", ReefBranch.B);
    branchChooser.addOption("BRANCH C", ReefBranch.C);
    branchChooser.addOption("BRANCH D", ReefBranch.D);
    branchChooser.addOption("BRANCH E", ReefBranch.E);
    branchChooser.addOption("BRANCH F", ReefBranch.F);
    branchChooser.addOption("BRANCH G", ReefBranch.G);
    branchChooser.addOption("BRANCH H", ReefBranch.H);
    branchChooser.addOption("BRANCH I", ReefBranch.I);
    branchChooser.addOption("BRANCH J", ReefBranch.J);
    branchChooser.addOption("BRANCH K", ReefBranch.K);
    branchChooser.addOption("BRANCH L", ReefBranch.L);
  }

  public void addAllReefLevels() {
    levelChooser.addDefaultOption("LEVEL 4", ReefLevel.LEVEL_4);
    levelChooser.addOption("LEVEL 3", ReefLevel.LEVEL_3);
    levelChooser.addOption("LEVEL 2", ReefLevel.LEVEL_2);
  }

  @Override
  public void periodic() {
    field.getObject("NextReefPosition").setPose(branchChooser.get().targetPose);
  }

  public Command goToCurrentReefPosition() {
    // This should be deferred or else it will always use the initial branch instead of the current
    return Commands.deferredProxy(() -> factory.alignWith(branchChooser.get().targetPose));
  }

  public Command goToCurrentReefLevel() {
    // This should be deferred or else it will always use the initial level instead of the current
    return Commands.deferredProxy(() -> cyberCommands.reefLevel(levelChooser.get()));
  }
}
