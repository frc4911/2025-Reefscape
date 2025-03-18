// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.auto;

import static edu.wpi.first.units.Units.Degrees;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ck4911.commands.CyberCommands;
import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.drive.Drive;
import com.ck4911.quest.QuestNav;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public final class AutoCommandHandler implements VirtualSubsystem {
  private final AutoChooser autoChooser;
  private final AutoFactory autoFactory;
  private final Drive drive;
  private final QuestNav questNav;
  private final CyberCommands cyberCommands;
  private double autoStart;
  private boolean autoMessagePrinted;
  private Command currentAutoCommand;

  @Inject
  public AutoCommandHandler(
      AutoFactory autoFactory,
      Drive drive,
      QuestNav questNav,
      CyberCommands cyberCommands,
      AutoChooser autoChooser) {
    this.autoChooser = autoChooser;
    this.drive = drive;
    this.questNav = questNav;
    this.cyberCommands = cyberCommands;
    this.autoFactory = autoFactory;

    bindNamedCommands();
    addAutos();
  }

  @Override
  public void periodic() {
    // Print auto duration
    if (currentAutoCommand != null) {
      if (!currentAutoCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }
  }

  private void bindNamedCommands() {
    autoFactory.bind("Home", cyberCommands.home());
    // TODO: bind name commands so that waypoints trigger them
  }

  private void addAutos() {
    autoChooser.addCmd("test", () -> Commands.print("hi"));
    autoChooser.addRoutine("Middle Score L4", this::middleScoreL4);
    autoChooser.addRoutine("Middle Score L4 and Collect", this::middleScoreL4AndCollect);
    autoChooser.addRoutine("gpTapeAuto", this::gpTapeAuto);
    autoChooser.addRoutine("pleaseWork", this::pleaseWork);
    autoChooser.addRoutine("Distance Test", this::distanceTest);
    autoChooser.addCmd("Wheel Radius", () -> cyberCommands.wheelRadiusCharacterization(drive));
    autoChooser.addCmd(
        "Leave",
        () ->
            Commands.sequence(
                autoFactory.resetOdometry("Leave"),
                autoFactory.trajectoryCmd("Leave"),
                Commands.runOnce(
                    () -> {
                      boolean red =
                          DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red;
                      Angle forwardAngle = Degrees.of(red ? 180 : 0);
                      CommandScheduler.getInstance()
                          .schedule(cyberCommands.resetForward(forwardAngle));
                    })));

    autoChooser.addCmd("Quest Offset Calibration", () -> questNav.calibrateCommand(drive));

    SmartDashboard.putData("Autos", autoChooser);
  }

  public AutoRoutine pleaseWork() {
    AutoRoutine routine = autoFactory.newRoutine("pleaseWork");

    AutoTrajectory pleaseWork = routine.trajectory("pleaseWork");

    routine
        .active()
        .onTrue(
            Commands.sequence(pleaseWork.resetOdometry(), pleaseWork.cmd())
                .alongWith(cyberCommands.levelFour())); // up to

    // pleaseWork.atTime("L4").onTrue(cyberCommands.levelFour());

    pleaseWork
        .done()
        .onTrue(
            Commands.sequence(cyberCommands.score())
                .andThen(Commands.waitSeconds(3).andThen(cyberCommands.prepareForCollect())));

    return routine;
  }

  public AutoRoutine distanceTest() {
    List<Double> startingPositions = drive.getDrivePositionRadians();
    AutoRoutine routine = autoFactory.newRoutine("Distance Test");
    AutoTrajectory meterTest = routine.trajectory("Distance Test");
    routine.active().onTrue(Commands.sequence(meterTest.resetOdometry(), meterTest.cmd()));
    meterTest
        .done()
        .onTrue(
            Commands.runOnce(
                () -> {
                  List<Double> endingPositions = drive.getDrivePositionRadians();
                  double averageDistance = 0;
                  System.out.println("------------------------------------");
                  for (int i = 0; i < startingPositions.size(); i++) {
                    double distance = Math.abs(endingPositions.get(i) - startingPositions.get(i));
                    averageDistance += distance;
                    System.out.println("Start: " + startingPositions.get(i));
                    System.out.println("End: " + endingPositions.get(i));
                    System.out.println("Rotational distance traveled(radians): " + distance);
                  }
                  averageDistance = averageDistance / 4.0;
                  System.out.println("Average: " + averageDistance);
                  System.out.println("Divide linear distance by this to get wheel radius");
                  System.out.println("------------------------------------");
                }));
    return routine;
  }

  public AutoRoutine gpTapeAuto() {
    AutoRoutine routine = autoFactory.newRoutine("gpTapeAuto");

    // Load the trajectory
    AutoTrajectory gpTapeAuto = routine.trajectory("gpTapeAuto");

    routine.active().onTrue(Commands.sequence(gpTapeAuto.resetOdometry(), gpTapeAuto.cmd()));

    gpTapeAuto.atTime("Trough").onTrue(cyberCommands.trough());
    gpTapeAuto.atTime("Stow").onTrue(cyberCommands.stow());
    gpTapeAuto.atTime("Score").onTrue(cyberCommands.score());

    return routine;
  }

  public AutoRoutine middleScoreL4() {
    AutoRoutine routine = autoFactory.newRoutine("Middle Score L4");

    AutoTrajectory middleScoreL4 = routine.trajectory("Middle Score L4");

    routine.active().onTrue(Commands.sequence(middleScoreL4.resetOdometry(), middleScoreL4.cmd()));

    middleScoreL4.atTime("L4").onTrue(Commands.print("L4").alongWith(cyberCommands.levelFour()));
    middleScoreL4
        .atTime("ScoreDown")
        .onTrue(
            Commands.print("ScoreDown")
                .alongWith(
                    cyberCommands
                        .score()
                        .raceWith(Commands.waitSeconds(.5))
                        .andThen(cyberCommands.prepareForCollect())));

    return routine;
  }

  public AutoRoutine middleScoreL4AndCollect() {
    AutoRoutine routine = autoFactory.newRoutine("Middle Score L4 And Collect");

    AutoTrajectory middleScoreL4 = routine.trajectory("Middle Score L4");
    AutoTrajectory collectRight = routine.trajectory("Collect Right");

    routine.active().onTrue(Commands.sequence(middleScoreL4.resetOdometry(), middleScoreL4.cmd()));

    middleScoreL4.atTime("L4").onTrue(Commands.print("L4").alongWith(cyberCommands.levelFour()));
    middleScoreL4
        .atTime("ScoreDown")
        .onTrue(
            Commands.print("ScoreDown")
                .alongWith(
                    cyberCommands
                        .score()
                        .raceWith(Commands.waitSeconds(.5))
                        .andThen(
                            cyberCommands.prepareForCollect().raceWith(Commands.waitSeconds(1)))));

    middleScoreL4.done().onTrue(collectRight.cmd());

    collectRight.active().whileTrue(cyberCommands.prepareForCollect());

    return routine;
  }

  public void startCurrentCommand() {
    stopCurrentCommand();
    autoStart = Timer.getFPGATimestamp();
    currentAutoCommand = autoChooser.selectedCommandScheduler();
    if (currentAutoCommand != null) {
      currentAutoCommand.schedule();
    }
  }

  public void stopCurrentCommand() {
    if (currentAutoCommand != null) {
      currentAutoCommand.cancel();
    }
  }
}
