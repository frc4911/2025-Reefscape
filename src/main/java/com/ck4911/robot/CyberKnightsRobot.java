// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.robot;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ck4911.BuildConstants;
import com.ck4911.Constants.Mode;
import com.ck4911.commands.VirtualSubsystem;
import com.ck4911.util.Alert;
import com.ck4911.util.Alert.AlertType;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import javax.inject.Inject;
import javax.inject.Named;
import javax.inject.Provider;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class CyberKnightsRobot extends LoggedRobot {
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double canivoreErrorTimeThreshold = 0.5;
  private static final double lowBatteryVoltage = 11.8;
  private static final double lowBatteryDisabledTime = 1.5;

  private RobotContainer robotContainer;
  private final Timer disabledTimer = new Timer();
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private double teleStart;
  private static double teleElapsedTime = 0.0;

  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);
  private final Alert canivoreErrorAlert =
      new Alert("CANivore error detected, robot may not be controllable.", AlertType.ERROR);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.WARNING);
  private final Alert gcAlert =
      new Alert("Please wait to enable, collecting garbage. ", AlertType.WARNING);

  public static Trigger createTeleopTimeTrigger(DoubleSupplier teleElapsedTime) {
    return new Trigger(
        () ->
            DriverStation.isFMSAttached()
                && DriverStation.isTeleopEnabled()
                && CyberKnightsRobot.teleElapsedTime > teleElapsedTime.getAsDouble());
  }

  private final CommandScheduler scheduler;
  private final Set<VirtualSubsystem> virtualSubsystems;
  private final Mode robotMode;
  private LaserCan armLaserCan;
  private final String robotName;
  private final boolean tuningMode;
  private final CANBus canivore;
  private final Provider<RobotContainer> containerProvider;

  @Inject
  public CyberKnightsRobot(
      CommandScheduler scheduler,
      Set<VirtualSubsystem> virtualSubsystems,
      @Named("RobotName") String robotName,
      @Named("TuningMode") boolean tuningMode,
      @Named("Bob") CANBus canivore,
      Mode robotMode,
      Provider<RobotContainer> containerProvider) {
    super();
    this.scheduler = scheduler;
    this.virtualSubsystems = virtualSubsystems;
    this.robotName = robotName;
    this.tuningMode = tuningMode;
    this.robotMode = robotMode;
    this.canivore = canivore;
    this.containerProvider = containerProvider;

    if (tuningMode) {
      CanBridge.runTCP();
    }

    armLaserCan = new LaserCan(0);
    try {
      armLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      armLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      armLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Record metadata
    Logger.recordMetadata("Robot", robotName);
    Logger.recordMetadata("TuningMode", Boolean.toString(tuningMode));
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (robotMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    scheduler.onCommandInitialize(
        (Command command) -> {
          logCommandFunction.accept(command, true);
        });
    scheduler.onCommandFinish(
        (Command command) -> {
          logCommandFunction.accept(command, false);
        });
    scheduler.onCommandInterrupt(
        (Command command) -> {
          logCommandFunction.accept(command, false);
        });

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    RobotController.setBrownoutVoltage(6.0);
    robotContainer = containerProvider.get();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    for (VirtualSubsystem virtualSubsystem : virtualSubsystems) {
      virtualSubsystem.periodic();
    }
    scheduler.run();

    // Robot container periodic methods
    robotContainer.updateDashboardOutputs();
    // TODO: run other important checks

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

    // Log CANivore status
    if (robotMode == Mode.REAL) {
      var canivoreStatus = canivore.getStatus();
      Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.Status.getName());
      Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.BusUtilization);
      Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.BusOffCount);
      Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.TxFullCount);
      Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.REC);
      Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.TEC);
      if (!canivoreStatus.Status.isOK()
          || canStatus.transmitErrorCount > 0
          || canStatus.receiveErrorCount > 0) {
        canivoreErrorTimer.restart();
      }
      canivoreErrorAlert.set(
          !canivoreErrorTimer.hasElapsed(canivoreErrorTimeThreshold)
              && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));
    }

    // Low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      lowBatteryAlert.set(true);
    }

    // GC alert
    gcAlert.set(Timer.getFPGATimestamp() < 45.0);

    Threads.setCurrentThreadPriority(true, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.autonomousInit();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    robotContainer.teleopInit();

    teleStart = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    teleElapsedTime = Timer.getFPGATimestamp() - teleStart;
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    scheduler.cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
