// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ck4911.characterization.Characterizable;
import com.ck4911.quest.QuestNav;
import com.ck4911.util.LoggedTunableNumber;
import com.ck4911.util.LoggedTunableNumber.TunableNumbers;
import com.ck4911.util.PhoenixUtils;
import com.ck4911.vision.VisionConsumer;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
@Singleton
public class Drive extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements Subsystem, VisionConsumer, Characterizable {
  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

  private Notifier simNotifier = null;
  private double lastSimTime;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  private final QuestNav questNav;
  private final LoggedTunableNumber driveP;
  private final LoggedTunableNumber driveD;
  private final LoggedTunableNumber driveS;
  private final LoggedTunableNumber driveV;
  private final LoggedTunableNumber driveA;
  private final LoggedTunableNumber turnP;
  private final LoggedTunableNumber turnD;
  private final List<DriveLogger> driveLoggers = new ArrayList<>();
  private final Field2d field;

  @Inject
  Drive(QuestNav questNav, TunableNumbers tunableNumbers, Field2d field) {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
    this.questNav = questNav;
    this.field = field;
    driveP = tunableNumbers.create("Drive/driveP", TunerConstants.driveGains.kP);
    driveD = tunableNumbers.create("Drive/driveD", TunerConstants.driveGains.kD);
    driveS = tunableNumbers.create("Drive/driveS", TunerConstants.driveGains.kS);
    driveV = tunableNumbers.create("Drive/driveV", TunerConstants.driveGains.kV);
    driveA = tunableNumbers.create("Drive/driveA", TunerConstants.driveGains.kA);
    turnP = tunableNumbers.create("Drive/turnP", TunerConstants.steerGains.kP);
    turnD = tunableNumbers.create("Drive/turnD", TunerConstants.steerGains.kD);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = getModules();
    driveLoggers.add(new DriveLogger("FrontLeft", modules[0]));
    driveLoggers.add(new DriveLogger("FrontRight", modules[1]));
    driveLoggers.add(new DriveLogger("BackLeft", modules[2]));
    driveLoggers.add(new DriveLogger("BackRight", modules[3]));

    SmartDashboard.putData("Field", field);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param requestSupplier Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? RED_ALLIANCE_PERSPECTIVE_ROTATION
                        : BLUE_ALLIANCE_PERSPECTIVE_ROTATION);
                hasAppliedOperatorPerspective = true;
              });
    }
    Logger.recordOutput("SwerveStates/Measured", getState().ModuleStates);
    Logger.recordOutput("Drive/OdometryPose", getState().Pose);
    for (DriveLogger logger : driveLoggers) {
      logger.updateInputs();
    }
    field.getRobotObject().setPose(getState().Pose);

    LoggedTunableNumber.ifChanged(
        hashCode(), this::updateDriveGains, driveP, driveD, driveS, driveV, driveA);
    LoggedTunableNumber.ifChanged(hashCode(), this::updateSteerGains, turnP, turnD);
  }

  private void updateDriveGains() {
    Slot0Configs newDriveGains =
        TunerConstants.driveGains
            .withKP(driveP.get())
            .withKD(driveD.get())
            .withKS(driveS.get())
            .withKV(driveV.get())
            .withKA(driveA.get());
    SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = getModules();
    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : modules) {
      PhoenixUtils.tryUntilOk(
          5, () -> module.getDriveMotor().getConfigurator().apply(newDriveGains));
    }
  }

  private void updateSteerGains() {
    Slot0Configs newSteerGains = TunerConstants.steerGains.withKP(turnP.get()).withKD(turnD.get());
    SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = getModules();
    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : modules) {
      PhoenixUtils.tryUntilOk(
          5, () -> module.getSteerMotor().getConfigurator().apply(newSteerGains));
    }
  }

  @Override
  public SysIdRoutine getSysIdRoutine() {
    // The SysId routine to test
    return sysIdRoutineTranslation;
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  @Override
  public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    // Convert timestamp
    // https://www.chiefdelphi.com/t/phoenix-6-pose-estimator-phoenix-x-serve/481470/4
    addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestamp), stdDevs);
  }

  public List<Double> getDrivePositionRadians() {
    return driveLoggers.stream()
        .map(DriveLogger::getDrivePositionRadians)
        .collect(Collectors.toList());
  }
}
