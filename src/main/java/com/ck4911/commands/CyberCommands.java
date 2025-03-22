// Copyright (c) 2025 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.ck4911.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ck4911.arm.Arm;
import com.ck4911.drive.Drive;
import com.ck4911.elevator.Elevator;
import com.ck4911.field.ReefLevel;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public final class CyberCommands {
  private static final double WHEEL_RADIUS_MAX_VELOCITY =
      RadiansPerSecond.convertFrom(.5, RotationsPerSecond);
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final double DRIVEBASE_RADIUS = Math.sqrt(22.72 * 22.72 + 22.72 * 22.72);

  private final Arm arm;
  private final Elevator elevator;
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<Pose2d> poseConsumer;

  @Inject
  public CyberCommands(
      Arm arm, Elevator elevator, Supplier<Pose2d> poseSupplier, Consumer<Pose2d> poseConsumer) {
    this.arm = arm;
    this.elevator = elevator;
    this.poseSupplier = poseSupplier;
    this.poseConsumer = poseConsumer;
  }

  public Command prepareForCollect() {
    return Commands.sequence(
        elevator.prepareCollect().raceWith(elevator.waitUntilPrepareCollect()),
        elevator.prepareCollect().alongWith(arm.collect()));
  }

  public Command collect() {
    return Commands.sequence(
        elevator.collect().raceWith(arm.waitForCoralPresent()),
        elevator.passCorral().raceWith(elevator.waitUntilPrepareCollect()),
        stow());
  }

  public Command score() {
    return arm.score().raceWith(arm.waitForCoralGone()).andThen(Commands.print("Done"));
  }

  public Command home() {
    return elevator.home().alongWith(arm.stow());
  }

  public Command homeWithCoral() {
    return elevator.homeWithCoral().alongWith(arm.stow());
  }

  public Command reefLevel(ReefLevel reefLevel) {
    return elevator.reefLevel(reefLevel).alongWith(arm.reefLevel(reefLevel));
  }

  public Command stow() {
    // TODO: Check if clear of corral first
    // If the elevator is too low, this could be bad
    return arm.stow().raceWith(arm.waitForStowPosition().andThen(elevator.stow()));
  }

  // TODO: check with driver for which direction is prefered for zeroing
  // currently, this assumes facing away from driver station.
  public Command resetForward(Angle forwardAngle) {
    return Commands.runOnce(
        () ->
            poseConsumer.accept(
                new Pose2d(poseSupplier.get().getTranslation(), new Rotation2d(forwardAngle))));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
    SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> limiter.reset(0.0)),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.setControl(applyFieldSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, speed)));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getDrivePositionRadians();
                  state.lastAngle = poseSupplier.get().getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = poseSupplier.get().getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;

                      List<Double> positions = drive.getDrivePositionRadians();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions.get(i) - state.positions.get(i)) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * DRIVEBASE_RADIUS) / wheelDelta;

                      Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                      Logger.recordOutput("Drive/WheelRadius", wheelRadius);
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      List<Double> positions = drive.getDrivePositionRadians();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions.get(i) - state.positions.get(i)) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * DRIVEBASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000000000000000000000000000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    List<Double> positions = new ArrayList<>();
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
