package com.ck4911.robot.Arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

import libraries.cyberlib.drivers.TalonFXFactory;


public class ArmIOReal implements ArmIO {
    // TODO: figure this out 👈(ﾟヮﾟ👈)
    private final TalonFX arm;
    private final CANcoder armEncoder;
    private final PIDCo armPidController;
    private final AnalogInput beamBreak;

    private final double armGearRatio;

    public ArmIOReal(ArmConstants constants) {
        armGearRatio = constants.armGearRatio();
        // arm = TalonFXFactory.createTalon(Arm.ARM, Constants.CANIVORE_NAME);
        arm = new TalonFX(Arm.ARM, Constants.CANIVORE_NAME);
        armEncoder = new CANcoder(Arm.ARM_CANCODER,Constants.CANIVORE_NAME);
        armPidController = arm.getPIDController();
        beamBreak = new AnalogInput(constants.sensorId());

        configureDevices();
    }
    @Override
  public void setArmOutput(double percent) {
    arm.set(percent);
  }

  @Override
  public void setArmVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  @Override
  public void setArmPosition(double positionRadians, double ffVolts) {
    // armPidController.setReference(
    //     Units.radiansToDegrees(positionRadians) * armGearRatio,
    //     ControlType.kSmartMotion,
    //     0,
    //     ffVolts,
    //     ArbFFUnits.kVoltage);

    armPidController.setReference(
        Units.radiansToRotations(positionRadians) * armGearRatio, ControlType.kPosition);
  }
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armPositionRad = Units.rotationsToRadians(armEncoder.getPosition()) / armGearRatio;
    inputs.armVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity()) / armGearRatio;

    inputs.armAppliedVolts = arm.getAppliedOutput() * arm.getBusVoltage();
    inputs.armCurrentAmps = arm.getOutputCurrent();
  }

  @Override
  public void stopArm() {
    arm.stopMotor();
  }

  @Override
  public void configureAimerPID(double kP, double kI, double kD) {
    armPidController.setP(kP, 0);
    armPidController.setI(kI, 0);
    armPidController.setD(kD, 0);
    armPidController.setFF(0, 0);
  }

  @Override
  public void configureLimits(double forwardLimit, double backwardLimit) {
    // arm.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
    // arm.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
    // arm.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) forwardLimit);
    // arm.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) backwardLimit);
  }

  private void configureDevices() {
    sparkBurnManager.maybeBurnConfig(
        () -> {
          SparkConfig.configNotLeader(arm);
}
