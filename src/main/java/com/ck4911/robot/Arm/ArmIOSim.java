package com.ck4911.robot.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class ArmIOSim implements ArmIO {
    private final DCMotorSim sim;
    private final PIDController pid;

    private double appliedVolts = 0.0;

    public ArmIOSIm(ArmConstants constants) {
        //TODO: determine these thingies
        sim = new DCMotorSim(null, null, null);
        pid = new PIDController(0.0, 0.0, 0.0);
    }

    @Override
    public void stopArm() {
        setArmVoltage(appliedVolts);
    }

    @Override
  public void configureArmPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

}
