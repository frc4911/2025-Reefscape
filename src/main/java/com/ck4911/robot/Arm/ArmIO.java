package com.ck4911.robot.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double armPositionRad;
        public double armVelocityRadPerSec; 
        public double armAppliedVolts;
        public double armCurrentAmps;
    }

     /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmOutput(double percent) {}

  /** Run the arm at the specified voltage. */
  public default void setArmVoltage(double volts) {}

   /** Run closed loop to the specified position. */
   public default void setArmPosition(double position, double ffVolts) {}

    /** Stop aimer in open loop. */
  public default void stopArm() {}

    /** Set velocity PID constants. */
  public default void configureAimerPID(double kP, double kI, double kD) {}

  public default void configureLimits(double forwardLimit, double backwardLimit) {}

}
