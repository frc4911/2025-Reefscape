package com.ck4911.robot.Arm;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Arm extends SubsystemBase {
    
    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmConstants constants, ArmIO armIO) {
        super();
        this.armIO = armIO;
        armKp.initDefault(constants.armFeedBackValues().kP());
        armKd.initDefault(constants.armFeedBackValues().kD());
        armTime.initDefault(constants.armTime());
        forwardLimit.initDefault(constants.armForwardLimit());
        backwardLimit.initDefault(constants.armBackwardLimit());

        feedforward = new SimpleMotorFeedforward(armKs.get(), armKv.get());
        armIO.configureAimerPID(armKp.get(), 0.0, armKd.get());

        
    }
}
