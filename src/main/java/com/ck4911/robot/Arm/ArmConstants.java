package com.ck4911.robot.Arm;

import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record ArmConstants(
    int armMotorId, 
    int sensorId,
    double armGearRatio,
    double armForwardLimit,
    double armBackwardLimit,
    double armTime,
    double collectPositionDegrees,
    double troughPositionDegrees,
    double levelTwoAndThreePositionDegrees,
    double levelFourPositionDegrees,
    PidValues armFeedBackValues,
    FeedForwardValues armFeedForwardValues
    ) {}
