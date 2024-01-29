package com.team1701.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        boolean inputSensor;
        boolean outputSensor;
    }

    default void updateInputs(IntakeInputs inputs) {}
}
