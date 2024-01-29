package com.team1701.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOReal implements IntakeIO {
    private final DigitalInput mInputSensor;

    public IntakeIOReal(int inputSensorId, int outputSensorId) {
        mInputSensor = new DigitalInput(inputSensorId);
        // TODO: Add output sensor
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.inputSensor = mInputSensor.get();
        // TODO: Update output sensor
    }
}
