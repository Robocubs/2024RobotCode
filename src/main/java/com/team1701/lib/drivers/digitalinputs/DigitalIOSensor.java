package com.team1701.lib.drivers.digitalinputs;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalIOSensor implements DigitalIO {
    private final DigitalInput mSensor;

    public DigitalIOSensor(int channel) {
        mSensor = new DigitalInput(channel);
    }

    @Override
    public void updateInputs(DigitalInputs inputs) {
        inputs.blocked = mSensor.get();
    }
}
