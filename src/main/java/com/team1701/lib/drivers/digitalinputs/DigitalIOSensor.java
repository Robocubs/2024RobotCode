package com.team1701.lib.drivers.digitalinputs;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalIOSensor implements DigitalIO {
    private final DigitalInput mSensor;
    private boolean mInverted;

    public DigitalIOSensor(int channel, boolean inverted) {
        mInverted = inverted;
        mSensor = new DigitalInput(channel);
    }

    @Override
    public void updateInputs(DigitalInputs inputs) {
        inputs.blocked = mInverted ? !mSensor.get() : mSensor.get();
    }
}
