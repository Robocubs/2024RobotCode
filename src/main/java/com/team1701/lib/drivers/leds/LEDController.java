package com.team1701.lib.drivers.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDController {
    private final AddressableLED mLED;
    private final AddressableLEDBuffer mLEDBuffer;

    private LEDState mLEDState;

    private double mLastTimestamp = 0.0;

    public LEDController() {
        mLED = new AddressableLED(0);
        mLEDBuffer = new AddressableLEDBuffer(27);
        mLED.setLength(mLEDBuffer.getLength());
        mLED.start();
    }

    public void setCurrLEDState(LEDState state) {
        mLEDState = state;
    }

    private int percentageColorToStandard(double p) {
        return (int) (p * 255);
    }

    private void setRBGfromRGBGradient(int i, double r, double g, double b) {
        mLEDBuffer.setRGB(i, percentageColorToStandard(r), percentageColorToStandard(b), percentageColorToStandard(g));
    }

    public void update() {
        var time = Timer.getFPGATimestamp();

        switch (mLEDState.pattern) {
            case BLINK:
                if (time - mLastTimestamp > 1) {
                    for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                        mLEDBuffer.setLED(i, Color.kBlack);
                    }
                    mLastTimestamp = time;
                } else {
                    for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                        setRBGfromRGBGradient(i, mLEDState.color.red, mLEDState.color.blue, mLEDState.color.green);
                    }
                }
                break;
            case STATIC:
                for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                    setRBGfromRGBGradient(i, mLEDState.color.red, mLEDState.color.blue, mLEDState.color.green);
                }
                break;
            default:
                break;
        }

        mLED.setData(mLEDBuffer);
    }
}
