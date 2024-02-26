package com.team1701.lib.drivers.leds;

import com.team1701.lib.drivers.leds.RobotLEDStates.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDController {
    private final AddressableLED mLED;
    private final AddressableLEDBuffer mLEDBuffer;

    private LEDState mLEDState;

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
        int timestamp = (int) Timer.getFPGATimestamp();
        for (var i = 0; i < mLEDBuffer.getLength(); i++) {
            if (mLEDState.pattern == LEDPattern.BLINK) {
                if (timestamp % 2 == 0) {
                    mLEDBuffer.setLED(i, Color.kBlack);
                } else {
                    setRBGfromRGBGradient(i, mLEDState.color.red, mLEDState.color.blue, mLEDState.color.green);
                }
            } else if (mLEDState.pattern == LEDPattern.STATIC) {
                setRBGfromRGBGradient(i, mLEDState.color.red, mLEDState.color.blue, mLEDState.color.green);
            }
        }

        mLED.setData(mLEDBuffer);
    }
}
