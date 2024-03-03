package com.team1701.lib.drivers.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.Logger;

public class LEDController {
    private final AddressableLED mLED;
    private final AddressableLEDBuffer mLEDBuffer;

    private LEDState mLEDState;

    private double mLastTimestamp = 0.0;

    public static record LEDState(Color color, LEDPattern pattern) {
        public LEDState(Color color) {
            this(color, LEDPattern.STATIC);
        }
    }

    public static enum LEDPattern {
        STATIC,
        BLINK
    }

    public LEDController(int port, int length) {
        mLED = new AddressableLED(port);
        mLEDBuffer = new AddressableLEDBuffer(length);
        mLED.setLength(mLEDBuffer.getLength());
        mLED.start();
    }

    public void setCurrLEDState(LEDState state) {
        mLEDState = state;
    }

    private int percentageColorToStandard(double p) {
        return (int) (p * 128);
    }

    private void setRBGfromRGBGradient(int i, double r, double g, double b) {
        mLEDBuffer.setRGB(i, percentageColorToStandard(r), percentageColorToStandard(b), percentageColorToStandard(g));
    }

    public void update() {
        var time = Timer.getFPGATimestamp();
        Logger.recordOutput("LEDs/mLEDState", mLEDState.toString());
        switch (mLEDState.pattern) {
            case BLINK:
                if (time - mLastTimestamp > 0.5) {
                    if (mLEDBuffer.getLED(0).equals(Color.kBlack)) {
                        for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                            setRBGfromRGBGradient(i, mLEDState.color.red, mLEDState.color.green, mLEDState.color.blue);
                        }
                    } else {
                        for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                            mLEDBuffer.setLED(i, Color.kBlack);
                        }
                    }
                    mLastTimestamp = time;
                }
                break;
            case STATIC:
                for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                    setRBGfromRGBGradient(i, mLEDState.color.red, mLEDState.color.green, mLEDState.color.blue);
                }
                break;
            default:
                break;
        }

        mLED.setData(mLEDBuffer);
    }
}
