package com.team1701.lib.drivers.leds;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDController {
    private static final int kMaxBrightness = 128;

    private final AddressableLED mLED;
    private final AddressableLEDBuffer mLEDBuffer;
    private final Color[] mColors;
    private final int[] mBrightness;

    public LEDController(int port, int length) {
        mLED = new AddressableLED(port);
        mLEDBuffer = new AddressableLEDBuffer(length);
        mLED.setLength(mLEDBuffer.getLength());
        mLED.start();
        mColors = new Color[length];
        mBrightness = new int[length];
        Arrays.fill(mColors, Color.kBlack);
        Arrays.fill(mBrightness, kMaxBrightness);
    }

    public void set(int index, Color color) {
        mColors[index] = color;
        mBrightness[index] = kMaxBrightness;
    }

    public void set(int index, Color color, double brightness) {
        mColors[index] = color;
        mBrightness[index] = toBrightnessValue(brightness);
    }

    public void setAll(Color state) {
        setRange(0, mColors.length, state);
    }

    public void setAll(Color state, double brightness) {
        setRange(0, mColors.length, state, brightness);
    }

    public void setRange(int start, int end, Color color) {
        Arrays.fill(mColors, start, end, color);
        Arrays.fill(mBrightness, start, end, kMaxBrightness);
    }

    public void setRange(int start, int end, Color color, double brightness) {
        Arrays.fill(mColors, start, end, color);
        Arrays.fill(mBrightness, start, end, toBrightnessValue(brightness));
    }

    private int toBrightnessValue(double brightness) {
        return MathUtil.clamp((int) (brightness * kMaxBrightness), 0, kMaxBrightness);
    }

    private void setRBGFromGradient(int index, double red, double green, double blue) {
        var brightness = mBrightness[index];
        mLEDBuffer.setRGB(index, (int) (red * brightness), (int) (blue * brightness), (int) (green * brightness));
    }

    public void update() {
        for (var i = 0; i < mColors.length; i++) {
            var color = mColors[i];
            setRBGFromGradient(i, color.red, color.green, color.blue);
        }

        mLED.setData(mLEDBuffer);
    }

    public String[] getColorHexStrings() {
        var hexStrings = new String[mColors.length];
        for (var i = 0; i < mColors.length; i++) {
            hexStrings[i] = String.format(
                    "#%02X%02X%02X",
                    (int) (mColors[i].red * mBrightness[i]), (int) (mColors[i].green * mBrightness[i]), (int)
                            (mColors[i].blue * mBrightness[i]));
        }

        return hexStrings;
    }
}
