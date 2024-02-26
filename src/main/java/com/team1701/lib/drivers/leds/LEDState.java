package com.team1701.lib.drivers.leds;

import com.team1701.lib.drivers.leds.RobotLEDStates.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDState {
    public Color color;
    public LEDPattern pattern;

    public LEDState(Color color, LEDPattern pattern) {
        this.color = color;
        this.pattern = pattern;
    }
}
