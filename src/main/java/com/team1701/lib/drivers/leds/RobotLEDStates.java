package com.team1701.lib.drivers.leds;

import edu.wpi.first.wpilibj.util.Color;

public class RobotLEDStates {
    // Disabled color is set in LED subsystem
    // TODO:
    //  3 stage progress for heading, speed, and wrist angle
    //  Distance gradient (yellow -> blue)
    public static LEDState mRobotIdleHasNote = new LEDState(Color.kGreen, LEDPattern.STATIC);
    public static LEDState mRobotIdleNoNote = new LEDState(Color.kRed, LEDPattern.BLINK);
    public static LEDState mRobotSeesNote = new LEDState(Color.kOrange, LEDPattern.BLINK);
    public static LEDState mRobotScoring = new LEDState(Color.kGreen, LEDPattern.BLINK);
    public static LEDState mRobotClimbing = new LEDState(Color.kWhite, LEDPattern.BLINK);

    public enum LEDPattern {
        STATIC,
        BLINK
    }
}
