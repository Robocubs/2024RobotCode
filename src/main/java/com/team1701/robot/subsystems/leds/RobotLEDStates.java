package com.team1701.robot.subsystems.leds;

import com.team1701.lib.drivers.leds.LEDController.LEDPattern;
import com.team1701.lib.drivers.leds.LEDController.LEDState;
import edu.wpi.first.wpilibj.util.Color;

public class RobotLEDStates {
    // TODO:
    //  3 stage progress for heading, speed, and wrist angle
    //  Distance gradient (yellow -> blue)
    public static final LEDState kDisabledRed = new LEDState(Color.kRed, LEDPattern.STATIC);
    public static final LEDState kDisabledBlue = new LEDState(Color.kBlue, LEDPattern.STATIC);
    public static final LEDState kIdleHasNote = new LEDState(Color.kBlue);
    public static final LEDState kIdleNoNote = new LEDState(Color.kRed);
    public static final LEDState kSeesNote = new LEDState(Color.kOrangeRed);
    public static final LEDState kScoring = new LEDState(Color.kGreen);
    public static final LEDState kClimbing = new LEDState(Color.kWhite);
}
