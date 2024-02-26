package com.team1701.robot.subsystems.leds;

import com.team1701.lib.drivers.leds.LEDController;
import com.team1701.lib.drivers.leds.LEDState;
import com.team1701.lib.drivers.leds.RobotLEDStates;
import com.team1701.lib.drivers.leds.RobotLEDStates.LEDPattern;
import com.team1701.robot.Configuration;
import com.team1701.robot.states.RobotState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private LEDController mLEDController;
    private LEDState mDisabledRobotState = new LEDState(Color.kRed, LEDPattern.STATIC);
    private RobotState mRobotState;

    public LED(RobotState robotState) {
        mLEDController = new LEDController();
        mRobotState = robotState;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            mDisabledRobotState.color = Configuration.isBlueAlliance() ? Color.kBlue : Color.kRed;
            mLEDController.setCurrLEDState(mDisabledRobotState);
        } else {
            if (mRobotState.hasNote()) {
                if (mRobotState.getIsScoring()) {
                    mLEDController.setCurrLEDState(RobotLEDStates.mRobotScoring);
                } else {
                    mLEDController.setCurrLEDState(RobotLEDStates.mRobotIdleHasNote);
                }
            } else {
                if (mRobotState.getDetectedNotePoses2d().length > 0) {
                    mLEDController.setCurrLEDState(RobotLEDStates.mRobotSeesNote);
                } else {
                    mLEDController.setCurrLEDState(RobotLEDStates.mRobotIdleNoNote);
                }
            }
        }
        mLEDController.update();
    }
}
