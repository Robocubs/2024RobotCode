package com.team1701.robot.subsystems.leds;

import com.team1701.lib.drivers.leds.LEDController;
import com.team1701.robot.Configuration;
import com.team1701.robot.states.RobotState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final LEDController mLEDController;
    private final RobotState mRobotState;

    public LED(RobotState robotState) {
        mLEDController = new LEDController(0, 27);
        mRobotState = robotState;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            var state = Configuration.isBlueAlliance() ? RobotLEDStates.kDisabledBlue : RobotLEDStates.kDisabledRed;
            mLEDController.setCurrLEDState(state);
        } else if (mRobotState.hasNote()) {
            if (mRobotState.isScoring()) {
                mLEDController.setCurrLEDState(RobotLEDStates.kScoring);
            } else {
                mLEDController.setCurrLEDState(RobotLEDStates.kIdleHasNote);
            }
        } else {
            if (mRobotState.getDetectedNotePoses3d().length > 0) {
                mLEDController.setCurrLEDState(RobotLEDStates.kSeesNote);
            } else {
                mLEDController.setCurrLEDState(RobotLEDStates.kIdleNoNote);
            }
        }
        mLEDController.update();
    }
}
