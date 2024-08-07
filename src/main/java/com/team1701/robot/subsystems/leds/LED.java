package com.team1701.robot.subsystems.leds;

import com.team1701.lib.drivers.leds.LEDController;
import com.team1701.robot.Configuration;
import com.team1701.robot.states.RobotState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
    private static final double kMaxRange = 4.0;
    private static final int kTopLEDCount = 27;
    private static final int kTopLEDsRowCount = 3;
    private static final int kTopLEDsPerRow = kTopLEDCount / kTopLEDsRowCount;
    private static final int kCylonFrequency = 16;
    private static final int kStrobeFrequency = 10;

    private final LEDController mLEDController;
    private final RobotState mRobotState;

    public LED(RobotState robotState) {
        mLEDController = new LEDController(0, kTopLEDCount);
        mRobotState = robotState;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setDisabledLEDStates();
        } else if (mRobotState.hasNote()) {
            if (mRobotState.getShootingState().isActive) {
                setScoringLEDStates();
            } else if (!mRobotState.hasLoadedNote()) {
                setStrobe(LEDColors.kIdleHasNote);
            } else if (mRobotState.isSpeakerMode() && !mRobotState.inOpponentWing()) {
                setSpeakerRangeLEDStates();
            } else {
                mLEDController.setAll(LEDColors.kIdleHasNote);
            }
        } else {
            if (mRobotState.getDetectedNoteForPickup().isPresent()) {
                mLEDController.setAll(LEDColors.kSeesNote);
            } else {
                mLEDController.setAll(LEDColors.kIdleNoNote);
            }
        }
        mLEDController.update();
    }

    @Override
    public void simulationPeriodic() {
        Logger.recordOutput("LED", mLEDController.getColorHexStrings());
    }

    private void setStrobe(Color color) {
        var tick = (int) (Timer.getFPGATimestamp() * kStrobeFrequency * 2);
        mLEDController.setAll(tick % 2 == 0 ? color : Color.kWhite);
    }

    private void setRainbow() {
        var tick = (int) (Timer.getFPGATimestamp() * 60);
        var hue = tick % 180;
        // for (var row = 0; row < kTopLEDsRowCount; row++) {
        //     var start = row * kTopLEDsPerRow;
        //     mLEDController.setRange(start, start + 3, Color.fromHSV(hue, 255, 128));
        // }

        for (var row = 0; row < kTopLEDsRowCount; row++) {
            if (row % 2 == 0) {
                for (var i = 0; i < kTopLEDsPerRow; i++) {
                    mLEDController.set(i + row * kTopLEDsPerRow, Color.fromHSV((hue + i * 10) % 180, 255, 128));
                }
            } else {
                for (var i = 0; i < kTopLEDsPerRow; i++) {
                    mLEDController.set(
                            kTopLEDsPerRow - i - 1 + row * kTopLEDsPerRow,
                            Color.fromHSV((hue + i * 10) % 180, 255, 128));
                }
            }
        }
    }

    private void setDisabledLEDStates() {
        var color = Configuration.isBlueAlliance() ? LEDColors.kDisabledBlue : LEDColors.kDisabledRed;
        var cylonColumn = (int) ((Timer.getFPGATimestamp() * kCylonFrequency) % ((kTopLEDsPerRow - 1) * 2));
        if (cylonColumn >= kTopLEDsPerRow) {
            cylonColumn = (kTopLEDsPerRow - 1) * 2 - cylonColumn;
        }

        mLEDController.setAll(Color.kBlack);
        for (var row = 0; row < kTopLEDsRowCount; row++) {
            var rowStart = row * kTopLEDsPerRow;
            var rowEnd = rowStart + kTopLEDsPerRow - 1;
            var column = row % 2 == 0 ? rowStart + cylonColumn : rowEnd - cylonColumn;
            mLEDController.set(column, color);
            if (column > rowStart) {
                mLEDController.set(column - 1, color, 0.25);
            }
            if (column < rowEnd) {
                mLEDController.set(column + 1, color, 0.25);
            }
        }
    }

    // private void setDisabledLEDStates() {
    //     setRainbow();
    // }

    private void setScoringLEDStates() {
        var state = mRobotState.getShootingState();
        if (state.isShooting) {
            mLEDController.setAll(LEDColors.kShooting);
            return;
        }

        var tick = (int) (Timer.getFPGATimestamp() * kStrobeFrequency * 2);
        var validColor = state.canShootWhileMove && tick % 2 == 1 ? Color.kWhite : LEDColors.kShootingValid;
        var atAngleState = state.atAngle ? validColor : LEDColors.kShootingInvalid;
        var atSpeedState = state.atSpeed ? validColor : LEDColors.kShootingInvalid;
        var atHeadingState = state.atHeading ? validColor : LEDColors.kShootingInvalid;

        var ledsPerSegment = kTopLEDsPerRow / 3;
        for (var row = 0; row < kTopLEDsRowCount; row++) {
            var startCol0 = row * kTopLEDsPerRow;
            var startCol1 = startCol0 + ledsPerSegment;
            var startCol2 = startCol1 + ledsPerSegment;
            mLEDController.setRange(startCol0, startCol1, row % 2 == 0 ? atAngleState : atSpeedState);
            mLEDController.setRange(startCol1, startCol2, atHeadingState);
            mLEDController.setRange(startCol2, startCol2 + ledsPerSegment, row % 2 == 0 ? atSpeedState : atAngleState);
        }
    }

    private void setSpeakerRangeLEDStates() {
        var unlitLEDsPerSide =
                MathUtil.clamp((int) (mRobotState.getDistanceToSpeaker() - kMaxRange), 0, kTopLEDsPerRow / 2);
        var litLEDs = kTopLEDsPerRow - 2 * unlitLEDsPerSide;
        mLEDController.setRange(0, kTopLEDCount, LEDColors.kIdleHasNoteSpeakerFar);
        for (var row = 0; row < kTopLEDsRowCount; row++) {
            var start = unlitLEDsPerSide + row * kTopLEDsPerRow;
            mLEDController.setRange(start, start + litLEDs, LEDColors.kIdleHasNote);
        }
    }
}
