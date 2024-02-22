package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants.ScoringMode;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.arm.Arm;
import com.team1701.robot.subsystems.arm.Arm.ArmPosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PositionArm extends Command {

    public static final String kLoggingPrefix = "Command/PositionArm";

    private static final LoggedTunableNumber kArmAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "ArmAngleToleranceRadians", 0.01);

    private final Arm mArm;
    private final boolean mEndAtPosition;
    private final boolean mIdle;
    private Supplier<ScoringMode> mScoringMode;
    private Rotation2d mTargetRotation;
    private RobotState mRobotState;

    public PositionArm(
            Arm arm, boolean endAtPosition, boolean idle, RobotState robotState, Supplier<ScoringMode> scoringMode) {
        mArm = arm;
        mEndAtPosition = endAtPosition;
        mIdle = idle;
        mScoringMode = scoringMode;
        mRobotState = robotState;

        addRequirements(arm);
    }

    @Override
    public void execute() {

        switch (mScoringMode.get()) {
            case SPEAKER:
                mTargetRotation = ArmPosition.HOME.armRotation;
                break;
            case AMP:
                mTargetRotation = (!mIdle || (mRobotState.getDistanceToAmp() <= 1))
                        ? ArmPosition.AMP.armRotation
                        : ArmPosition.HOME.armRotation;
                break;
            case CLIMB:
                mTargetRotation = mIdle ? ArmPosition.HOME.armRotation : ArmPosition.LOW_CLIMB.armRotation;
                break;
            default:
                mTargetRotation = ArmPosition.HOME.armRotation;
                break;
        }
        mArm.setRotationAngle(mTargetRotation);
    }

    @Override
    public boolean isFinished() {
        return mEndAtPosition
                ? GeometryUtil.isNear(
                        mArm.getAngle(), mTargetRotation, Rotation2d.fromRadians(kArmAngleToleranceRadians.get()))
                : false;
    }
}
