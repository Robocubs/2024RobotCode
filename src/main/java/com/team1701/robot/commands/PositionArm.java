package com.team1701.robot.commands;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.Constants.ScoringMode;
import com.team1701.robot.subsystems.arm.Arm;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PositionArm extends Command {

    public static final String kLoggingPrefix = "Command/PositionArm";

    private static final LoggedTunableNumber kArmAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "ArmAngleToleranceRadians", 0.01);

    private final Arm mArm;
    private final Rotation2d mTargetRotation;
    private final boolean mEndAtPosition;

    public PositionArm(Arm arm, boolean endAtPosition, ScoringMode scoringMode) {
        mArm = arm;
        mEndAtPosition = endAtPosition;

        switch (scoringMode) {
            case SPEAKER:
                mTargetRotation = ArmPosition.HOME.armRotation;
                break;
            case AMP:
                mTargetRotation = ArmPosition.AMP.armRotation;
                break;
            case CLIMB:
                mTargetRotation = ArmPosition.LOW_CLIMB.armRotation;
                break;
            default:
                mTargetRotation = ArmPosition.HOME.armRotation;
                break;
        }

        addRequirements(arm);
    }

    @Override
    public void execute() {
        mArm.setRotationAngle(mTargetRotation);
    }

    @Override
    public boolean isFinished() {
        return mEndAtPosition
                ? GeometryUtil.isNear(
                        mArm.getAngle(),
                        mTargetRotation,
                        Rotation2d.fromRadians(kArmAngleToleranceRadians.get()))
                : true;
    }

    public enum ArmPosition {
        // TODO: update values
        HOME(0),
        LOW(45),
        LOW_CLIMB(75),
        HIGH_CLIMB(90),
        AMP(Constants.Arm.kArmAmpRotationDegrees.get());

        public Rotation2d armRotation;

        private ArmPosition(double degrees) {
            armRotation = Rotation2d.fromDegrees(degrees);
        }
    }
}
