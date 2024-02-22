package com.team1701.robot.commands;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.subsystems.arm.Arm;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PositionClimb extends Command {

    public static final String kLoggingPrefix = "Command/PositionClimb";

    private static final LoggedTunableNumber kArmAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "ArmAngleToleranceRadians", 0.01);

    private final Arm mArm;
    private final ArmPosition mPosition;
    private final boolean mEndAtPosition;

    public PositionClimb(Arm arm, ArmPosition position, boolean endAtPosition) {
        mArm = arm;
        mPosition = position;
        mEndAtPosition = endAtPosition;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        mArm.setRotationAngle(mPosition.armRotation);
    }

    @Override
    public boolean isFinished() {
        return mEndAtPosition
                ? GeometryUtil.isNear(
                        mArm.getAngle(), mPosition.armRotation, Rotation2d.fromRadians(kArmAngleToleranceRadians.get()))
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
