package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PauseDrive extends Command {
    private static final String kLoggingPrefix = "Command/PauseDrive/";
    private final Drive mDrive;
    private final RobotState mRobotState;
    private Supplier<Pose2d> mTargetPoseSupplier;
    private Command mRotationCommand;

    public PauseDrive(Drive drive, RobotState robotState, Supplier<Pose2d> targetPoseSupplier) {
        mDrive = drive;
        mRobotState = robotState;
        mTargetPoseSupplier = targetPoseSupplier;
        mRotationCommand = new RotateToFieldHeading(
                drive,
                mTargetPoseSupplier.get()::getRotation,
                mRobotState::getHeading,
                mRobotState::getToleranceSpeakerHeading,
                Constants.Drive.kFastTrapezoidalKinematicLimits,
                false);

        CommandScheduler.getInstance().registerComposedCommands(mRotationCommand);
        addRequirements(mRotationCommand.getRequirements().toArray(Subsystem[]::new));
    }

    @Override
    public void initialize() {
        mRotationCommand.initialize();
    }

    @Override
    public void execute() {
        if (Math.hypot(
                        mDrive.getFieldRelativeVelocity().vxMetersPerSecond,
                        mDrive.getFieldRelativeVelocity().vyMetersPerSecond)
                < 0.5) {

            mDrive.orientModules(mTargetPoseSupplier
                    .get()
                    .getTranslation()
                    .minus(mRobotState.getPose2d().getTranslation())
                    .getAngle()
                    .rotateBy(mRobotState.getHeading()));
        } else {
            mRotationCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(
                0,
                Math.hypot(
                        mDrive.getFieldRelativeVelocity().vxMetersPerSecond,
                        mDrive.getFieldRelativeVelocity().vyMetersPerSecond),
                0.1);
    }
}
