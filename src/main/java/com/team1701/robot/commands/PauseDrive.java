package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

public class PauseDrive extends Command {
    private static final String kLoggingPrefix = "Command/PauseDrive/";
    private final Drive mDrive;
    private final RobotState mRobotState;
    private final Supplier<Rotation2d> mTargetModuleHeadingSupplier;
    private final Command mRotationCommand;

    private Rotation2d mTargetModuleHeading = GeometryUtil.kRotationIdentity;

    public PauseDrive(
            Drive drive,
            RobotState robotState,
            Supplier<Rotation2d> targetModuleHeadingSupplier,
            Supplier<Rotation2d> targetRobotHeadingSupplier) {
        mDrive = drive;
        mRobotState = robotState;
        mTargetModuleHeadingSupplier = targetModuleHeadingSupplier;
        mRotationCommand = new RotateToFieldHeading(
                drive,
                targetRobotHeadingSupplier::get,
                mRobotState::getHeading,
                mRobotState::getSpeakerHeadingTolerance,
                Constants.Drive.kFastTrapezoidalKinematicLimits,
                false);

        CommandScheduler.getInstance().registerComposedCommands(mRotationCommand);
        addRequirements(mRotationCommand.getRequirements().toArray(Subsystem[]::new));
    }

    @Override
    public void initialize() {
        mRotationCommand.initialize();
        mTargetModuleHeading = mTargetModuleHeadingSupplier.get();
        Logger.recordOutput(kLoggingPrefix + "TargetModuleHeading", mTargetModuleHeading);
    }

    @Override
    public void execute() {
        var velocity = mDrive.getVelocity();
        if (Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond) < 0.5) {
            mDrive.orientModules(mTargetModuleHeading.minus(mRobotState.getHeading()));
        } else {
            mRotationCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(kLoggingPrefix + "TargetModuleHeading", GeometryUtil.kRotationIdentity);
    }

    @Override
    public boolean isFinished() {
        var velocity = mDrive.getVelocity();
        return MathUtil.isNear(0, Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond), 0.1);
    }
}
