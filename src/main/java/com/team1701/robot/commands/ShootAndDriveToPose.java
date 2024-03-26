package com.team1701.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.tuning.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

public class ShootAndDriveToPose extends Command {
    private static final String kLoggingPrefix = "Command/RotateToSpeakerAndDriveToPose/";
    private static final TrapezoidProfile.State kZeroState = new TrapezoidProfile.State();

    private static final LoggedTunableNumber kTranslationKp =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationKp", 6.0);
    private static final LoggedTunableNumber kTranslationKi =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationKi", 0.0);
    private static final LoggedTunableNumber kTranslationKd =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationKd", 0.0);

    private final Drive mDrive;
    private final RobotState mRobotState;
    private final Command mShootAndMoveCommand;
    private final Supplier<Pose2d> mTargetPoseSupplier;
    private final PIDController mTranslationController;
    private final TrapezoidProfile mTranslationProfile;
    private final FinishedState mFinishedState;

    private TrapezoidProfile.State mTranslationState = kZeroState;
    private boolean mAtTranslation = false;

    public ShootAndDriveToPose(
            Drive drive,
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            Supplier<Pose2d> targetPoseSupplier,
            FinishedState finishedState) {
        mDrive = drive;
        mRobotState = robotState;
        mTargetPoseSupplier = targetPoseSupplier;
        mFinishedState = finishedState;

        mTranslationController = new PIDController(
                kTranslationKp.get(), kTranslationKi.get(), kTranslationKd.get(), Constants.kLoopPeriodSeconds);
        mTranslationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Constants.Drive.kSlowTrapezoidalKinematicLimits.maxDriveVelocity(),
                Constants.Drive.kSlowTrapezoidalKinematicLimits.maxDriveAcceleration()));

        mShootAndMoveCommand = new ShootAndMove(drive, shooter, indexer, robotState, this::getSpeed);
        CommandScheduler.getInstance().registerComposedCommands(mShootAndMoveCommand);
    }

    @Override
    public void initialize() {
        var translationToTarget = mTargetPoseSupplier
                .get()
                .getTranslation()
                .minus(mRobotState.getPose2d().getTranslation());

        mTranslationController.reset();
        mAtTranslation = false;

        mShootAndMoveCommand.initialize();
        mDrive.setKinematicLimits(Constants.Drive.kFastKinematicLimits);

        mTranslationState = new TrapezoidProfile.State(translationToTarget.getNorm(), 0);

        var hash = hashCode();
        if (kTranslationKp.hasChanged(hash) || kTranslationKi.hasChanged(hash) || kTranslationKd.hasChanged(hash)) {
            mTranslationController.setPID(kTranslationKp.get(), kTranslationKi.get(), kTranslationKd.get());
        }
    }

    @Override
    public void execute() {
        mShootAndMoveCommand.execute();
        Logger.recordOutput(kLoggingPrefix + "TranslationError", mTranslationController.getPositionError());
    }

    @Override
    public void end(boolean interrupted) {
        mShootAndMoveCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return switch (mFinishedState) {
            case NONE -> false;
            case END_AFTER_SHOOTING_AND_MOVING -> mShootAndMoveCommand.isFinished() && mAtTranslation;
            case END_AFTER_SHOOTING -> mShootAndMoveCommand.isFinished();
            case END_AFTER_MOVING -> mAtTranslation;
        };
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return mShootAndMoveCommand.getRequirements();
    }

    private Translation2d getSpeed() {
        var currentPose = mRobotState.getPose2d();
        Translation2d velocity;

        mAtTranslation =
                GeometryUtil.isNear(mTargetPoseSupplier.get().getTranslation(), currentPose.getTranslation(), .01);

        if (mAtTranslation) {
            velocity = new Translation2d();
        } else {
            var translationToTarget = mTargetPoseSupplier
                    .get()
                    .getTranslation()
                    .minus(mRobotState.getPose2d().getTranslation());
            var distanceToTarget = translationToTarget.getNorm();
            var headingToTarget = translationToTarget.getAngle();

            var translationPidOutput = mTranslationController.calculate(distanceToTarget, mTranslationState.position);
            mTranslationState =
                    mTranslationProfile.calculate(Constants.kLoopPeriodSeconds, mTranslationState, kZeroState);
            velocity = new Translation2d(-(mTranslationState.velocity + translationPidOutput), headingToTarget);
        }

        Logger.recordOutput(kLoggingPrefix + "TranslationError", mTranslationController.getPositionError());
        Logger.recordOutput(kLoggingPrefix + "TargetPose", mTargetPoseSupplier.get());

        return velocity;
    }

    public static enum FinishedState {
        NONE,
        END_AFTER_SHOOTING_AND_MOVING,
        END_AFTER_SHOOTING,
        END_AFTER_MOVING
    }
}
