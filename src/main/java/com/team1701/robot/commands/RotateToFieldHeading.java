package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/* Note: positive rotations are counterclockwise (turning left) */

public class RotateToFieldHeading extends Command {
    private RobotState mRobotState = new RobotState();
    private static final String kLoggingPrefix = "Command/RotateRelativeToRobot/";
    private static final double kModuleRadius = Constants.Drive.kModuleRadius;
    private static final KinematicLimits kMaxKinematicLimits = Constants.Drive.kFastTrapezoidalKinematicLimits;
    private static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularVelocity", kMaxKinematicLimits.maxDriveVelocity() / kModuleRadius);
    private static final LoggedTunableNumber kMaxAngularAcceleration =
            new LoggedTunableNumber(kLoggingPrefix + "MaxAngularAcceleration", kMaxAngularVelocity.get() / 2.0);
    // TODO: Update PID values
    private static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber(kLoggingPrefix + "RotationKp", 4.0);
    private static final LoggedTunableNumber kRotationKi = new LoggedTunableNumber(kLoggingPrefix + "RotationKi", 0.0);
    private static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber(kLoggingPrefix + "RotationKd", 0.0);
    private static final LoggedTunableNumber kRotationToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "RotationToleranceRadians", 0.01);

    private final Drive mDrive;
    private Supplier<Rotation2d> mTargetFieldHeading;
    private final KinematicLimits mKinematicLimits;
    private final boolean mFinishAtRotation;
    private final PIDController mRotationController;

    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mRotationState = new TrapezoidProfile.State();

    RotateToFieldHeading(
            Drive drive,
            Supplier<Rotation2d> targetHeading,
            KinematicLimits kinematicLimits,
            boolean finishAtRotation) {
        mDrive = drive;
        mTargetFieldHeading = targetHeading;
        mKinematicLimits = kinematicLimits;
        mFinishAtRotation = finishAtRotation;

        mRotationController = new PIDController(
                kRotationKp.get(), kRotationKi.get(), kRotationKd.get(), Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(kMaxAngularVelocity.get(), kMaxAngularAcceleration.get()));

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(Constants.Drive.kFastKinematicLimits);

        Logger.recordOutput(kLoggingPrefix + "requestedRotation/fieldRelative", mTargetFieldHeading.get());

        mRotationController.reset();

        var currentHeading = mRobotState.getHeading();
        var fieldRelativeChassisSpeeds = mDrive.getFieldRelativeVelocity();
        mRotationState = new TrapezoidProfile.State(
                MathUtil.inputModulus(
                        currentHeading.getRadians(),
                        mTargetFieldHeading.get().getRadians() - Math.PI,
                        mTargetFieldHeading.get().getRadians() + Math.PI),
                fieldRelativeChassisSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput(kLoggingPrefix + "SuccessfullyInitialized", true);
    }

    @Override
    public void execute() {
        var hash = hashCode();
        if (kMaxAngularVelocity.hasChanged(hash)
                || kMaxAngularAcceleration.hasChanged(hash)
                || kRotationKp.hasChanged(hash)
                || kRotationKi.hasChanged(hash)
                || kRotationKd.hasChanged(hash)) {
            mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    Math.min(kMaxAngularVelocity.get(), mKinematicLimits.maxDriveVelocity() / kModuleRadius),
                    Math.min(kMaxAngularAcceleration.get(), mKinematicLimits.maxDriveAcceleration() / kModuleRadius)));
            mRotationController.setPID(kRotationKp.get(), kRotationKi.get(), kRotationKd.get());
        }

        var currentHeading = mRobotState.getPose2d().getRotation();

        // Calculate rotational velocity
        var rotationPidOutput = mRotationController.calculate(currentHeading.getRadians(), mRotationState.position);
        var rotationTargetState =
                new TrapezoidProfile.State(mTargetFieldHeading.get().getRadians(), 0.0);
        mRotationState = mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, rotationTargetState);
        var rotationalVelocity = mRotationState.velocity + rotationPidOutput;

        // Set drive outputs
        var atTargetRotation = GeometryUtil.isNear(
                mTargetFieldHeading.get(), currentHeading, Rotation2d.fromRadians(kRotationToleranceRadians.get()));
        if (atTargetRotation) {
            mDrive.stop();
            currentHeading = mRobotState.getPose2d().getRotation();
        } else {
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotationalVelocity, currentHeading));
            currentHeading = Rotation2d.fromRadians(mRotationState.position);
        }

        Logger.recordOutput(
                kLoggingPrefix + "RotationError", Rotation2d.fromRadians(mRotationController.getPositionError()));
        Logger.recordOutput(kLoggingPrefix + "ActualHeading", currentHeading);
        Logger.recordOutput(kLoggingPrefix + "TargetHeading", mTargetFieldHeading.get());
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return mFinishAtRotation && atTargetPose();
    }

    public boolean atTargetPose() {
        var currentRotation = mRobotState.getHeading();
        var rotationError = mTargetFieldHeading.get().minus(currentRotation);

        return Util.inRange(rotationError.getRadians(), kRotationToleranceRadians.get())
                && GeometryUtil.isNear(
                        mTargetFieldHeading.get(),
                        currentRotation,
                        Rotation2d.fromRadians(kRotationToleranceRadians.get()));
    }
}
