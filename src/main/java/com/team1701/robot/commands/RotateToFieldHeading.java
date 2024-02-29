package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
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
    private static final String kLoggingPrefix = "Command/RotateToFieldHeading/";
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
            new LoggedTunableNumber(kLoggingPrefix + "RotationToleranceRadians", 0.009);

    private final Drive mDrive;
    private final Supplier<Rotation2d> mTargetHeadingSupplier;
    private final Supplier<Rotation2d> mRobotHeadingSupplier;
    private final KinematicLimits mKinematicLimits;
    private final boolean mFinishAtRotation;
    private final PIDController mRotationController;

    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mRotationState = new TrapezoidProfile.State();
    private boolean mAtTargetRotation = false;

    private DoubleSupplier mXSupplier;
    private DoubleSupplier mYSupplier;

    RotateToFieldHeading(
            Drive drive,
            Supplier<Rotation2d> targetHeadingSupplier,
            Supplier<Rotation2d> robotHeadingSupplier,
            KinematicLimits kinematicLimits,
            boolean finishAtRotation) {
        mDrive = drive;
        mTargetHeadingSupplier = targetHeadingSupplier;
        mRobotHeadingSupplier = robotHeadingSupplier;
        mKinematicLimits = kinematicLimits;
        mFinishAtRotation = finishAtRotation;

        mRotationController = new PIDController(
                kRotationKp.get(), kRotationKi.get(), kRotationKd.get(), Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(kMaxAngularVelocity.get(), kMaxAngularAcceleration.get()));

        mXSupplier = () -> 0;
        mYSupplier = () -> 0;

        addRequirements(drive);
    }

    RotateToFieldHeading(
            Drive drive,
            Supplier<Rotation2d> targetHeadingSupplier,
            Supplier<Rotation2d> robotHeadingSupplier,
            KinematicLimits kinematicLimits,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        this(drive, targetHeadingSupplier, robotHeadingSupplier, kinematicLimits, false);

        mXSupplier = xSupplier;
        mYSupplier = ySupplier;
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(Constants.Drive.kFastKinematicLimits);

        mRotationController.reset();
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mAtTargetRotation = false;

        var currentHeading = mRobotHeadingSupplier.get();
        var fieldRelativeChassisSpeeds = mDrive.getFieldRelativeVelocity();
        mRotationState = new TrapezoidProfile.State(
                MathUtil.inputModulus(
                        currentHeading.getRadians(),
                        mTargetHeadingSupplier.get().getRadians() - Math.PI,
                        mTargetHeadingSupplier.get().getRadians() + Math.PI),
                fieldRelativeChassisSpeeds.omegaRadiansPerSecond);
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

        var currentHeading = mRobotHeadingSupplier.get();
        var targetHeading = mTargetHeadingSupplier.get();
        Rotation2d setpoint;
        mAtTargetRotation = GeometryUtil.isNear(
                targetHeading, currentHeading, Rotation2d.fromRadians(kRotationToleranceRadians.get()));
        if (mAtTargetRotation
                && Util.epsilonEquals(mXSupplier.getAsDouble(), 0.0)
                && Util.epsilonEquals(mYSupplier.getAsDouble(), 0.0)) {
            mDrive.stop();
            setpoint = currentHeading;
        } else {

            // Calculate rotational velocity
            var rotationPidOutput = mRotationController.calculate(currentHeading.getRadians(), mRotationState.position);
            var rotationTargetState = new TrapezoidProfile.State(targetHeading.getRadians(), 0.0);
            mRotationState =
                    mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, rotationTargetState);
            var rotationalVelocity = mRotationState.velocity + rotationPidOutput;

            // Set drive outputs
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    mXSupplier.getAsDouble(), mYSupplier.getAsDouble(), rotationalVelocity, currentHeading));
            setpoint = Rotation2d.fromRadians(mRotationState.position);
        }

        Logger.recordOutput(
                kLoggingPrefix + "RotationError", Rotation2d.fromRadians(mRotationController.getPositionError()));
        Logger.recordOutput(kLoggingPrefix + "Setpoint", setpoint);
        Logger.recordOutput(kLoggingPrefix + "TargetHeading", targetHeading);
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return mFinishAtRotation && mAtTargetRotation == false;
    }
}
