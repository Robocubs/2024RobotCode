package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class RotateToFieldHeading extends Command {
    private static final String kLoggingPrefix = "Command/RotateToFieldHeading/";
    private static final double kModuleRadius = Constants.Drive.kModuleRadius;
    private static final TrapezoidProfile.State kZeroState = new TrapezoidProfile.State(0.0, 0.0);

    private static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularVelocity",
            Constants.Drive.kFastTrapezoidalKinematicLimits.maxDriveVelocity() / kModuleRadius);
    private static final LoggedTunableNumber kMaxAngularAcceleration = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularAcceleration",
            Constants.Drive.kFastTrapezoidalKinematicLimits.maxDriveAcceleration() / kModuleRadius);

    private static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber(kLoggingPrefix + "RotationKp", 15.0);
    private static final LoggedTunableNumber kRotationKi = new LoggedTunableNumber(kLoggingPrefix + "RotationKi", 0.0);
    private static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber(kLoggingPrefix + "RotationKd", 0.0);
    private static final LoggedTunableNumber kRotationToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "RotationToleranceRadians", 0.009);

    private final Drive mDrive;
    private final Supplier<Rotation2d> mTargetHeadingSupplier;
    private final Supplier<Rotation2d> mRobotHeadingSupplier;
    private final KinematicLimits mRotationKinematicLimits;
    private final boolean mFinishAtRotation;
    private final PIDController mRotationController;

    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mRotationState = new TrapezoidProfile.State();
    private boolean mAtTargetRotation = false;

    RotateToFieldHeading(
            Drive drive,
            Supplier<Rotation2d> targetHeadingSupplier,
            Supplier<Rotation2d> robotHeadingSupplier,
            KinematicLimits kinematicLimits,
            boolean finishAtRotation) {
        mDrive = drive;
        mTargetHeadingSupplier = targetHeadingSupplier;
        mRobotHeadingSupplier = robotHeadingSupplier;
        mRotationKinematicLimits = kinematicLimits;
        mFinishAtRotation = finishAtRotation;

        mRotationController = new PIDController(
                kRotationKp.get(), kRotationKi.get(), kRotationKd.get(), Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Math.min(kMaxAngularVelocity.get(), mRotationKinematicLimits.maxDriveVelocity() / kModuleRadius),
                Math.min(
                        kMaxAngularAcceleration.get(),
                        mRotationKinematicLimits.maxDriveAcceleration() / kModuleRadius)));

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(Constants.Drive.kFastKinematicLimits);

        mRotationController.reset();
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mAtTargetRotation = false;

        var fieldRelativeChassisSpeeds = mDrive.getFieldRelativeVelocity();
        var headingError = mRobotHeadingSupplier.get().minus(mTargetHeadingSupplier.get());
        mRotationState = new TrapezoidProfile.State(
                MathUtil.angleModulus(headingError.getRadians()), fieldRelativeChassisSpeeds.omegaRadiansPerSecond);

        var hash = hashCode();
        if (kMaxAngularVelocity.hasChanged(hash)
                || kMaxAngularAcceleration.hasChanged(hash)
                || kRotationKp.hasChanged(hash)
                || kRotationKi.hasChanged(hash)
                || kRotationKd.hasChanged(hash)) {
            mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    Math.min(kMaxAngularVelocity.get(), mRotationKinematicLimits.maxDriveVelocity() / kModuleRadius),
                    Math.min(
                            kMaxAngularAcceleration.get(),
                            mRotationKinematicLimits.maxDriveAcceleration() / kModuleRadius)));
            mRotationController.setPID(kRotationKp.get(), kRotationKi.get(), kRotationKd.get());
        }
    }

    @Override
    public void execute() {
        var currentHeading = mRobotHeadingSupplier.get();
        var targetHeading = mTargetHeadingSupplier.get();
        var headingError = currentHeading.minus(targetHeading);
        Rotation2d setpoint;
        mAtTargetRotation = MathUtil.isNear(0, headingError.getRadians(), kRotationToleranceRadians.get());
        double rotationalVelocity;
        if (MathUtil.isNear(0, headingError.getRadians(), 0.1)) {
            var rotationPidOutput = mRotationController.calculate(headingError.getRadians(), 0);
            rotationalVelocity = rotationPidOutput;
            mRotationState = kZeroState;
            setpoint = targetHeading;
        } else {
            var rotationPidOutput = mRotationController.calculate(headingError.getRadians(), mRotationState.position);
            mRotationState = mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, kZeroState);
            rotationalVelocity = mRotationState.velocity + rotationPidOutput;
            setpoint = Rotation2d.fromRadians(targetHeading.getRadians() + mRotationState.position);
        }

        mDrive.setVelocity(new ChassisSpeeds(0, 0, rotationalVelocity));

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
        return mFinishAtRotation && !mAtTargetRotation;
    }
}
