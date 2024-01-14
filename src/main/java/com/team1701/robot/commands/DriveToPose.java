package com.team1701.robot.commands;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
    private static final String kLoggingPrefix = "Command/DriveToPose/";
    private static final double kModuleRadius = Constants.Drive.kModuleRadius;
    private static final KinematicLimits kMaxKinematicLimits = Constants.Drive.kFastTrapezoidalKinematicLimits;
    private static final LoggedTunableNumber kMaxVelocity =
            new LoggedTunableNumber(kLoggingPrefix + "MaxVelocity", kMaxKinematicLimits.maxDriveVelocity());
    private static final LoggedTunableNumber kMaxAcceleration =
            new LoggedTunableNumber(kLoggingPrefix + "MaxAcceleration", kMaxKinematicLimits.maxDriveVelocity() / 2.0);
    private static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularVelocity", kMaxKinematicLimits.maxDriveVelocity() / kModuleRadius);
    private static final LoggedTunableNumber kMaxAngularAcceleration =
            new LoggedTunableNumber(kLoggingPrefix + "MaxAngularAcceleration", kMaxAngularVelocity.get() / 2.0);
    // TODO: Update PID values
    private static final LoggedTunableNumber kTranslationKp =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationKp", 6.0);
    private static final LoggedTunableNumber kTranslationKi =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationKi", 0.0);
    private static final LoggedTunableNumber kTranslationKd =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationKd", 0.0);
    private static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber(kLoggingPrefix + "RotationKp", 4.0);
    private static final LoggedTunableNumber kRotationKi = new LoggedTunableNumber(kLoggingPrefix + "RotationKi", 0.0);
    private static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber(kLoggingPrefix + "RotationKd", 0.0);
    private static final LoggedTunableNumber kTranslationToleranceMeters =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationToleranceMeters", 0.01);
    private static final LoggedTunableNumber kRotationToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "RotationToleranceRadians", 0.01);

    private final Drive mDrive;
    private final Pose2d mTargetPose;
    private final KinematicLimits mKinematicLimits;
    private final boolean mFinishAtPose;
    private final PIDController mTranslationController;
    private final PIDController mRotationController;

    private Pose2d mSetpoint = GeometryUtil.kPoseIdentity;
    private TrapezoidProfile mTranslationProfile;
    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mTranslationState = new TrapezoidProfile.State();
    private TrapezoidProfile.State mRotationState = new TrapezoidProfile.State();

    DriveToPose(Drive drive, Pose2d pose, KinematicLimits kinematicLimits, boolean finishAtPose) {
        mDrive = drive;
        mTargetPose = pose;
        mKinematicLimits = kinematicLimits;
        mFinishAtPose = finishAtPose;

        mTranslationController = new PIDController(0.0, 0.0, 0.0, Constants.kLoopPeriodSeconds);
        mTranslationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.0, 0.0));

        mRotationController = new PIDController(0.0, 0.0, 0.0, Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.0, 0.0));

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(Constants.Drive.kFastKinematicLimits);
        mSetpoint = PoseEstimator.getInstance().getPose2d();

        mTranslationController.reset();
        mRotationController.reset();

        var currentPose = PoseEstimator.getInstance().getPose2d();
        var translationToTarget = mTargetPose.getTranslation().minus(currentPose.getTranslation());
        var rotationToTarget = translationToTarget.getAngle();
        var fieldRelativeChassisSpeeds = mDrive.getFieldRelativeVelocity();
        var velocityToTarget =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeChassisSpeeds, rotationToTarget).vxMetersPerSecond;
        mTranslationState = new TrapezoidProfile.State(translationToTarget.getNorm(), -velocityToTarget);
        mRotationState = new TrapezoidProfile.State(
                MathUtil.inputModulus(
                        currentPose.getRotation().getRadians(),
                        mTargetPose.getRotation().getRadians() - Math.PI,
                        mTargetPose.getRotation().getRadians() + Math.PI),
                fieldRelativeChassisSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        var hash = hashCode();
        if (kMaxVelocity.hasChanged(hash)
                || kMaxAcceleration.hasChanged(hash)
                || kMaxAngularVelocity.hasChanged(hash)
                || kMaxAngularAcceleration.hasChanged(hash)
                || kTranslationKp.hasChanged(hash)
                || kTranslationKi.hasChanged(hash)
                || kTranslationKd.hasChanged(hash)
                || kRotationKp.hasChanged(hash)
                || kRotationKi.hasChanged(hash)
                || kRotationKd.hasChanged(hash)) {
            mTranslationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    Math.min(kMaxVelocity.get(), mKinematicLimits.maxDriveVelocity()),
                    Math.min(kMaxAcceleration.get(), mKinematicLimits.maxDriveAcceleration())));
            mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    Math.min(kMaxAngularVelocity.get(), mKinematicLimits.maxDriveVelocity() / kModuleRadius),
                    Math.min(kMaxAngularAcceleration.get(), mKinematicLimits.maxDriveAcceleration() / kModuleRadius)));
            mTranslationController.setPID(kTranslationKp.get(), kTranslationKi.get(), kTranslationKd.get());
            mRotationController.setPID(kRotationKp.get(), kRotationKi.get(), kRotationKd.get());
        }

        var currentPose = PoseEstimator.getInstance().getPose2d();
        var translationToTarget = mTargetPose.getTranslation().minus(currentPose.getTranslation());
        var distanceToTarget = translationToTarget.getNorm();
        var headingToTarget = translationToTarget.getAngle();

        // Calculate directional velocity
        var translationPidOutput = mTranslationController.calculate(distanceToTarget, mTranslationState.position);
        var translationTargetState = new TrapezoidProfile.State(0.0, 0.0);
        mTranslationState =
                mTranslationProfile.calculate(Constants.kLoopPeriodSeconds, mTranslationState, translationTargetState);
        var velocity = new Translation2d(-(mTranslationState.velocity + translationPidOutput), headingToTarget);

        // Calculate rotational velocity
        var rotationPidOutput =
                mRotationController.calculate(currentPose.getRotation().getRadians(), mRotationState.position);
        var rotationTargetState =
                new TrapezoidProfile.State(mTargetPose.getRotation().getRadians(), 0.0);
        mRotationState = mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, rotationTargetState);
        var rotationalVelocity = mRotationState.velocity + rotationPidOutput;

        // Set drive outputs
        var atTargetPose = distanceToTarget < kTranslationToleranceMeters.get()
                && GeometryUtil.isNear(
                        mTargetPose.getRotation(),
                        currentPose.getRotation(),
                        Rotation2d.fromRadians(kRotationToleranceRadians.get()));
        if (atTargetPose) {
            mDrive.stop();
            mSetpoint = currentPose;
        } else {
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    velocity.getX(), velocity.getY(), rotationalVelocity, currentPose.getRotation()));
            mSetpoint = new Pose2d(
                    mTargetPose.getTranslation().minus(new Translation2d(mTranslationState.position, headingToTarget)),
                    Rotation2d.fromRadians(mRotationState.position));
        }

        Logger.recordOutput(kLoggingPrefix + "TranslationError", mTranslationController.getPositionError());
        Logger.recordOutput(
                kLoggingPrefix + "RotationError", Rotation2d.fromRadians(mRotationController.getPositionError()));
        Logger.recordOutput(kLoggingPrefix + "Setpoint", mSetpoint);
        Logger.recordOutput(kLoggingPrefix + "TargetPose", mTargetPose);
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return mFinishAtPose && atTargetPose();
    }

    public boolean atTargetPose() {
        var currentPose = PoseEstimator.getInstance().getPose2d();
        var translationError =
                mTargetPose.getTranslation().minus(currentPose.getTranslation()).getNorm();
        return Util.inRange(translationError, kTranslationToleranceMeters.get())
                && GeometryUtil.isNear(
                        mTargetPose.getRotation(),
                        currentPose.getRotation(),
                        Rotation2d.fromRadians(kRotationToleranceRadians.get()));
    }
}
