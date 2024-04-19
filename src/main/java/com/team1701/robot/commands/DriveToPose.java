package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.tuning.LoggedTunableNumber;
import com.team1701.lib.util.tuning.LoggedTunableValue;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
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
    private static final TrapezoidProfile.State kZeroState = new TrapezoidProfile.State();
    private static final double kModuleRadius = Constants.Drive.kModuleRadius;
    private static final KinematicLimits kMaxKinematicLimits = Constants.Drive.kMaxTrapezoidalKinematicLimits;

    private static final LoggedTunableNumber kMaxVelocity =
            new LoggedTunableNumber(kLoggingPrefix + "MaxVelocity", kMaxKinematicLimits.maxDriveVelocity());
    private static final LoggedTunableNumber kMaxAcceleration =
            new LoggedTunableNumber(kLoggingPrefix + "MaxAcceleration", kMaxKinematicLimits.maxDriveAcceleration());
    private static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularVelocity", kMaxKinematicLimits.maxDriveVelocity() / kModuleRadius);
    private static final LoggedTunableNumber kMaxAngularAcceleration = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularAcceleration", kMaxKinematicLimits.maxDriveAcceleration() / kModuleRadius);
    private static final LoggedTunableNumber kTranslationKp =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationKp", 6.0);
    private static final LoggedTunableNumber kTranslationKi =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationKi", 0.0);
    private static final LoggedTunableNumber kTranslationKd =
            new LoggedTunableNumber(kLoggingPrefix + "TranslationKd", 0.0);
    private static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber(kLoggingPrefix + "RotationKp", 6.0);
    private static final LoggedTunableNumber kRotationKi = new LoggedTunableNumber(kLoggingPrefix + "RotationKi", 0.0);
    private static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber(kLoggingPrefix + "RotationKd", 0.5);
    private static DoubleSupplier kTranslationToleranceMeters;
    private static final LoggedTunableNumber kRotationToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "RotationToleranceRadians", 0.01);

    private final Drive mDrive;
    private final RobotState mRobotState;
    private final Supplier<Pose2d> mTargetPoseSupplier;
    private final Supplier<Pose2d> mRobotPoseSupplier;
    private final KinematicLimits mKinematicLimits;
    private final boolean mFinishAtPose;
    private final PIDController mTranslationController;
    private final PIDController mRotationController;

    private TrapezoidProfile mTranslationProfile;
    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mTranslationState = new TrapezoidProfile.State();
    private TrapezoidProfile.State mRotationState = new TrapezoidProfile.State();
    private boolean mAtTargetPose = false;
    private boolean mEndIfHasPiece = false;

    DriveToPose(
            Drive drive,
            RobotState robotState,
            Supplier<Pose2d> poseSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            KinematicLimits kinematicLimits,
            double translationTolerance,
            boolean finishAtPose) {
        this(
                drive,
                robotState,
                poseSupplier,
                robotPoseSupplier,
                kinematicLimits,
                translationTolerance,
                finishAtPose,
                false);
    }

    DriveToPose(
            Drive drive,
            RobotState robotState,
            Supplier<Pose2d> poseSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            KinematicLimits kinematicLimits,
            double translationTolerance,
            boolean finishAtPose,
            boolean endIfHasPiece) {
        mDrive = drive;
        mRobotState = robotState;
        mTargetPoseSupplier = poseSupplier;
        mRobotPoseSupplier = robotPoseSupplier;
        mKinematicLimits = kinematicLimits;
        mFinishAtPose = finishAtPose;
        mEndIfHasPiece = endIfHasPiece;

        kTranslationToleranceMeters = () -> translationTolerance;
        mTranslationController = new PIDController(
                kTranslationKp.get(), kTranslationKi.get(), kTranslationKd.get(), Constants.kLoopPeriodSeconds);
        mTranslationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Math.min(kMaxVelocity.get(), mKinematicLimits.maxDriveVelocity()),
                Math.min(kMaxAcceleration.get(), mKinematicLimits.maxDriveAcceleration())));

        mRotationController = new PIDController(
                kRotationKp.get(), kRotationKi.get(), kRotationKd.get(), Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Math.min(kMaxAngularVelocity.get(), mKinematicLimits.maxDriveVelocity() / kModuleRadius),
                Math.min(kMaxAngularAcceleration.get(), mKinematicLimits.maxDriveAcceleration() / kModuleRadius)));

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(Constants.Drive.kFastKinematicLimits);
        var targetPose = mTargetPoseSupplier.get();

        mTranslationController.reset();
        mRotationController.reset();
        mAtTargetPose = false;

        var currentPose = mRobotPoseSupplier.get();
        var translationToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        var rotationToTarget = translationToTarget.getAngle();
        var fieldRelativeChassisSpeeds = mDrive.getFieldRelativeVelocity();
        var velocityToTarget = ChassisSpeeds.fromFieldRelativeSpeeds(
                        mRobotState.getFieldRelativeSpeedSetpoint(), rotationToTarget)
                .vxMetersPerSecond;
        mTranslationState = new TrapezoidProfile.State(translationToTarget.getNorm(), -velocityToTarget);
        mRotationState = new TrapezoidProfile.State(
                MathUtil.inputModulus(
                        currentPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians() - Math.PI,
                        targetPose.getRotation().getRadians() + Math.PI),
                fieldRelativeChassisSpeeds.omegaRadiansPerSecond);

        LoggedTunableValue.ifChanged(
                hashCode(),
                () -> {
                    mTranslationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                            Math.min(kMaxVelocity.get(), mKinematicLimits.maxDriveVelocity()),
                            Math.min(kMaxAcceleration.get(), mKinematicLimits.maxDriveAcceleration())));
                    mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                            Math.min(kMaxAngularVelocity.get(), mKinematicLimits.maxDriveVelocity() / kModuleRadius),
                            Math.min(
                                    kMaxAngularAcceleration.get(),
                                    mKinematicLimits.maxDriveAcceleration() / kModuleRadius)));
                    mTranslationController.setPID(kTranslationKp.get(), kTranslationKi.get(), kTranslationKd.get());
                    mRotationController.setPID(kRotationKp.get(), kRotationKi.get(), kRotationKd.get());
                },
                kMaxVelocity,
                kMaxAcceleration,
                kMaxAngularVelocity,
                kMaxAngularAcceleration,
                kTranslationKp,
                kTranslationKi,
                kTranslationKd,
                kRotationKp,
                kRotationKi,
                kRotationKd);
        Logger.recordOutput(kLoggingPrefix + "InitialVelocity", velocityToTarget);
    }

    @Override
    public void execute() {
        var currentPose = mRobotPoseSupplier.get();
        var targetPose = mTargetPoseSupplier.get();
        Pose2d setpoint;
        mAtTargetPose = GeometryUtil.isNear(
                targetPose,
                currentPose,
                kTranslationToleranceMeters.getAsDouble(),
                Rotation2d.fromRadians(kRotationToleranceRadians.get()));
        if (mAtTargetPose) {
            mDrive.stop();
            setpoint = currentPose;
        } else {
            var translationToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
            var distanceToTarget = translationToTarget.getNorm();
            var headingToTarget = translationToTarget.getAngle();

            // Calculate directional velocity
            var translationPidOutput = mTranslationController.calculate(distanceToTarget, mTranslationState.position);
            mTranslationState = new TrapezoidProfile.State(
                    Math.min(distanceToTarget, mTranslationState.position), mTranslationState.velocity);
            mTranslationState =
                    mTranslationProfile.calculate(Constants.kLoopPeriodSeconds, mTranslationState, kZeroState);
            var velocity = new Translation2d(-(mTranslationState.velocity + translationPidOutput), headingToTarget);

            // Calculate rotational velocity
            var currentRotationRadians = MathUtil.inputModulus(
                    currentPose.getRotation().getRadians(),
                    targetPose.getRotation().getRadians() - Math.PI,
                    targetPose.getRotation().getRadians() + Math.PI);
            var rotationPidOutput = mRotationController.calculate(currentRotationRadians, mRotationState.position);
            var rotationTargetState =
                    new TrapezoidProfile.State(targetPose.getRotation().getRadians(), 0.0);
            mRotationState =
                    mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, rotationTargetState);
            var rotationalVelocity = mRotationState.velocity + rotationPidOutput;

            // Set drive outputs
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    velocity.getX(), velocity.getY(), rotationalVelocity, currentPose.getRotation()));
            setpoint = new Pose2d(
                    targetPose.getTranslation().minus(new Translation2d(mTranslationState.position, headingToTarget)),
                    Rotation2d.fromRadians(mRotationState.position));
        }

        Logger.recordOutput(kLoggingPrefix + "TranslationError", mTranslationController.getPositionError());
        Logger.recordOutput(
                kLoggingPrefix + "RotationError", Rotation2d.fromRadians(mRotationController.getPositionError()));
        Logger.recordOutput(kLoggingPrefix + "Setpoint", setpoint);
        Logger.recordOutput(kLoggingPrefix + "TargetPose", targetPose);
        Logger.recordOutput(
                kLoggingPrefix + "AtTranslation",
                GeometryUtil.isNear(
                        targetPose.getTranslation(),
                        currentPose.getTranslation(),
                        kTranslationToleranceMeters.getAsDouble()));
        Logger.recordOutput(
                kLoggingPrefix + "AtRotation",
                MathUtil.isNear(
                        targetPose.getRotation().getRadians(),
                        currentPose.getRotation().getRadians(),
                        kRotationToleranceRadians.get()));
        Logger.recordOutput(kLoggingPrefix + "translationTolerance", kTranslationToleranceMeters.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return (mFinishAtPose && mAtTargetPose) || (mEndIfHasPiece && mRobotState.hasNote());
    }
}
