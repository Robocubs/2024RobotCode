package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.Util;
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

public class DriveWithAimAssist extends Command {
    private static final String kLoggingPrefix = "Command/DriveWithAimAssist/";
    private static final double kModuleRadius = Constants.Drive.kModuleRadius;
    private static final TrapezoidProfile.State kZeroState = new TrapezoidProfile.State(0.0, 0.0);

    private static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularVelocity",
            Constants.Drive.kFastKinematicLimits.maxDriveVelocity() / kModuleRadius);
    private static final LoggedTunableNumber kMaxAngularAcceleration = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularAcceleration",
            Constants.Drive.kFastKinematicLimits.maxDriveAcceleration() / kModuleRadius);

    private static final LoggedTunableNumber kLoopsLatency =
            new LoggedTunableNumber(kLoggingPrefix + "LoopsLatency", 2.0);
    private static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber(kLoggingPrefix + "RotationKp", 5.0);
    private static final LoggedTunableNumber kRotationKi = new LoggedTunableNumber(kLoggingPrefix + "RotationKi", 0.0);
    private static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber(kLoggingPrefix + "RotationKd", 0.0);

    private final Drive mDrive;
    private final RobotState mRobotState;
    private final DoubleSupplier mThrottleSupplier;
    private final DoubleSupplier mStrafeSupplier;
    private final DoubleSupplier mRotationSupplier;
    private final Supplier<KinematicLimits> mKinematicLimits;
    private final PIDController mRotationController;

    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mRotationState = kZeroState;

    public DriveWithAimAssist(
            Drive drive,
            RobotState robotState,
            DoubleSupplier throttleSupplier,
            DoubleSupplier strafeSupplier,
            DoubleSupplier rotationSupplier,
            Supplier<KinematicLimits> kinematicLimits) {
        mDrive = drive;
        mRobotState = robotState;
        mThrottleSupplier = throttleSupplier;
        mStrafeSupplier = strafeSupplier;
        mRotationSupplier = rotationSupplier;
        mKinematicLimits = kinematicLimits;
        mRotationController = new PIDController(
                kRotationKp.get(), kRotationKi.get(), kRotationKd.get(), Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Math.min(kMaxAngularVelocity.get(), mKinematicLimits.get().maxDriveVelocity() / kModuleRadius),
                Math.min(
                        kMaxAngularAcceleration.get(), mKinematicLimits.get().maxDriveAcceleration() / kModuleRadius)));

        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(mKinematicLimits.get());

        mRotationController.reset();

        var fieldRelativeChassisSpeeds = mRobotState.getFieldRelativeSpeeds();
        var detectedNoteHeading = mRobotState.getHeadingToDetectedNoteForPickup();
        var headingError = detectedNoteHeading.isEmpty()
                ? GeometryUtil.kRotationIdentity
                : mRobotState.getHeadingToDetectedNoteForPickup().get().minus(mRobotState.getHeading());
        mRotationState = new TrapezoidProfile.State(
                MathUtil.angleModulus(headingError.getRadians()), fieldRelativeChassisSpeeds.omegaRadiansPerSecond);

        LoggedTunableValue.ifChanged(
                hashCode(),
                () -> {
                    mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                            Math.min(
                                    kMaxAngularVelocity.get(),
                                    mKinematicLimits.get().maxDriveVelocity() / kModuleRadius),
                            Math.min(
                                    kMaxAngularAcceleration.get(),
                                    mKinematicLimits.get().maxDriveAcceleration() / kModuleRadius)));
                    mRotationController.setPID(kRotationKp.get(), kRotationKi.get(), kRotationKd.get());
                },
                kMaxAngularVelocity,
                kMaxAngularAcceleration,
                kRotationKp,
                kRotationKi,
                kRotationKd);
    }

    @Override
    public void execute() {
        mDrive.setKinematicLimits(mKinematicLimits.get());
        var currentPose = mRobotState.getPose2d();
        var endTranslation = new Translation2d(
                currentPose.getX()
                        + mThrottleSupplier.getAsDouble() * Constants.kLoopPeriodSeconds * kLoopsLatency.get(),
                currentPose.getY()
                        + mStrafeSupplier.getAsDouble() * Constants.kLoopPeriodSeconds * kLoopsLatency.get());
        var detectedNoteHeading = mRobotState.getHeadingToDetectedNoteForPickup();
        var targetHeading = detectedNoteHeading.isEmpty()
                ? mRobotState.getHeading()
                : detectedNoteHeading.get().plus(GeometryUtil.kRotationPi);
        var headingError = detectedNoteHeading.isEmpty()
                ? GeometryUtil.kRotationIdentity
                : targetHeading.minus(mRobotState.getHeading());
        var headingTolerance = Rotation2d.fromRadians(0.03);
        var translationVelocities = DriveCommands.calculateDriveWithJoysticksVelocities(
                mThrottleSupplier.getAsDouble(),
                mStrafeSupplier.getAsDouble(),
                mDrive.getFieldRelativeHeading(),
                mKinematicLimits.get().maxDriveVelocity());
        var rotation = MathUtil.applyDeadband(mRotationSupplier.getAsDouble(), Constants.Controls.kDriverDeadband);
        var rotationRadiansPerSecond = Math.copySign(rotation * rotation, rotation)
                * mKinematicLimits.get().maxDriveVelocity()
                / Constants.Drive.kModuleRadius;
        var toleranceDegrees = detectedNoteHeading.isEmpty()
                ? 0.0
                : (180)
                        / Math.pow(
                                Math.abs(mRobotState
                                        .getDistanceToDetectedNoteForPickup()
                                        .getAsDouble()),
                                2);

        var fieldRelativeVelocityHeading =
                translationVelocities.rotateBy(mRobotState.getHeading()).getAngle();

        Logger.recordOutput(kLoggingPrefix + "ToleranceDegrees", toleranceDegrees);
        Logger.recordOutput(kLoggingPrefix + "TargetHeading", targetHeading);
        Logger.recordOutput(kLoggingPrefix + "TranslationalVelocityAngle", fieldRelativeVelocityHeading);

        if (detectedNoteHeading.isEmpty()
                || mRobotState.hasNote()
                || Math.abs(mRotationSupplier.getAsDouble()) > 0.5
                || !GeometryUtil.isNear(
                        fieldRelativeVelocityHeading,
                        detectedNoteHeading.get(),
                        Rotation2d.fromDegrees(toleranceDegrees))) {
            mDrive.setVelocity(new ChassisSpeeds(
                    translationVelocities.getX(), translationVelocities.getY(), rotationRadiansPerSecond));
            return;
        }

        Rotation2d setpoint;
        double rotationalVelocity;
        if (GeometryUtil.isNear(GeometryUtil.kRotationIdentity, headingError, headingTolerance)
                && Util.epsilonEquals(mThrottleSupplier.getAsDouble(), 0)
                && Util.epsilonEquals(mStrafeSupplier.getAsDouble(), 0)) {
            rotationalVelocity = 0;
            mRotationState = kZeroState;
            setpoint = targetHeading;
        } else {
            var rotationPidOutput = mRotationController.calculate(headingError.getRadians(), mRotationState.position);
            mRotationState = mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, kZeroState);
            rotationalVelocity = mRotationState.velocity + rotationPidOutput;
            setpoint = Rotation2d.fromRadians(targetHeading.getRadians() + mRotationState.position);
        }

        mDrive.setVelocity(new ChassisSpeeds(
                translationVelocities.getX(),
                translationVelocities.getY(),
                -rotationalVelocity + (rotationRadiansPerSecond * 0.5)));
        Logger.recordOutput(kLoggingPrefix + "Setpoint", new Pose2d(endTranslation, setpoint));
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }
}
