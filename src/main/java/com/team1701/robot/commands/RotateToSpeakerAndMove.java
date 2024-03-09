package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class RotateToSpeakerAndMove extends Command {
    private static final String kLoggingPrefix = "Command/RotateToSpeakerAndMove/";
    private static final double kModuleRadius = Constants.Drive.kModuleRadius;
    private static final TrapezoidProfile.State kZeroState = new TrapezoidProfile.State(0.0, 0.0);
    private static final KinematicLimits kKinematicLimits = Constants.Drive.kFastKinematicLimits;

    private static final LoggedTunableNumber kMaxPAngularVelocity =
            new LoggedTunableNumber(kLoggingPrefix + "MaxAngularVelocity", 2.0);
    private static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularVelocity",
            Constants.Drive.kFastTrapezoidalKinematicLimits.maxDriveVelocity() / kModuleRadius);
    private static final LoggedTunableNumber kMaxAngularAcceleration =
            new LoggedTunableNumber(kLoggingPrefix + "MaxAngularAcceleration", kMaxAngularVelocity.get() / 2.0);

    private static final LoggedTunableNumber kLoopsLatency =
            new LoggedTunableNumber(kLoggingPrefix + "LoopsLatency", 15.0);
    private static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber(kLoggingPrefix + "RotationKp", 6.0);
    private static final LoggedTunableNumber kRotationKi = new LoggedTunableNumber(kLoggingPrefix + "RotationKi", 0.0);
    private static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber(kLoggingPrefix + "RotationKd", 0.0);

    private final Drive mDrive;
    private final RobotState mRobotState;
    private final Supplier<Translation2d> mFieldRelativeSpeeds;
    private final PIDController mRotationController;

    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mRotationState = kZeroState;

    RotateToSpeakerAndMove(Drive drive, RobotState robotState, Supplier<Translation2d> fieldRelativeSpeeds) {
        mDrive = drive;
        mRobotState = robotState;
        mFieldRelativeSpeeds = fieldRelativeSpeeds;
        mRotationController = new PIDController(
                kRotationKp.get(), kRotationKi.get(), kRotationKd.get(), Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(kMaxAngularVelocity.get(), kMaxAngularAcceleration.get()));
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(kKinematicLimits);

        mRotationController.reset();
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);

        var fieldRelativeChassisSpeeds = mDrive.getFieldRelativeVelocity();
        var headingError = mRobotState.getHeading().minus(mRobotState.getSpeakerHeading());
        mRotationState = new TrapezoidProfile.State(
                MathUtil.angleModulus(headingError.getRadians()), fieldRelativeChassisSpeeds.omegaRadiansPerSecond);
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
                    Math.min(kMaxAngularVelocity.get(), kKinematicLimits.maxDriveVelocity() / kModuleRadius),
                    Math.min(kMaxAngularAcceleration.get(), kKinematicLimits.maxDriveAcceleration() / kModuleRadius)));
            mRotationController.setPID(kRotationKp.get(), kRotationKi.get(), kRotationKd.get());
        }

        var currentPose = mRobotState.getPose2d();
        var fieldRelativeSpeeds = mFieldRelativeSpeeds.get();
        var endTranslation = new Translation2d(
                currentPose.getX() - fieldRelativeSpeeds.getX() * Constants.kLoopPeriodSeconds * kLoopsLatency.get(),
                currentPose.getY() - fieldRelativeSpeeds.getY() * Constants.kLoopPeriodSeconds * kLoopsLatency.get());
        var targetHeading = mRobotState
                .getSpeakerPose()
                .toTranslation2d()
                .minus(endTranslation)
                .getAngle();
        var headingError = currentPose.getRotation().minus(targetHeading);

        Rotation2d setpoint;
        double rotationalVelocity;
        if (MathUtil.isNear(0, headingError.getRadians(), 0.1)) {
            var rotationPidOutput = MathUtil.clamp(
                    mRotationController.calculate(headingError.getRadians(), 0),
                    -kMaxPAngularVelocity.get(),
                    kMaxPAngularVelocity.get());
            rotationalVelocity = rotationPidOutput;
            mRotationState = kZeroState;
            setpoint = targetHeading;
        } else {
            var rotationPidOutput = mRotationController.calculate(headingError.getRadians(), 0);
            mRotationState = mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, kZeroState);
            rotationalVelocity = mRotationState.velocity + rotationPidOutput;
            setpoint = Rotation2d.fromRadians(targetHeading.getRadians() + mRotationState.position);
        }

        mDrive.setVelocity(
                new ChassisSpeeds(fieldRelativeSpeeds.getX(), fieldRelativeSpeeds.getY(), rotationalVelocity));

        Logger.recordOutput(kLoggingPrefix + "Setpoint", setpoint);
        Logger.recordOutput(kLoggingPrefix + "TargetHeading", targetHeading);
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }
}
