package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.lib.util.ShooterUtil;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.ShootingState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ShootAndMove extends Command {
    private static final String kLoggingPrefix = "Command/ShootAndMove/";
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

    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.01);
    private static final LoggedTunableNumber kSpeedToleranceRadiansPerSecond =
            new LoggedTunableNumber(kLoggingPrefix + "SpeedToleranceRadiansPerSecond", 50.0);
    private static final LoggedTunableNumber kHeadingToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "HeadingToleranceDegrees", 2);

    private static final LoggedTunableNumber kLoopsLatency =
            new LoggedTunableNumber(kLoggingPrefix + "LoopsLatency", 15.0);
    private static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber(kLoggingPrefix + "RotationKp", 6.0);
    private static final LoggedTunableNumber kRotationKi = new LoggedTunableNumber(kLoggingPrefix + "RotationKi", 0.0);
    private static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber(kLoggingPrefix + "RotationKd", 0.0);

    private final Drive mDrive;
    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;
    private final Supplier<Translation2d> mFieldRelativeSpeeds;
    private final PIDController mRotationController;

    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mRotationState = kZeroState;

    private TimeLockedBoolean mLockedReadyToShoot;
    private boolean mShooting;

    ShootAndMove(
            Drive drive,
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            Supplier<Translation2d> fieldRelativeSpeeds) {
        mDrive = drive;
        mShooter = shooter;
        mIndexer = indexer;
        mRobotState = robotState;
        mFieldRelativeSpeeds = fieldRelativeSpeeds;
        mRotationController = new PIDController(
                kRotationKp.get(), kRotationKi.get(), kRotationKd.get(), Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(kMaxAngularVelocity.get(), kMaxAngularAcceleration.get()));
        mLockedReadyToShoot = new TimeLockedBoolean(.1, Timer.getFPGATimestamp());
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

        mLockedReadyToShoot.update(false, Timer.getFPGATimestamp());
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

        var targetShooterAngle = GeometryUtil.clampRotation(
                ShooterUtil.calculateShooterAngleWithMotion(mRobotState, endTranslation),
                Constants.Shooter.kShooterLowerLimitRotations,
                Constants.Shooter.kShooterUpperLimitRotations);

        mShooter.setRotationAngle(targetShooterAngle);

        var targetRollerSpeeds = ShooterUtil.calculateShooterSpeedsWithMotion(mRobotState, endTranslation);
        mShooter.setRollerSpeeds(targetRollerSpeeds);

        var atAngle = GeometryUtil.isNear(
                mShooter.getAngle(), targetShooterAngle, Rotation2d.fromRadians(kAngleToleranceRadians.get()));

        var atHeading = GeometryUtil.isNear(
                targetHeading, mRobotState.getHeading(), Rotation2d.fromRadians(kHeadingToleranceRadians.get()));

        var atSpeed = Util.sequentiallyMatch(
                targetRollerSpeeds, mShooter.getRollerSpeedsRadiansPerSecond(), kSpeedToleranceRadiansPerSecond.get());

        if (mLockedReadyToShoot.update(atAngle && atHeading && atSpeed, Timer.getFPGATimestamp())) {
            mIndexer.setForwardShoot();
            mShooting = true;
        }

        if (!mShooting) {
            if (mIndexer.hasNoteAtExit()) {
                mIndexer.stop();
            } else {
                mIndexer.setForwardLoad();
            }
        }
        mRobotState.setShootingState(new ShootingState(true, atAngle, atSpeed, atHeading, mShooting));

        Logger.recordOutput(kLoggingPrefix + "Setpoint", setpoint);
        Logger.recordOutput(kLoggingPrefix + "TargetHeading", targetHeading);
        Logger.recordOutput(kLoggingPrefix + "AtAngle", atAngle);
        Logger.recordOutput(kLoggingPrefix + "AtHeading", atHeading);
        Logger.recordOutput(kLoggingPrefix + "AtSpeed", atSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        mShooting = false;

        mRobotState.setShootingState(new ShootingState());
        mShooter.stopRollers();
        mShooter.stopRotation();
        mIndexer.stop();
        mDrive.stop();

        Logger.recordOutput(kLoggingPrefix + "Shooting", false);
        Logger.recordOutput(kLoggingPrefix + "AtAngle", false);
        Logger.recordOutput(kLoggingPrefix + "AtHeading", false);
        Logger.recordOutput(kLoggingPrefix + "AtSpeed", false);
    }

    @Override
    public boolean isFinished() {
        return mShooting && !mRobotState.hasNote();
    }
}