package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.lib.util.Util;
import com.team1701.lib.util.tuning.LoggedTunableBoolean;
import com.team1701.lib.util.tuning.LoggedTunableNumber;
import com.team1701.lib.util.tuning.LoggedTunableValue;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.ShootingState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSpeeds;
import com.team1701.robot.util.FieldUtil;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ShootAndMove extends Command {
    private static final String kLoggingPrefix = "Command/ShootAndMove/";
    private static final double kModuleRadius = Constants.Drive.kModuleRadius;
    private static final TrapezoidProfile.State kZeroState = new TrapezoidProfile.State(0.0, 0.0);
    private static final KinematicLimits kKinematicLimits = Constants.Drive.kFastSmoothKinematicLimits;

    private static final LoggedTunableBoolean mTuningEnabled =
            new LoggedTunableBoolean(kLoggingPrefix + "TuningEnabled", false);

    private static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularVelocity",
            Constants.Drive.kFastTrapezoidalKinematicLimits.maxDriveVelocity() / kModuleRadius);
    private static final LoggedTunableNumber kMaxAngularAcceleration =
            new LoggedTunableNumber(kLoggingPrefix + "MaxAngularAcceleration", kMaxAngularVelocity.get() / 2.0);

    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.01);
    private static final LoggedTunableNumber kSpeedToleranceRadiansPerSecond =
            new LoggedTunableNumber(kLoggingPrefix + "SpeedToleranceRadiansPerSecond", 25.0);
    private Rotation2d headingTolerance;

    private static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber(kLoggingPrefix + "RotationKp", 6.0);
    private static final LoggedTunableNumber kRotationKi = new LoggedTunableNumber(kLoggingPrefix + "RotationKi", 0.0);
    private static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber(kLoggingPrefix + "RotationKd", 0.5);

    private final Drive mDrive;
    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;
    private final Supplier<Translation2d> mFieldRelativeSpeeds;
    private final boolean mEndAfterShooting;
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
            Supplier<Translation2d> fieldRelativeSpeeds,
            boolean endAfterShooting) {
        mDrive = drive;
        mShooter = shooter;
        mIndexer = indexer;
        mRobotState = robotState;
        mFieldRelativeSpeeds = fieldRelativeSpeeds;
        mEndAfterShooting = endAfterShooting;
        mRotationController = new PIDController(
                kRotationKp.get(), kRotationKi.get(), kRotationKd.get(), Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(kMaxAngularVelocity.get(), kMaxAngularAcceleration.get()));
        mLockedReadyToShoot = new TimeLockedBoolean(.2, Timer.getFPGATimestamp());

        addRequirements(mDrive, mShooter, mIndexer);
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

        LoggedTunableValue.ifChanged(
                hashCode(),
                () -> {
                    mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                            Math.min(kMaxAngularVelocity.get(), kKinematicLimits.maxDriveVelocity() / kModuleRadius),
                            Math.min(
                                    kMaxAngularAcceleration.get(),
                                    kKinematicLimits.maxDriveAcceleration() / kModuleRadius)));
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
        var currentPose = mRobotState.getPose2d();
        var fieldRelativeSpeeds = mFieldRelativeSpeeds.get();
        var droppedVelocity = Constants.Shooter.kShooterSpeedInterpolator.get(mRobotState.getDistanceToSpeaker())
                * Units.inchesToMeters(2) // wheel radius
                * .907; // calculated drop in roller speed
        var headingAngleFromSpeaker = mFieldRelativeSpeeds
                .get()
                .getAngle()
                .minus(mRobotState
                        .getSpeakerHeading()
                        .plus(Rotation2d.fromRadians(Constants.Shooter.kShooterHeadingOffsetInterpolator.get(
                                mRobotState.getDistanceToSpeaker()))));
        var robotVelocityTowardsSpeaker =
                mDrive.getSpeedMetersPerSecond() * Math.cos(headingAngleFromSpeaker.getRadians());
        var timeInAir = mRobotState.getDistanceToSpeaker() / (robotVelocityTowardsSpeaker + droppedVelocity);
        var endTranslation = new Translation2d(
                currentPose.getX() + fieldRelativeSpeeds.getX() * timeInAir,
                currentPose.getY() + fieldRelativeSpeeds.getY() * timeInAir);

        var shooterSetpoint = ShooterUtil.calculateSetpoint(FieldUtil.getDistanceToSpeaker(endTranslation));

        var targetHeading = mRobotState
                .getSpeakerPose()
                .toTranslation2d()
                .minus(endTranslation)
                .getAngle()
                .minus(shooterSetpoint.releaseAngle());
        var headingError = currentPose.getRotation().minus(targetHeading);

        // headingTolerance = Rotation2d.fromDegrees(0.5);
        headingTolerance = mRobotState.getToleranceSpeakerHeading();

        Rotation2d setpoint;
        double rotationalVelocity;
        if (GeometryUtil.isNear(GeometryUtil.kRotationIdentity, headingError, headingTolerance.times(0.95))
                && Util.epsilonEquals(fieldRelativeSpeeds.getX(), 0)
                && Util.epsilonEquals(fieldRelativeSpeeds.getY(), 0)) {
            rotationalVelocity = 0;
            mRotationState = kZeroState;
            setpoint = targetHeading;
        } else {
            var rotationPidOutput = mRotationController.calculate(headingError.getRadians(), 0);
            mRotationState = mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, kZeroState);
            rotationalVelocity = mRotationState.velocity + rotationPidOutput;
            setpoint = Rotation2d.fromRadians(targetHeading.getRadians() + mRotationState.position);
        }

        mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds.getX(), fieldRelativeSpeeds.getY(), rotationalVelocity, currentPose.getRotation()));

        Rotation2d targetShooterAngle;
        ShooterSpeeds targetRollerSpeeds;
        if (mTuningEnabled.get()) {
            targetShooterAngle = Rotation2d.fromRadians(Constants.Shooter.kTunableShooterAngleRadians.get());
            targetRollerSpeeds = new ShooterSpeeds(Constants.Shooter.kTunableShooterSpeedRadiansPerSecond.get());
        } else {
            targetShooterAngle = shooterSetpoint.angle();
            targetRollerSpeeds = shooterSetpoint.speeds();
        }

        mShooter.setRotationAngle(targetShooterAngle);

        mShooter.setRollerSpeeds(targetRollerSpeeds);

        var atAngle = GeometryUtil.isNear(
                mShooter.getAngle(), targetShooterAngle, Rotation2d.fromRadians(kAngleToleranceRadians.get()));

        var atHeading = GeometryUtil.isNear(targetHeading, mRobotState.getHeading(), headingTolerance);

        var atSpeed = targetRollerSpeeds.allMatch(
                mShooter.getRollerSpeedsRadiansPerSecond(), kSpeedToleranceRadiansPerSecond.get());

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

        Logger.recordOutput(kLoggingPrefix + "Setpoint", new Pose2d(endTranslation, setpoint));
        Logger.recordOutput(kLoggingPrefix + "TargetHeading", targetHeading);
        Logger.recordOutput(kLoggingPrefix + "TargetAngle", targetShooterAngle);
        Logger.recordOutput(kLoggingPrefix + "TargetSpeed", targetRollerSpeeds.upperSpeed());
        Logger.recordOutput(kLoggingPrefix + "TimeInAir", timeInAir);
        Logger.recordOutput(kLoggingPrefix + "VelocityTowardsSpeaker", robotVelocityTowardsSpeaker);

        Logger.recordOutput(
                kLoggingPrefix + "EndTranslation", new Pose2d(endTranslation, GeometryUtil.kRotationIdentity));
        Logger.recordOutput(kLoggingPrefix + "AtAngle", atAngle);
        Logger.recordOutput(kLoggingPrefix + "AtHeading", atHeading);
        Logger.recordOutput(kLoggingPrefix + "AtSpeed", atSpeed);
        Logger.recordOutput(kLoggingPrefix + "Shooting", mShooting);
    }

    @Override
    public void end(boolean interrupted) {
        mShooting = false;

        mRobotState.setShootingState(new ShootingState());
        mShooter.stop();
        mIndexer.stop();
        mDrive.stop();

        Logger.recordOutput(kLoggingPrefix + "Shooting", false);
        Logger.recordOutput(kLoggingPrefix + "AtAngle", false);
        Logger.recordOutput(kLoggingPrefix + "AtHeading", false);
        Logger.recordOutput(kLoggingPrefix + "AtSpeed", false);
    }

    @Override
    public boolean isFinished() {
        return mEndAfterShooting && mShooting && !mRobotState.hasNote();
    }
}
