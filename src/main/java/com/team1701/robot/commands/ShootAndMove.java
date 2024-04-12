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
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSetpoint;
import com.team1701.robot.util.FieldUtil;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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

    private static final LoggedTunableNumber kStationaryRotationKp =
            new LoggedTunableNumber(kLoggingPrefix + "StationaryRotationKp", 11.0);
    private static final LoggedTunableNumber kStationaryRotationKi =
            new LoggedTunableNumber(kLoggingPrefix + "StationaryRotationKi", 0.0);
    private static final LoggedTunableNumber kStationaryRotationKd =
            new LoggedTunableNumber(kLoggingPrefix + "StationaryRotationKd", 1.0);

    private static final LoggedTunableNumber kMovingRotationKp =
            new LoggedTunableNumber(kLoggingPrefix + "MovingRotationKp", 8.0);
    private static final LoggedTunableNumber kMovingRotationKi =
            new LoggedTunableNumber(kLoggingPrefix + "MovingRotationKi", 0.0);
    private static final LoggedTunableNumber kMovingRotationKd =
            new LoggedTunableNumber(kLoggingPrefix + "MovingRotationKd", 0.5);

    private final Drive mDrive;
    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;
    private final Supplier<Translation2d> mFieldRelativeSpeeds;
    private final boolean mEndAfterShooting;
    private final PIDController mStationaryRotationController;
    private final PIDController mMovingRotationController;

    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mRotationState = kZeroState;

    private TimeLockedBoolean mLockedReadyToShoot;
    private boolean mShooting;

    private ShooterSetpoint mLastSetpoint =
            new ShooterSetpoint(Double.POSITIVE_INFINITY, GeometryUtil.kRotationIdentity);
    private Rotation2d mLastTargetHeading = GeometryUtil.kRotationIdentity;

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
        mStationaryRotationController = new PIDController(
                kStationaryRotationKp.get(),
                kStationaryRotationKi.get(),
                kStationaryRotationKd.get(),
                Constants.kLoopPeriodSeconds);

        mMovingRotationController = new PIDController(
                kMovingRotationKp.get(),
                kMovingRotationKi.get(),
                kMovingRotationKd.get(),
                Constants.kLoopPeriodSeconds);
        mStationaryRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mMovingRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(kMaxAngularVelocity.get(), kMaxAngularAcceleration.get()));
        mLockedReadyToShoot = new TimeLockedBoolean(.1, Timer.getFPGATimestamp());

        addRequirements(mDrive, mShooter, mIndexer);
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(kKinematicLimits);

        mStationaryRotationController.reset();
        mMovingRotationController.reset();
        mStationaryRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mMovingRotationController.enableContinuousInput(-Math.PI, Math.PI);

        var fieldRelativeChassisSpeeds = mDrive.getFieldRelativeVelocity();
        var headingError = mRobotState.getHeading().minus(mRobotState.getSpeakerHeading());
        mRotationState = new TrapezoidProfile.State(
                MathUtil.angleModulus(headingError.getRadians()), fieldRelativeChassisSpeeds.omegaRadiansPerSecond);

        mLockedReadyToShoot.update(false, Timer.getFPGATimestamp());
        mLastSetpoint = new ShooterSetpoint(Double.POSITIVE_INFINITY, GeometryUtil.kRotationIdentity);
        mLastTargetHeading = GeometryUtil.kRotationIdentity;

        LoggedTunableValue.ifChanged(
                hashCode(),
                () -> {
                    mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                            Math.min(kMaxAngularVelocity.get(), kKinematicLimits.maxDriveVelocity() / kModuleRadius),
                            Math.min(
                                    kMaxAngularAcceleration.get(),
                                    kKinematicLimits.maxDriveAcceleration() / kModuleRadius)));
                    if (Util.epsilonEquals(mDrive.getSpeedMetersPerSecond(), 0.0)) {
                        mStationaryRotationController.setPID(
                                kStationaryRotationKp.get(), kStationaryRotationKi.get(), kStationaryRotationKd.get());
                    } else {
                        mMovingRotationController.setPID(
                                kMovingRotationKp.get(), kMovingRotationKi.get(), kMovingRotationKd.get());
                    }
                },
                kMaxAngularVelocity,
                kMaxAngularAcceleration,
                kStationaryRotationKp,
                kStationaryRotationKi,
                kStationaryRotationKd);
    }

    @Override
    public void execute() {
        var currentPose = mRobotState.getPose2d();
        var fieldRelativeSpeeds = mFieldRelativeSpeeds.get();
        var distanceToSpeaker = mRobotState.getDistanceToSpeaker();
        var droppedVelocity =
                ShooterUtil.calculateSpeakerSpeed(distanceToSpeaker) * Constants.Shooter.kRollerSpeedToNoteSpeed;
        var robotVelocityTowardsSpeaker = fieldRelativeSpeeds
                .rotateBy(mRobotState.getSpeakerHeading().unaryMinus())
                .getX();
        var timeInAir = distanceToSpeaker / (robotVelocityTowardsSpeaker + droppedVelocity);
        var endTranslation = new Translation2d(
                currentPose.getX() + fieldRelativeSpeeds.getX() * timeInAir,
                currentPose.getY() + fieldRelativeSpeeds.getY() * timeInAir);

        var shooterSetpoint = mTuningEnabled.get()
                ? new ShooterSetpoint(
                        Constants.Shooter.kTunableShooterSpeedRadiansPerSecond.get(),
                        Rotation2d.fromRadians(Constants.Shooter.kTunableShooterAngleRadians.get()))
                : ShooterUtil.calculateShooterSetpoint(FieldUtil.getDistanceToSpeaker(endTranslation));

        mShooter.setSetpoint(shooterSetpoint);

        var targetHeading = mRobotState
                .getSpeakerPose()
                .toTranslation2d()
                .minus(endTranslation)
                .getAngle()
                .minus(shooterSetpoint.releaseAngle());
        var headingError = currentPose.getRotation().minus(targetHeading);

        // headingTolerance = Rotation2d.fromDegrees(0.75);
        headingTolerance = mRobotState.getSpeakerHeadingTolerance();

        Rotation2d setpoint;
        double rotationalVelocity;
        if (GeometryUtil.isNear(GeometryUtil.kRotationIdentity, headingError, headingTolerance.times(0.95))
                && Util.epsilonEquals(fieldRelativeSpeeds.getX(), 0)
                && Util.epsilonEquals(fieldRelativeSpeeds.getY(), 0)) {
            rotationalVelocity = 0;
            mRotationState = kZeroState;
            setpoint = targetHeading;
        } else {
            var rotationPidOutput = getPIDController().calculate(headingError.getRadians(), 0);
            mRotationState = mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, kZeroState);
            rotationalVelocity = mRotationState.velocity + rotationPidOutput;
            setpoint = Rotation2d.fromRadians(targetHeading.getRadians() + mRotationState.position);
        }

        mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds.getX(), fieldRelativeSpeeds.getY(), rotationalVelocity, currentPose.getRotation()));

        var atAngle = GeometryUtil.isNear(
                mShooter.getAngle(), mLastSetpoint.angle(), Rotation2d.fromRadians(kAngleToleranceRadians.get()));

        var stationaryTargetHeading = FieldUtil.getHeadingToSpeaker(currentPose);

        var atHeading = GeometryUtil.isNear(mLastTargetHeading, mRobotState.getHeading(), headingTolerance)
                || GeometryUtil.isNear(stationaryTargetHeading, mRobotState.getHeading(), headingTolerance);

        var atSpeed = mLastSetpoint
                .speeds()
                .allMatch(mShooter.getRollerSpeedsRadiansPerSecond(), kSpeedToleranceRadiansPerSecond.get());

        var atAcceptableRobotSpeed = mRobotState.getHorizontalToSpeaker() < Constants.Drive.kShootWhileMoveDistanceCap
                || mDrive.getSpeedMetersPerSecond() < Constants.Drive.kShootWhileMoveSpeedCap;

        if (mLockedReadyToShoot.update(atAngle && atHeading && atSpeed, Timer.getFPGATimestamp())
                && atAcceptableRobotSpeed) {
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

        mLastSetpoint = shooterSetpoint;
        mLastTargetHeading = targetHeading;

        mRobotState.setShootingState(new ShootingState(shooterSetpoint, true, atAngle, atSpeed, atHeading, mShooting));

        Logger.recordOutput(kLoggingPrefix + "Setpoint", new Pose2d(endTranslation, setpoint));
        Logger.recordOutput(kLoggingPrefix + "TimeInAir", timeInAir);
        Logger.recordOutput(kLoggingPrefix + "VelocityTowardsSpeaker", robotVelocityTowardsSpeaker);
        Logger.recordOutput(kLoggingPrefix + "TargetHeading", targetHeading);
        Logger.recordOutput(kLoggingPrefix + "AtRobotSpeed", atAcceptableRobotSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        mShooting = false;

        mRobotState.setShootingState(ShootingState.kDefault);
        mShooter.stop();
        mIndexer.stop();
        mDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return mEndAfterShooting && mShooting && !mRobotState.hasNote();
    }

    public PIDController getPIDController() {
        return Util.epsilonEquals(mFieldRelativeSpeeds.get().getNorm(), 0.0)
                ? mStationaryRotationController
                : mMovingRotationController;
    }
}
