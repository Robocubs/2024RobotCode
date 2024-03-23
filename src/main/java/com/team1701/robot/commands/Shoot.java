package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.lib.util.tuning.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.ShootingState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class Shoot extends Command {
    private static final String kLoggingPrefix = "Command/Shoot/";
    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.01);
    private static final LoggedTunableNumber kSpeedToleranceRadiansPerSecond =
            new LoggedTunableNumber(kLoggingPrefix + "SpeedToleranceRadiansPerSecond", 50.0);
    private Rotation2d headingTolerance;

    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final Drive mDrive;
    private final RobotState mRobotState;
    private Supplier<Translation2d> mFieldRelativeSpeeds;
    private final boolean mWaitForHeading;
    private final boolean mWaitForSpeed;
    private final boolean mWaitForAngle;

    private TimeLockedBoolean mLockedReadyToShoot;
    private boolean mShooting;

    public Shoot(Shooter shooter, Indexer indexer, Drive drive, RobotState robotState, boolean waitForHeading) {
        this(shooter, indexer, drive, robotState, waitForHeading, true, true);
    }

    public Shoot(
            Shooter shooter,
            Indexer indexer,
            Drive drive,
            RobotState robotState,
            boolean waitForHeading,
            boolean waitForSpeed,
            boolean waitForAngle) {
        mShooter = shooter;
        mDrive = drive;
        mIndexer = indexer;
        mRobotState = robotState;
        mFieldRelativeSpeeds = () -> new Translation2d(
                robotState.getFieldRelativeSpeeds().vxMetersPerSecond,
                robotState.getFieldRelativeSpeeds().vyMetersPerSecond);
        mWaitForHeading = waitForHeading;
        mWaitForSpeed = waitForSpeed;
        mWaitForAngle = waitForAngle;

        mLockedReadyToShoot = new TimeLockedBoolean(.1, Timer.getFPGATimestamp());

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        mShooting = false;
        mLockedReadyToShoot.update(false, Timer.getFPGATimestamp());
    }

    @Override
    public void execute() {
        if (mRobotState.isClimbMode()) {
            cancel();
            return;
        }
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
                        .plus(Rotation2d.fromDegrees(Constants.Shooter.kShooterHeadingOffsetInterpolator.get(
                                mRobotState.getDistanceToSpeaker()))));
        var robotVelocityTowardsSpeaker =
                mDrive.getSpeedMetersPerSecond() * Math.cos(headingAngleFromSpeaker.getRadians());
        var timeInAir = mRobotState.getDistanceToSpeaker() / (robotVelocityTowardsSpeaker + droppedVelocity);
        var endTranslation = new Translation2d(
                currentPose.getX() + fieldRelativeSpeeds.getX() * timeInAir,
                currentPose.getY() + fieldRelativeSpeeds.getY() * timeInAir);

        Rotation2d targetHeading = mRobotState.getStationaryTargetHeading();
        headingTolerance = mRobotState.getToleranceSpeakerHeading();

        Rotation2d desiredShooterAngle = GeometryUtil.clampRotation(
                ShooterUtil.calculateShooterAngleWithMotion(mRobotState, endTranslation),
                Constants.Shooter.kShooterLowerLimit,
                Constants.Shooter.kShooterUpperLimit);

        mShooter.setRotationAngle(desiredShooterAngle);

        var speeds = mRobotState.isAmpMode()
                ? ShooterUtil.calculateStationaryRollerSpeeds(mRobotState)
                : ShooterUtil.calculateShooterSpeedsWithMotion(mRobotState, endTranslation);
        mShooter.setRollerSpeeds(speeds);

        var atAngle = !mWaitForAngle
                || GeometryUtil.isNear(
                        mShooter.getAngle(), desiredShooterAngle, Rotation2d.fromRadians(kAngleToleranceRadians.get()));

        var atHeading =
                !mWaitForHeading || GeometryUtil.isNear(targetHeading, mRobotState.getHeading(), headingTolerance);

        var atSpeed = !mWaitForSpeed
                || speeds.allMatch(mShooter.getRollerSpeedsRadiansPerSecond(), kSpeedToleranceRadiansPerSecond.get());

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

        Logger.recordOutput(kLoggingPrefix + "TargetShooterAngle", desiredShooterAngle);
        Logger.recordOutput(kLoggingPrefix + "Shooting", mShooting);
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
