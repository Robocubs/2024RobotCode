package com.team1701.robot.commands;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.lib.util.ShooterUtil;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.ShootingState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class Shoot extends Command {
    private static final String kLoggingPrefix = "Command/Shoot/";
    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.01);
    private static final LoggedTunableNumber kSpeedToleranceRadiansPerSecond =
            new LoggedTunableNumber(kLoggingPrefix + "SpeedToleranceRadiansPerSecond", 50.0);
    private static final LoggedTunableNumber kHeadingToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "HeadingToleranceDegrees", 2);

    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;
    private final boolean mWaitForHeading;

    private TimeLockedBoolean mLockedReadyToShoot;
    private boolean mShooting;

    public Shoot(Shooter shooter, Indexer indexer, RobotState robotState, boolean waitForHeading) {
        mShooter = shooter;
        mIndexer = indexer;
        mRobotState = robotState;
        mWaitForHeading = waitForHeading;

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

        Rotation2d targetHeading = mRobotState.getStationaryTargetHeading();

        Rotation2d desiredShooterAngle = GeometryUtil.clampRotation(
                ShooterUtil.calculateStationaryDesiredAngle(mRobotState),
                Constants.Shooter.kShooterLowerLimitRotations,
                Constants.Shooter.kShooterUpperLimitRotations);

        mShooter.setRotationAngle(desiredShooterAngle);

        var speeds = ShooterUtil.calculateStationaryRollerSpeeds(mRobotState);
        mShooter.setRollerSpeeds(speeds);

        var atAngle = GeometryUtil.isNear(
                mShooter.getAngle(), desiredShooterAngle, Rotation2d.fromRadians(kAngleToleranceRadians.get()));

        var atHeading = !mWaitForHeading
                || GeometryUtil.isNear(
                        targetHeading,
                        mRobotState.getHeading(),
                        Rotation2d.fromDegrees(kHeadingToleranceRadians.get()));

        var atSpeed = Util.sequentiallyMatch(
                speeds, mShooter.getRollerSpeedsRadiansPerSecond(), kSpeedToleranceRadiansPerSecond.get());

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
