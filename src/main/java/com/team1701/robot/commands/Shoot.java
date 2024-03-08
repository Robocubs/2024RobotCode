package com.team1701.robot.commands;

import java.util.stream.DoubleStream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.RobotState.ScoringMode;
import com.team1701.robot.states.ShootingState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
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
    private ScoringMode mScoringMode;

    public Shoot(
            Shooter shooter, Indexer indexer, RobotState robotState, boolean waitForHeading, ScoringMode scoringMode) {
        mShooter = shooter;
        mIndexer = indexer;
        mRobotState = robotState;
        mWaitForHeading = waitForHeading;

        mScoringMode = scoringMode;

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
        Rotation2d desiredShooterAngle;
        Rotation2d targetHeading;

        double upperTargetSpeed;
        double lowerTargetSpeed; // if we want to induce spin

        switch (mScoringMode) {
            case SPEAKER:
                // desiredShooterAngle = new Rotation2d(Constants.Shooter.kTunableShooterAngleRadians.get());
                desiredShooterAngle =
                        mRobotState.calculateShooterAngleTowardsSpeaker().minus(Rotation2d.fromDegrees(1));

                // upperTargetSpeed = Constants.Shooter.kTargetShootSpeedRadiansPerSecond.get();

                upperTargetSpeed = Constants.Shooter.kShooterSpeedInterpolator.get(mRobotState.getDistanceToSpeaker());
                lowerTargetSpeed = upperTargetSpeed;

                Logger.recordOutput(kLoggingPrefix + "InterpolatedShooterSpeed", upperTargetSpeed);

                targetHeading = mRobotState.getSpeakerHeading();
                break;
            case AMP:
                desiredShooterAngle = Rotation2d.fromDegrees(Constants.Shooter.kShooterAmpAngleDegrees.get());
                upperTargetSpeed = Constants.Shooter.kUpperAmpSpeed.get();
                lowerTargetSpeed = Constants.Shooter.kLowerAmpSpeed.get();
                targetHeading = mRobotState.getAmpHeading();
                break;
            default:
                cancel();
                return;
        }

        // desiredShooterAngle = Rotation2d.fromRadians(Constants.Shooter.kTunableShooterAngleRadians.get());
        var clampedDesiredRotations = MathUtil.clamp(
                desiredShooterAngle.getRotations(),
                Constants.Shooter.kShooterLowerLimitRotations,
                Constants.Shooter.kShooterUpperLimitRotations);

        mShooter.setRotationAngle(Rotation2d.fromRotations(clampedDesiredRotations));
        mShooter.setUpperRollerSpeeds(upperTargetSpeed);
        mShooter.setLowerRollerSpeeds(lowerTargetSpeed);

        var atAngle = GeometryUtil.isNear(
                mShooter.getAngle(), desiredShooterAngle, Rotation2d.fromRadians(kAngleToleranceRadians.get()));

        var atHeading = !mWaitForHeading
                || GeometryUtil.isNear(
                        targetHeading,
                        mRobotState.getHeading(),
                        Rotation2d.fromDegrees(kHeadingToleranceRadians.get()));

        var atSpeed = DoubleStream.of(mShooter.getUpperRollerSpeedsRadiansPerSecond())
                        .allMatch(actualSpeed ->
                                MathUtil.isNear(upperTargetSpeed, actualSpeed, kSpeedToleranceRadiansPerSecond.get()))
                && DoubleStream.of(mShooter.getLowerRollerSpeedsRadiansPerSecond())
                        .allMatch(actualSpeed -> MathUtil.isNear(lowerTargetSpeed, actualSpeed, 50.0));

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
