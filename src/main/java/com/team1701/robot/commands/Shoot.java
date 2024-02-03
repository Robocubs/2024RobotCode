package com.team1701.robot.commands;

import java.util.stream.DoubleStream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class Shoot extends Command {
    private static final String kLoggingPrefix = "Command/Shoot/";
    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.01);
    private static final LoggedTunableNumber kHeadingToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "HeadingToleranceRadians", 0.01);
    private static final LoggedTunableNumber kTargetSpeedRadiansPerSecond =
            new LoggedTunableNumber(kLoggingPrefix + "TargetSpeedRadiansPerSecond", 600);

    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;
    private final boolean mWaitForHeading;

    private boolean mShooting;

    public Shoot(Shooter shooter, Indexer indexer, RobotState robotState, boolean waitForHeading) {
        mShooter = shooter;
        mIndexer = indexer;
        mRobotState = robotState;
        mWaitForHeading = waitForHeading;

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        mShooting = false;
    }

    @Override
    public void execute() {
        var distanceToTarget = mRobotState
                .getPose2d()
                .getTranslation()
                .getDistance(mRobotState.getSpeakerPose().toTranslation2d());
        var shooterAngleFromHorizontal = new Rotation2d(
                distanceToTarget - Constants.Shooter.kShooterAxisOffset,
                FieldConstants.kSpeakerHeight - Constants.Shooter.kShooterAxisHeight);

        mShooter.setRotationAngle(shooterAngleFromHorizontal);

        // TODO: Determine linear regression of speeds
        var targetSpeed = kTargetSpeedRadiansPerSecond.get();
        mShooter.setRollerSpeed(targetSpeed);

        var atAngle = GeometryUtil.isNear(
                mShooter.getAngle(), shooterAngleFromHorizontal, Rotation2d.fromRadians(kAngleToleranceRadians.get()));

        var atHeading = !mWaitForHeading
                || GeometryUtil.isNear(
                        mRobotState.getSpeakerHeading(),
                        mRobotState.getHeading(),
                        Rotation2d.fromRadians(kHeadingToleranceRadians.get()));

        // TODO: Determine if time-locked boolean is needed
        // Or alternatively use a speed range based on distance
        var atSpeed = DoubleStream.of(mShooter.getRollerSpeedsRadiansPerSecond())
                .allMatch(actualSpeed -> MathUtil.isNear(targetSpeed, actualSpeed, 10.0));

        if (atAngle && atHeading && atSpeed) {
            mIndexer.setForwardShoot();
            mShooting = true;
        }

        if (!mShooting) {
            if (mIndexer.noteIsLoaded()) {
                mIndexer.stop();
            } else {
                mIndexer.setForwardLoad();
            }
        }

        Logger.recordOutput(kLoggingPrefix + "TargetShooterAngle", shooterAngleFromHorizontal);
        Logger.recordOutput(kLoggingPrefix + "Shooting", mShooting);
        Logger.recordOutput(kLoggingPrefix + "AtAngle", atAngle);
        Logger.recordOutput(kLoggingPrefix + "AtHeading", atHeading);
        Logger.recordOutput(kLoggingPrefix + "AtSpeed", atSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopRollers();
        mShooter.stopRotation();
        mIndexer.stop();
    }

    @Override
    public boolean isFinished() {
        return mShooting && !mRobotState.hasNote();
    }
}
