package com.team1701.robot.commands;

import java.util.stream.DoubleStream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
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
    private static final LoggedTunableNumber kRightTargetSpeedRadiansPerSecond =
            new LoggedTunableNumber(kLoggingPrefix + "TargetSpeedRadiansPerSecond/Right", 600);
    private static final LoggedTunableNumber kLeftTargetSpeedRadiansPerSecond =
            new LoggedTunableNumber(kLoggingPrefix + "TargetSpeedRadiansPerSecond/Left", 600);

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
        var shooterAngleFromHorizontal = mRobotState.calculateShooterAngleTowardsSpeaker();

        mShooter.setRotationAngle(shooterAngleFromHorizontal);

        // TODO: Determine linear regression of speeds
        var leftTargetSpeed = kLeftTargetSpeedRadiansPerSecond.get();
        var rightTargetSpeed = kRightTargetSpeedRadiansPerSecond.get();
        mShooter.setLeftRollerSpeeds(leftTargetSpeed);
        mShooter.setRightRollerSpeeds(rightTargetSpeed);

        var atAngle = GeometryUtil.isNear(
                mShooter.getAngle(), shooterAngleFromHorizontal, Rotation2d.fromRadians(kAngleToleranceRadians.get()));

        var atHeading = !mWaitForHeading
                || GeometryUtil.isNear(
                        mRobotState.getSpeakerHeading(),
                        mRobotState.getHeading(),
                        Rotation2d.fromRadians(kHeadingToleranceRadians.get()));

        // TODO: Determine if time-locked boolean is needed
        // Or alternatively use a speed range based on distance
        var atSpeed = DoubleStream.of(mShooter.getLeftRollerSpeedsRadiansPerSecond())
                        .allMatch(actualSpeed -> MathUtil.isNear(leftTargetSpeed, actualSpeed, 10.0))
                && DoubleStream.of(mShooter.getRightRollerSpeedsRadiansPerSecond())
                        .allMatch(actualSpeed -> MathUtil.isNear(rightTargetSpeed, actualSpeed, 10.0));

        if (atAngle && atHeading && atSpeed) {
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
