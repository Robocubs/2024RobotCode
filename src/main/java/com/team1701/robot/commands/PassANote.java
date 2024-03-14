package com.team1701.robot.commands;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class PassANote extends Command {
    private static final String kLoggingPrefix = "Command/PassANote/";
    private static final LoggedTunableNumber kPassingTunableAngleRadians =
            new LoggedTunableNumber(kLoggingPrefix + "PassingTunableAngle", .4);
    private static final LoggedTunableNumber kPassingTunableSpeeds =
            new LoggedTunableNumber(kLoggingPrefix + "PassingTunableSpeeds", 400);

    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.01);

    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;

    private boolean mShooting;

    public PassANote(Shooter shooter, Indexer indexer, RobotState robotState) {
        mShooter = shooter;
        mIndexer = indexer;
        mRobotState = robotState;

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        mShooting = false;
    }

    @Override
    public void execute() {
        var targetSpeeds = ShooterUtil.calculatePassingShooterSpeeds(mRobotState);
        mShooter.setRollerSpeeds(targetSpeeds);

        var targetAngle = new Rotation2d(kPassingTunableAngleRadians.get());
        mShooter.setRotationAngle(targetAngle);

        var atSpeed = targetSpeeds.allMatch(mShooter.getRollerSpeedsRadiansPerSecond(), 50.0);

        var atAngle = GeometryUtil.isNear(
                mShooter.getAngle(), targetAngle, Rotation2d.fromRadians(kAngleToleranceRadians.get()));

        var atHeading = GeometryUtil.isNear(
                mRobotState.getPassingHeading(), mRobotState.getHeading(), Constants.Shooter.kPassingHeadingTolerance);

        var atPose = mRobotState.getPassingDistance() > 6 && !mRobotState.inOpponentWing();

        if (atSpeed && atAngle && atHeading && atPose) {
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

        Logger.recordOutput(kLoggingPrefix + "AtAngle", atAngle);
        Logger.recordOutput(kLoggingPrefix + "AtHeading", atHeading);
        Logger.recordOutput(kLoggingPrefix + "AtPose", atPose);
        Logger.recordOutput(kLoggingPrefix + "Shooting", mShooting);
        Logger.recordOutput(kLoggingPrefix + "AtSpeed", atSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopRollers();
        mShooter.stopRotation();
        mIndexer.stop();
    }
}
