package com.team1701.robot.commands;

import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSpeeds;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ManualShoot extends Command {
    private static final String kLoggingPrefix = "Command/ManualShoot/";

    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;

    private boolean mShooting;

    public ManualShoot(Shooter shooter, Indexer indexer, RobotState robotState) {
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
        ShooterSpeeds targetSpeeds;
        targetSpeeds = ShooterUtil.calculateStationaryRollerSpeeds(mRobotState);

        mShooter.setRollerSpeeds(targetSpeeds);

        var atSpeed = targetSpeeds.allMatch(mShooter.getRollerSpeedsRadiansPerSecond(), 50.0);

        if (atSpeed) {
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

        Logger.recordOutput(kLoggingPrefix + "Shooting", mShooting);
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
        return mShooting;
    }
}
