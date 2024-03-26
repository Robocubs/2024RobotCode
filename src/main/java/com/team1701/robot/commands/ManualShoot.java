package com.team1701.robot.commands;

import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ManualShoot extends Command {
    private static final String kLoggingPrefix = "Command/ManualShoot/";
    private static final ShooterSpeeds kManualShootSpeed = new ShooterSpeeds(400);

    private final Shooter mShooter;
    private final Indexer mIndexer;

    private Rotation2d mAngle;
    private boolean mShooting;

    public ManualShoot(Shooter shooter, Indexer indexer) {
        mShooter = shooter;
        mIndexer = indexer;

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        mAngle = mShooter.getAngle();
        mShooting = false;
    }

    @Override
    public void execute() {
        mShooter.setRollerSpeeds(kManualShootSpeed);
        mShooter.setRotationAngle(mAngle);

        var atSpeed = kManualShootSpeed.allMatch(mShooter.getRollerSpeedsRadiansPerSecond(), 50.0);
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
        mShooting = false;
        mShooter.stop();
        mIndexer.stop();
    }
}
