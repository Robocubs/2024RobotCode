package com.team1701.robot.commands;

import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.ShootingState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSetpoint;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ManualShoot extends Command {
    private static final ShooterSpeeds kManualShootSpeed = new ShooterSpeeds(400);

    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;

    private Rotation2d mAngle;
    private boolean mShooting;

    public ManualShoot(Shooter shooter, Indexer indexer, RobotState robotState) {
        mShooter = shooter;
        mIndexer = indexer;
        mRobotState = robotState;

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        mAngle = mShooter.getAngle();
        mShooting = false;
    }

    @Override
    public void execute() {
        var setpoint = new ShooterSetpoint(kManualShootSpeed, mAngle);
        mShooter.setSetpoint(setpoint);

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

        mRobotState.setShootingState(new ShootingState(setpoint, true, true, atSpeed, true, mShooting));
    }

    @Override
    public void end(boolean interrupted) {
        mShooting = false;

        mRobotState.setShootingState(ShootingState.kDefault);
        mShooter.stop();
        mIndexer.stop();
    }
}
