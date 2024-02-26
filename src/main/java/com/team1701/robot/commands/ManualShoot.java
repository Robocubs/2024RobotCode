package com.team1701.robot.commands;

import java.util.stream.DoubleStream;

import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState.ScoringMode;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ManualShoot extends Command {
    private static final String kLoggingPrefix = "Command/ManualShoot/";
    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.01);
    private static final LoggedTunableNumber kHeadingToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "HeadingToleranceRadians", 0.01);

    private final Shooter mShooter;
    private final Indexer mIndexer;

    private boolean mShooting;

    private ScoringMode mScoringMode;

    public ManualShoot(Shooter shooter, Indexer indexer) {
        mShooter = shooter;
        mIndexer = indexer;

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        mShooting = false;
    }

    @Override
    public void execute() {
        Rotation2d desiredShooterAngle;
        Rotation2d targetHeading;

        double leftTargetSpeed;
        double rightTargetSpeed; // if we want to induce spin
        leftTargetSpeed = Constants.Shooter.kTargetShootSpeedRadiansPerSecond.get();
        rightTargetSpeed = leftTargetSpeed;

        mShooter.setUnifiedRollerSpeed(leftTargetSpeed);

        // TODO: Determine if time-locked boolean is needed
        // Or alternatively use a speed range based on distance
        var atSpeed = DoubleStream.of(mShooter.getLeftRollerSpeedsRadiansPerSecond())
                        .allMatch(actualSpeed -> MathUtil.isNear(leftTargetSpeed, actualSpeed, 50.0))
                && DoubleStream.of(mShooter.getRightRollerSpeedsRadiansPerSecond())
                        .allMatch(actualSpeed -> MathUtil.isNear(rightTargetSpeed, actualSpeed, 50.0));

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
