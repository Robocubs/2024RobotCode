package com.team1701.robot.commands;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.*;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSpeeds;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class IdleShooterCommand extends Command {
    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;
    private final Drive mDrive;

    public IdleShooterCommand(Shooter shooter, Indexer indexer, Drive drive, RobotState robotState) {
        mShooter = shooter;
        mIndexer = indexer;
        mRobotState = robotState;
        mDrive = drive;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        ShooterSpeeds shooterSpeeds;

        Rotation2d desiredShooterAngle = mIndexer.hasNoteAtExit()
                ? ShooterUtil.calculateStationaryDesiredAngle(mRobotState)
                : Constants.Shooter.kLoadingAngle;

        var clampedDesiredShooterAngle = GeometryUtil.clampRotation(
                desiredShooterAngle, Constants.Shooter.kShooterLowerLimit, Constants.Shooter.kShooterUpperLimit);

        mShooter.setRotationAngle(clampedDesiredShooterAngle);

        shooterSpeeds = ShooterUtil.calculateIdleRollerSpeeds(mRobotState, mDrive);
        mShooter.setRollerSpeeds(shooterSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopRotation();
    }
}
