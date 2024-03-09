package com.team1701.robot.commands;

import java.util.stream.DoubleStream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.lib.util.ShooterUtil;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class IdleShooterCommand extends Command {
    private static final String kLoggingPrefix = "Command/IdleShooterCommand/";

    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.01);

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
        double[] shooterSpeeds;

        Rotation2d desiredShooterAngle = !mIndexer.hasNote()
                ? Rotation2d.fromDegrees(18)
                : ShooterUtil.calculateStationaryDesiredAngle(mRobotState);

        var clampedDesiredRotations = MathUtil.clamp(
                desiredShooterAngle.getRotations(),
                Constants.Shooter.kShooterLowerLimitRotations,
                Constants.Shooter.kMaxAngleDegrees.get());

        mShooter.setRotationAngle(Rotation2d.fromRotations(clampedDesiredRotations));

        shooterSpeeds = ShooterUtil.calculateIdleRollerSpeeds(mRobotState, mDrive);
        mShooter.setRollerSpeeds(shooterSpeeds);

        var atAngle = GeometryUtil.isNear(
                mShooter.getAngle(), desiredShooterAngle, Rotation2d.fromRadians(kAngleToleranceRadians.get()));
        var atSpeed = DoubleStream.of(mShooter.getRollerSpeedsRadiansPerSecond())
                .allMatch(actualSpeed ->
                        MathUtil.isNear(Constants.Shooter.kIdleSpeedRadiansPerSecond.get(), actualSpeed, 10.0));

        Logger.recordOutput(kLoggingPrefix + "AtAngle", atAngle);
        Logger.recordOutput(kLoggingPrefix + "AtSpeed", atSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopRotation();
    }
}
