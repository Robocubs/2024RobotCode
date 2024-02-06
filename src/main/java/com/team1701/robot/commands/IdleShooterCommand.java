package com.team1701.robot.commands;

import java.util.stream.DoubleStream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.shooter.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

// Default shooter command which consistently updates shooter angle to always point towards the speaker
public class IdleShooterCommand extends Command {
    private static final String kLoggingPrefix = "Command/AlwaysAngleTowardsShooter/";

    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.01);
    private static final LoggedTunableNumber kIdleSpeedRadiansPerSecond =
            new LoggedTunableNumber(kLoggingPrefix + "IdleSpeedRadiansPerSecond", 300);

    private final Shooter mShooter;
    private final RobotState mRobotState;

    public IdleShooterCommand(Shooter shooter, RobotState robotState) {
        mShooter = shooter;
        mRobotState = robotState;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        Logger.recordOutput(kLoggingPrefix + "SuccessfullyInitialized", true);
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
        mShooter.setRollerSpeed(kIdleSpeedRadiansPerSecond.get());

        var atAngle = GeometryUtil.isNear(
                mShooter.getAngle(), shooterAngleFromHorizontal, Rotation2d.fromRadians(kAngleToleranceRadians.get()));
        var atSpeed = DoubleStream.of(mShooter.getRollerSpeedsRadiansPerSecond())
                .allMatch(actualSpeed -> MathUtil.isNear(kIdleSpeedRadiansPerSecond.get(), actualSpeed, 10.0));

        Logger.recordOutput(kLoggingPrefix + "AtAngle", atAngle);
        Logger.recordOutput(kLoggingPrefix + "AtSpeed", atSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopRotation();
        Logger.recordOutput(kLoggingPrefix + "EndDefaultCommand", interrupted);
    }
}
