package com.team1701.robot.commands;

import java.util.function.Supplier;
import java.util.stream.DoubleStream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.Constants.ScoringMode;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.shooter.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

// Default shooter command which consistently updates shooter angle to always point towards the speaker
public class IdleShooterCommand extends Command {
    private static final String kLoggingPrefix = "Command/IdleShooterCommand/";

    private static final LoggedTunableNumber kAngleToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "AngleToleranceRadians", 0.01);

    private final Shooter mShooter;
    private final RobotState mRobotState;
    private final Supplier<ScoringMode> mScoringMode;

    public IdleShooterCommand(Shooter shooter, RobotState robotState, Supplier<ScoringMode> scoringMode) {
        mShooter = shooter;
        mRobotState = robotState;
        mScoringMode = scoringMode;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Rotation2d desiredShooterAngle;
        double shooterSpeed;

        switch (mScoringMode.get()) {
            case SPEAKER:
                // TODO: ramp up speeds on approach
                desiredShooterAngle = mRobotState.calculateShooterAngleTowardsSpeaker();
                shooterSpeed = Constants.Shooter.kIdleSpeedRadiansPerSecond.get();
                break;
            case AMP:
                desiredShooterAngle = Rotation2d.fromDegrees(Constants.Shooter.kShooterAmpAngleDegrees.get());
                shooterSpeed = Constants.Shooter.kAmpRollerSpeedRadiansPerSecond.get();
                break;
            case CLIMB:
                desiredShooterAngle = Rotation2d.fromDegrees(85);
                shooterSpeed = Constants.Shooter.kTrapRollerSpeedRadiansPerSecond.get();
                break;
            default:
                desiredShooterAngle = mRobotState.calculateShooterAngleTowardsSpeaker();
                shooterSpeed = Constants.Shooter.kIdleSpeedRadiansPerSecond.get();
                break;
        }

        mShooter.setRotationAngle(desiredShooterAngle);
        mShooter.setRollerSpeed(shooterSpeed);

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
