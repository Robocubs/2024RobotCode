package com.team1701.robot.commands;

import java.util.stream.DoubleStream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
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

    private static final LoggedTunableNumber kExpectedShootingDistance =
            new LoggedTunableNumber(kLoggingPrefix + "ExpectedShootingDistance", 6.0);

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
        Rotation2d desiredShooterAngle;
        double shooterSpeed;

        switch (mRobotState.getScoringMode()) {
            case SPEAKER:
                // TODO: ramp up speeds on approach
                desiredShooterAngle =
                        mRobotState.calculateShooterAngleTowardsSpeaker().plus(Rotation2d.fromDegrees(2.5));
                if (mDrive.getKinematicLimits().equals(Constants.Drive.kSlowKinematicLimits)) {
                    shooterSpeed = Constants.Shooter.kTargetShootSpeedRadiansPerSecond.get();
                } else {
                    shooterSpeed = mRobotState.hasNote()
                            ? speedRegression(
                                    kExpectedShootingDistance.get(), mRobotState.getDistanceToSpeaker(), 100.0)
                            : Constants.Shooter.kIdleSpeedRadiansPerSecond.get();
                }
                break;
            case AMP:
                desiredShooterAngle = Rotation2d.fromDegrees(Constants.Shooter.kShooterAmpAngleDegrees.get());
                if (mRobotState.hasNote()) {
                    shooterSpeed = mRobotState.getDistanceToAmp() <= 1
                            ? Constants.Shooter.kAmpRollerSpeedRadiansPerSecond.get()
                            : 250;
                } else {
                    shooterSpeed = Constants.Shooter.kIdleSpeedRadiansPerSecond.get();
                }
                break;
            case CLIMB:
                desiredShooterAngle = Rotation2d.fromDegrees(Constants.Shooter.kShooterUpperLimitRotations);
                shooterSpeed = 0;
                break;
            default:
                desiredShooterAngle = mRobotState.calculateShooterAngleTowardsSpeaker();
                shooterSpeed = Constants.Shooter.kIdleSpeedRadiansPerSecond.get();
                break;
        }

        if (!mIndexer.hasNoteAtExit()) {
            desiredShooterAngle = Rotation2d.fromDegrees(18);
        }

        mShooter.setRotationAngle(desiredShooterAngle);
        mShooter.setUnifiedRollerSpeed(shooterSpeed);

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

    // 5 * max is top idle speed
    private double speedRegression(double reference, double current, double factor) {
        var scale = current - reference > 0.2 ? 4 / (1 + Math.abs(reference - current)) : 400 / 100;
        return factor * scale;
    }
}
