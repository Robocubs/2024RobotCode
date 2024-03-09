package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;

import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.arm.Arm;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShootCommands {
    public static Command idleShooterCommand(Shooter shooter, Indexer indexer, Drive drive, RobotState robotState) {
        return new IdleShooterCommand(shooter, indexer, drive, robotState);
    }

    public static Command shoot(Shooter shooter, Indexer indexer, RobotState robotState) {
        return new Shoot(shooter, indexer, robotState, false);
    }

    public static Command shoot(Shooter shooter, Indexer indexer, RobotState robotState, boolean waitForHeading) {
        return new Shoot(shooter, indexer, robotState, waitForHeading);
    }

    public static Command manualShoot(Shooter shooter, Indexer indexer, RobotState robotState) {
        return new ManualShoot(shooter, indexer, robotState);
    }

    public static Command aimAndShootInSpeaker(Shooter shooter, Indexer indexer, Drive drive, RobotState robotState) {
        return Commands.race(
                        new Shoot(shooter, indexer, robotState, true),
                        DriveCommands.rotateToSpeaker(
                                drive, robotState, Constants.Drive.kFastTrapezoidalKinematicLimits, false))
                .withName("AimAndShootInSpeaker");
    }

    public static Command scoreInAmp(Shooter shooter, Indexer indexer, Drive drive, Arm arm, RobotState robotState) {
        return Commands.sequence(
                        // ArmCommands.positionArm(
                        //         arm, Rotation2d.fromDegrees(Constants.Arm.kArmAmpRotationDegrees.get())),
                        new Shoot(shooter, indexer, robotState, false))
                .withName("scoreInAmp");
    }

    public static Command shootAndMove(
            Drive drive,
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            DoubleSupplier throttle,
            DoubleSupplier strafe) {
        var maxDriveVelocity = Constants.Drive.kFastSmoothKinematicLimits.maxDriveVelocity();
        return new ShootAndMove(drive, shooter, indexer, robotState, () -> {
            var translationSign = Configuration.isBlueAlliance() ? 1.0 : -1.0;
            return new Translation2d(
                            throttle.getAsDouble() * maxDriveVelocity * translationSign,
                            strafe.getAsDouble() * maxDriveVelocity * translationSign)
                    .rotateBy(drive.getFieldRelativeHeading());
        });
    }
}
