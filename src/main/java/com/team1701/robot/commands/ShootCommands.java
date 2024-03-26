package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;

import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShootCommands {
    public static Command idleShooterCommand(Shooter shooter, RobotState robotState) {
        return Commands.runEnd(
                () -> shooter.setSetpoint(ShooterUtil.calculateIdleSetpoint(robotState)),
                shooter::stopRotation,
                shooter);
    }

    public static Command stop(Shooter shooter) {
        return Commands.run(shooter::stop, shooter).withName("StopShooter");
    }

    public static Command shoot(Shooter shooter, Indexer indexer, RobotState robotState) {
        return new Shoot(shooter, indexer, robotState, false);
    }

    public static Command shoot(Shooter shooter, Indexer indexer, RobotState robotState, boolean waitForHeading) {
        return new Shoot(shooter, indexer, robotState, waitForHeading);
    }

    public static Command manualShoot(Shooter shooter, Indexer indexer) {
        return new ManualShoot(shooter, indexer).withName("ManualShoot");
    }

    public static Command aimAndShootInSpeaker(Shooter shooter, Indexer indexer, Drive drive, RobotState robotState) {
        return Commands.race(
                        new Shoot(shooter, indexer, robotState, true),
                        DriveCommands.rotateToSpeaker(
                                drive, robotState, Constants.Drive.kFastTrapezoidalKinematicLimits, false))
                .withName("AimAndShootInSpeaker");
    }

    public static Command scoreInAmp(Shooter shooter, Indexer indexer, Drive drive, RobotState robotState) {
        return Commands.sequence(new Shoot(shooter, indexer, robotState, false)).withName("scoreInAmp");
    }

    public static Command passANote(
            Drive drive,
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            DoubleSupplier throttleSupplier,
            DoubleSupplier strafeSupplier) {
        return Commands.deadline(
                        new PassANote(shooter, indexer, robotState),
                        DriveCommands.rotateToPassTarget(
                                drive,
                                throttleSupplier,
                                strafeSupplier,
                                robotState,
                                Constants.Drive.kFastTrapezoidalKinematicLimits))
                .withName("PassANote");
    }
}
