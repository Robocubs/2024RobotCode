package com.team1701.robot.commands;

import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShootCommands {
    public static Command alwaysAngleTowardsShooter(Shooter shooter, RobotState robotState) {
        return new IdleShooterCommand(shooter, robotState);
    }

    public static Command shoot(Shooter shooter, Indexer indexer, RobotState robotState) {
        return new Shoot(shooter, indexer, robotState, false);
    }

    public static Command aimAndShoot(Shooter shoot, Indexer indexer, Drive drive, RobotState robotState) {
        return Commands.race(
                        new Shoot(shoot, indexer, robotState, true),
                        DriveCommands.rotateToSpeaker(
                                drive, robotState, Constants.Drive.kFastTrapezoidalKinematicLimits, false))
                .withName("AimAndShoot");
    }
}
