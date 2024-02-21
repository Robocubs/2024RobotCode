package com.team1701.robot.commands;

import com.team1701.robot.Constants;
import com.team1701.robot.Constants.ScoringMode;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.arm.Arm;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShootCommands {
    public static Command idleShooterCommand(Shooter shooter, RobotState robotState, ScoringMode scoringMode) {
        return new IdleShooterCommand(shooter, robotState, scoringMode);
    }

    public static Command shoot(Shooter shooter, Indexer indexer, RobotState robotState, ScoringMode scoringMode) {
        return new Shoot(shooter, indexer, robotState, false, scoringMode);
    }

    public static Command aimAndShootInSpeaker(Shooter shooter, Indexer indexer, Drive drive, RobotState robotState) {
        return Commands.race(
                        new Shoot(shooter, indexer, robotState, true, ScoringMode.SPEAKER),
                        DriveCommands.rotateToSpeaker(
                                drive, robotState, Constants.Drive.kFastTrapezoidalKinematicLimits, false))
                .withName("AimAndShootInSpeaker");
    }

    public static Command scoreInAmp(Shooter shooter, Indexer indexer, Drive drive, Arm arm, RobotState robotState) {
        return Commands.sequence(
                        new PositionArm(arm, true, ScoringMode.AMP),
                        new Shoot(shooter, indexer, robotState, false, ScoringMode.AMP),
                        new PositionArm(arm, true, ScoringMode.AMP))
                .withName("scoreInAmp");
    }
}
