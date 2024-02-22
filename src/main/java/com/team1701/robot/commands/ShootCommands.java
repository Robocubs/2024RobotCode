package com.team1701.robot.commands;

import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.RobotState.ScoringMode;
import com.team1701.robot.subsystems.arm.Arm;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShootCommands {
    public static Command idleShooterCommand(Shooter shooter, RobotState robotState) {
        return new IdleShooterCommand(shooter, robotState);
    }

    public static Command shoot(Shooter shooter, Indexer indexer, RobotState robotState) {
        return new Shoot(shooter, indexer, robotState, false, robotState.getScoringMode());
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
                        ArmCommands.positionArm(
                                arm, Rotation2d.fromDegrees(Constants.Arm.kArmAmpRotationDegrees.get())),
                        new Shoot(shooter, indexer, robotState, false, ScoringMode.AMP))
                .withName("scoreInAmp");
    }
}
