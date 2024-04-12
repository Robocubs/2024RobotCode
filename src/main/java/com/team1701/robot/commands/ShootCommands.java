package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.ShootingState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

public class ShootCommands {
    public static Command idleShooterCommand(Shooter shooter, RobotState robotState) {
        return Commands.runEnd(
                        () -> shooter.setSetpoint(ShooterUtil.calculateIdleSetpoint(robotState)),
                        shooter::stopRotation,
                        shooter)
                .withName("IdleShooterCommand");
    }

    public static Command stop(Shooter shooter) {
        return Commands.run(shooter::stop, shooter).withName("StopShooter");
    }

    public static Command shoot(Shooter shooter, Indexer indexer, RobotState robotState) {
        return new Shoot(shooter, indexer, robotState, true);
    }

    public static Command forceShoot(Shooter shooter, Indexer indexer, RobotState robotState) {
        return new Shoot(shooter, indexer, robotState, false, false, false).withName("ForceShoot");
    }

    public static Command manualShoot(Shooter shooter, Indexer indexer, RobotState robotState) {
        return new ManualShoot(shooter, indexer, robotState).withName("ManualShoot");
    }

    public static Command aimAndShootInSpeaker(Shooter shooter, Indexer indexer, Drive drive, RobotState robotState) {
        return shoot(shooter, indexer, robotState)
                .deadlineWith(DriveCommands.rotateToSpeaker(
                        drive, robotState, Constants.Drive.kFastTrapezoidalKinematicLimits, false))
                .withName("AimAndShootInSpeaker");
    }

    public static Command scoreInAmp(Shooter shooter, Indexer indexer, RobotState robotState) {
        return new Shoot(shooter, indexer, robotState, false);
    }

    public static Command passLow(
            Drive drive,
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            DoubleSupplier throttleSupplier,
            DoubleSupplier strafeSupplier) {
        return Commands.deadline(
                DriveCommands.rotateToPassTarget(
                        drive,
                        throttleSupplier,
                        strafeSupplier,
                        robotState,
                        Constants.Drive.kFastTrapezoidalKinematicLimits),
                runEnd(
                        () -> {
                            var setpoint = Constants.Shooter.kLowPassSetpoint;
                            var shooting = false;
                            shooter.setSetpoint(setpoint);
                            var atSpeed = setpoint.speeds().allMatch(shooter.getRollerSpeedsRadiansPerSecond(), 50.0);
                            var atAngle = GeometryUtil.isNear(
                                    shooter.getAngle(), setpoint.angle(), Rotation2d.fromRadians(.02));
                            var atHeading = GeometryUtil.isNear(
                                    robotState.getPassingHeading(),
                                    robotState.getHeading(),
                                    Constants.Shooter.kPassingHeadingTolerance);
                            if (atSpeed && atAngle && atHeading) {
                                indexer.setForwardShoot();
                                shooting = true;
                            }
                            if (!shooting) {
                                if (indexer.hasNoteAtExit()) {
                                    indexer.stop();
                                } else {
                                    indexer.setForwardLoad();
                                }
                            }
                            robotState.setShootingState(
                                    new ShootingState(setpoint, true, atAngle, atSpeed, atHeading, shooting));
                        },
                        () -> {
                            shooter.stop();
                            indexer.stop();
                        },
                        shooter,
                        indexer));
    }
}
