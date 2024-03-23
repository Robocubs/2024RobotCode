package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class IntakeCommands {

    public static Command reverse(Intake intake, Indexer indexer) {
        return Commands.startEnd(
                        () -> {
                            intake.setReverse();
                            indexer.setReverse();
                        },
                        () -> {
                            intake.stop();
                            indexer.stop();
                        },
                        intake,
                        indexer)
                .withName("ReverseIntakeAndIndexer");
    }

    public static Command rejectAndDrive(
            Intake intake,
            Indexer indexer,
            Drive drive,
            CommandXboxController driverController,
            Supplier<Rotation2d> robotHeadingSupplier) {
        return Commands.startEnd(
                        () -> {
                            intake.setReverse();
                            indexer.setReverse();
                            drive.setVelocity(new ChassisSpeeds(0.75, 0, 0));
                        },
                        () -> {
                            intake.stop();
                            indexer.stop();
                        },
                        intake,
                        indexer,
                        drive)
                .withName("RejectAndDrive");
    }

    public static Command stopIntake(Intake intake, Indexer indexer) {
        return Commands.run(
                        () -> {
                            intake.stop();
                            indexer.stop();
                        },
                        intake,
                        indexer)
                .withName("StopIntaking");
    }

    public static Command idleIntake(Intake intake, RobotState robotState) {
        return Commands.run(
                        () -> {
                            if (robotState.hasLoadedNote()) {
                                intake.stop();
                            } else {
                                intake.setForward();
                            }
                        },
                        intake)
                .finallyDo(intake::stop)
                .withName("IdleIntake");
    }

    public static Command idleIndexer(Indexer indexer) {
        var recentlyUnloaded = new TimeLockedBoolean(0.05, Timer.getFPGATimestamp(), false, false);
        return Commands.run(
                        () -> {
                            var hasNoteAtExit = indexer.hasNoteAtExit();
                            recentlyUnloaded.update(!hasNoteAtExit, Timer.getFPGATimestamp());
                            if (hasNoteAtExit) {
                                if (recentlyUnloaded.getValue()) {
                                    indexer.setReverseBump();
                                } else {
                                    indexer.stop();
                                }
                            } else if (indexer.hasNoteAtEntrance()) {
                                indexer.setSlowLoad();
                            } else {
                                indexer.setForwardLoad();
                            }
                        },
                        indexer)
                .finallyDo(indexer::stop)
                .withName("IdleIndexer");
    }
}
