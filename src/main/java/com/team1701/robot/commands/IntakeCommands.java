package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
            Intake intake, Indexer indexer, Drive drive, Supplier<Rotation2d> robotHeadingSupplier) {
        return Commands.startEnd(
                        () -> {
                            intake.setReverse();
                            indexer.setReverse();
                            drive.setVelocity(
                                    ChassisSpeeds.fromRobotRelativeSpeeds(-.75, 0, 0, robotHeadingSupplier.get()));
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
        return Commands.startEnd(
                        () -> {
                            intake.stop();
                            indexer.stop();
                        },
                        () -> {},
                        intake,
                        indexer)
                .withName("StopIntaking");
    }

    public static Command defaultCommand(Intake intake, RobotState mRobotState) {
        return new IntakeCommand(intake, mRobotState);
    }
}
