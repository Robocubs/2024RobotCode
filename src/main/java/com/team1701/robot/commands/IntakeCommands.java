package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
                            driverController.getHID().setRumble(RumbleType.kLeftRumble, 0.3);
                        },
                        () -> {
                            intake.stop();
                            indexer.stop();
                            driverController.getHID().setRumble(RumbleType.kLeftRumble, 0);
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

    public static Command defaultCommand(Intake intake, CommandXboxController driverController, RobotState mRobotState) {
        return new IntakeCommand(intake, driverController, mRobotState);
    }
}
