package com.team1701.robot.controls;

import com.team1701.robot.commands.ArmCommands;
import com.team1701.robot.commands.IntakeCommands;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class DashboardControls {
    private LoggedDashboardBoolean stopIntake = new LoggedDashboardBoolean("Controls/StopIntake");

    private LoggedDashboardBoolean reverseIntakeAndIndexer = new LoggedDashboardBoolean("Controls/Reverse");

    private LoggedDashboardBoolean moveArm = new LoggedDashboardBoolean("Controls/ArmUp");

    private LoggedDashboardBoolean retractArm = new LoggedDashboardBoolean("Controls/RetractArm");

    public void bindCommands(Intake intake, Indexer indexer) {
        new Trigger(stopIntake::get).whileTrue(IntakeCommands.stop(intake));
        new Trigger(reverseIntakeAndIndexer::get).whileTrue(IntakeCommands.reverse(intake, indexer));
        new Trigger(moveArm::get)
                .whileTrue(Commands.sequence(ArmCommands.moveToAmp(), Commands.runOnce(() -> moveArm.set(false)))
                        .withName("MoveArmToAmp"));
        new Trigger(retractArm::get)
                .whileTrue(Commands.sequence(ArmCommands.retractArm(), Commands.runOnce(() -> retractArm.set(false)))
                        .withName("RetractArm"));
    }
}
