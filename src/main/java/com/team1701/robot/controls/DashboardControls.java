package com.team1701.robot.controls;

import com.team1701.robot.commands.ArmCommands;
import com.team1701.robot.commands.IntakeCommands;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class DashboardControls {
    private LoggedDashboardBoolean stopIntake = new LoggedDashboardBoolean("Controls/StopIntake");

    private LoggedDashboardBoolean reverseIntakeAndIndexer = new LoggedDashboardBoolean("Controls/Reverse");

    private LoggedDashboardBoolean moveArm = new LoggedDashboardBoolean("Controls/ArmUp");

    private LoggedDashboardBoolean retractArm = new LoggedDashboardBoolean("Controls/RetractArm");

    public void bindCommands(Intake intake, Indexer indexer) {
        createTriggeredCommand(stopIntake, IntakeCommands.stop(intake));

        createTriggeredCommand(reverseIntakeAndIndexer, IntakeCommands.reverse(intake, indexer));

        createTriggeredCommand(moveArm, ArmCommands.moveToAmp());

        createTriggeredCommand(retractArm, ArmCommands.retractArm());
    }

    private void createTriggeredCommand(LoggedDashboardBoolean dashboardBoolean, Command command) {
        new Trigger(dashboardBoolean::get)
                .whileTrue(command.finallyDo(() -> dashboardBoolean.set(false)))
                .and(DriverStation::isDisabled)
                .onTrue(Commands.runOnce(() -> dashboardBoolean.set(false)).ignoringDisable(true));
    }
}
