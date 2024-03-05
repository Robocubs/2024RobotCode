package com.team1701.robot.commands;

import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class IntakeCommand extends Command {
    private Intake mIntake;
    private RobotState mRobotState;
    private XboxController mDriverController;

    public IntakeCommand(Intake intake, CommandXboxController driverController, RobotState robotState) {
        mIntake = intake;
        mDriverController = driverController.getHID();
        mRobotState = robotState;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (mRobotState.hasNote() && !mIntake.hasNote()) {
            mIntake.stop();
            // mDriverController.setRumble(RumbleType.kLeftRumble, 0);
        } else if (mIntake.hasNote()) {
            mIntake.setForward();
            // mDriverController.setRumble(RumbleType.kLeftRumble, 0.1);

        } else if (mRobotState.getDetectedNotePoses2d().length > 0) {
            mIntake.setMediumForward();
        } else {
            mIntake.setSlowForward();
        }
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.stop();
    }
}
