package com.team1701.robot.commands;

import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private Intake mIntake;
    private RobotState mRobotState;

    public IntakeCommand(Intake intake, RobotState robotState) {
        mIntake = intake;
        mRobotState = robotState;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (mRobotState.hasNote() && !mIntake.hasNote()) {
            mIntake.stop();
        } else {
            mIntake.setForward();
        }
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.stop();
    }
}