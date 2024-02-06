package com.team1701.robot.commands;

import com.team1701.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private Intake mIntake;
    private boolean mIntaking;

    IntakeCommand(Intake intake) {
        mIntake = intake;
    }

    @Override
    public void initialize() {
        mIntaking = mIntake.hasNoteAtInput() ? true : false;
    }

    @Override
    public void execute() {
        mIntake.setForward();
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.stop();
    }

    @Override
    public boolean isFinished() {
        mIntaking = mIntake.hasNoteAtExit() ? false : true;
        return mIntaking;
    }
}
