package com.team1701.robot.commands;

import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private Intake mIntake;
    private Indexer mIndexer;

    IntakeCommand(Intake intake, Indexer indexer) {
        mIntake = intake;
        mIndexer = indexer;
    }

    public void execute() {
        if (mIntake.hasNoteAtInput()) {
            mIntake.setForward();
        } else if (mIntake.hasNoteAtExit()) {
            mIntake.stop();
            mIndexer.setForwardLoad();
        }
    }
}
