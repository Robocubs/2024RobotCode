package com.team1701.robot.commands;

import java.util.function.BooleanSupplier;

import com.team1701.robot.subsystems.indexer.Indexer;
import edu.wpi.first.wpilibj2.command.Command;

public class IndexCommand extends Command {
    private final Indexer mIndexer;
    private final BooleanSupplier mShouldLoad;

    public IndexCommand(Indexer indexer, BooleanSupplier shouldLoad) {
        mIndexer = indexer;
        mShouldLoad = shouldLoad;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        if (mIndexer.hasNoteAtExit() || !mShouldLoad.getAsBoolean()) {
            mIndexer.stop();
        } else if (mIndexer.hasNoteAtEntrance()) {
            mIndexer.setSlowLoad();
        } else {
            mIndexer.setForwardLoad();
        }
    }

    @Override
    public void end(boolean interrupted) {
        mIndexer.stop();
    }
}
