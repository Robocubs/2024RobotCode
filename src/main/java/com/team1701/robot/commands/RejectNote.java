package com.team1701.robot.commands;

import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class RejectNote extends Command {
    private static final String kLoggingPrefix = "Command/RejectNote";

    private final Intake mIntake;
    private final Indexer mIndexer;

    private final RobotState mRobotState;

    // use override for if we want to hold until note is rejected (if sensor fails etc)
    private boolean mOverrideRobotState;
    private boolean mRejected;

    public RejectNote(Intake intake, Indexer indexer, RobotState robotState, boolean overrideRobotState) {
        mIntake = intake;
        mIndexer = indexer;
        mRobotState = robotState;
        mOverrideRobotState = overrideRobotState;

        addRequirements(intake, indexer);
    }

    @Override
    public void initialize() {
        mRejected = false;
        if (!mOverrideRobotState && !mRobotState.hasNote()) {
            mRejected = true;
        }
    }

    @Override
    public void execute() {
        if (!mRejected) {
            mIndexer.setReverse();
            mIntake.setReverse();
        }

        mRejected = !mRobotState.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        mIndexer.stop();
        mIntake.stop();
    }

    @Override
    public boolean isFinished() {
        return mRejected && !mOverrideRobotState;
    }
}
