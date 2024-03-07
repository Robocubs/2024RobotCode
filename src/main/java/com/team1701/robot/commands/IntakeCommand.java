package com.team1701.robot.commands;

import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private Intake mIntake;
    private Indexer mIndexer;
    private RobotState mRobotState;

    public IntakeCommand(Intake intake, Indexer indexer, RobotState robotState) {
        mIntake = intake;
        mIndexer = indexer;
        mRobotState = robotState;
        addRequirements(intake);
    }

    @Override
    public void execute() {

        if (mIndexer.hasNote()) {
            mIntake.stop();
        } else {
            mIntake.setForward();
        }
        //     // mDriverController.setRumble(RumbleType.kLeftRumble, 0);
        // } else if (mIntake.hasNote()) {
        //     mIntake.setForward();
        //     // mDriverController.setRumble(RumbleType.kLeftRumble, 0.1);

        // } else if (mRobotState.getDetectedNotePoses2d().length > 0) {
        //     mIntake.setMediumForward();
        // } else {
        //     mIntake.setSlowForward();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.stop();
    }
}
