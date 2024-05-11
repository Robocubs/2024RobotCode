package com.team1701.robot.commands;

import com.team1701.lib.util.tuning.LoggedTunableNumber;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class SpitNote extends Command {
    private static final String kLoggingPrefix = "Command/SpitNote/";
    private static final LoggedTunableNumber kRollerSpeed =
            new LoggedTunableNumber(kLoggingPrefix + "RollerSpeed", 200);

    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final RobotState mRobotState;

    private boolean mHasSpitNote = false;

    public SpitNote(Shooter shooter, Indexer indexer, RobotState robotState) {
        mShooter = shooter;
        mIndexer = indexer;
        mRobotState = robotState;
        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        mHasSpitNote = !mRobotState.hasNote();
    }

    @Override
    public void execute() {
        mShooter.setRollerSpeeds(new ShooterSpeeds(kRollerSpeed.get()));
        mIndexer.setForwardLoad();
        Logger.recordOutput(kLoggingPrefix + "Spitting", true);
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stop();
        mIndexer.stop();
        Logger.recordOutput(kLoggingPrefix + "Spitting", false);
    }

    @Override
    public boolean isFinished() {
        mHasSpitNote = !mRobotState.hasNote();
        return mHasSpitNote;
    }
}
