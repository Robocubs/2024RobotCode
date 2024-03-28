package com.team1701.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

public class DriveAndSeekNote extends Command {
    private static final String kLoggingPrefix = "Command/DriveAndSeekNote/";

    private final Command mDriveCommand;
    private final Command mDriveToNoteCommand;
    private final Supplier<Optional<Pose2d>> mNotePoseSupplier;

    private boolean mSeeking;
    private Optional<Pose2d> mNotePose = Optional.empty();

    DriveAndSeekNote(
            Drive drive,
            RobotState robotState,
            Command driveCommand,
            Supplier<Optional<Pose2d>> notePose,
            KinematicLimits seekingKinematicLimits) {
        mDriveCommand = driveCommand;
        mNotePoseSupplier = notePose;

        mDriveToNoteCommand =
                DriveCommands.driveToNote(drive, robotState, () -> mNotePose, seekingKinematicLimits, true);
        CommandScheduler.getInstance().registerComposedCommands(mDriveCommand, mDriveToNoteCommand);

        addRequirements(mDriveCommand.getRequirements().toArray(Subsystem[]::new));
        addRequirements(mDriveToNoteCommand.getRequirements().toArray(Subsystem[]::new));
    }

    @Override
    public void initialize() {
        mDriveCommand.initialize();
        mSeeking = false;
        mNotePose = Optional.empty();
    }

    @Override
    public void execute() {
        if (!mSeeking) {
            mDriveCommand.execute();
        }

        mNotePose = mNotePoseSupplier.get();
        if (mSeeking) {
            mDriveToNoteCommand.execute();
        } else if (mNotePose.isPresent()) {
            mDriveCommand.end(true);
            mDriveToNoteCommand.initialize();
            mSeeking = true;
        }

        Logger.recordOutput(kLoggingPrefix + "Seeking", mSeeking);
    }

    @Override
    public void end(boolean interrupted) {
        if (mSeeking) {
            mDriveToNoteCommand.end(interrupted);
        } else {
            mDriveCommand.end(interrupted);
        }

        Logger.recordOutput(kLoggingPrefix + "Seeking", false);
    }

    @Override
    public boolean isFinished() {
        return mSeeking ? mDriveToNoteCommand.isFinished() : mDriveCommand.isFinished();
    }
}
