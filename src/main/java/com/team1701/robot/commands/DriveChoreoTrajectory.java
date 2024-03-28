package com.team1701.robot.commands;

import java.util.ArrayDeque;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.team1701.lib.util.ChoreoEventMarker;
import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class DriveChoreoTrajectory extends Command {
    private final Drive mDrive;
    private final RobotState mRobotState;
    private final ChoreoTrajectory mTrajectory;
    private final List<ChoreoEventMarker> mEventMarkers;
    private final ChoreoTrajectory mFlippedTrajectory;
    private final ChoreoControlFunction mControlFunction;
    private final Function<Pose2d, Pose2d> mPoseMutator;
    private final Timer mTimer = new Timer();
    private final boolean mResetPose;
    private final Map<Command, Boolean> mRunningEventCommands;
    private final Deque<ChoreoEventMarker> mRemainingEventMarkers;

    DriveChoreoTrajectory(
            Drive drive,
            RobotState robotState,
            ChoreoTrajectory trajectory,
            List<ChoreoEventMarker> eventMarkers,
            boolean resetPose) {
        this(drive, robotState, trajectory, eventMarkers, pose -> pose, resetPose);
    }

    DriveChoreoTrajectory(
            Drive drive,
            RobotState robotState,
            ChoreoTrajectory trajectory,
            List<ChoreoEventMarker> eventMarkers,
            Function<Pose2d, Pose2d> poseMutator,
            boolean resetPose) {
        mDrive = drive;
        mRobotState = robotState;
        mTrajectory = trajectory;
        mEventMarkers = eventMarkers;
        mFlippedTrajectory = trajectory.flipped();
        mControlFunction = Choreo.choreoSwerveController(
                new PIDController(Constants.Drive.kPathTranslationKp, 0.0, Constants.Drive.kPathTranslationKd),
                new PIDController(Constants.Drive.kPathTranslationKp, 0.0, Constants.Drive.kPathTranslationKd),
                new PIDController(Constants.Drive.kPathRotationKp, 0.0, 0.0));
        mPoseMutator = poseMutator;
        mResetPose = resetPose;

        mRunningEventCommands = new HashMap<>(mEventMarkers.size());
        mRemainingEventMarkers = new ArrayDeque<>(mEventMarkers.size());

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        mTimer.restart();

        mRunningEventCommands.clear();
        mRemainingEventMarkers.clear();
        mRemainingEventMarkers.addAll(mEventMarkers);

        var trajectory = shouldFlip() ? mFlippedTrajectory : mTrajectory;
        var initialPose = mPoseMutator.apply(trajectory.getInitialPose());
        if (mResetPose) {
            mRobotState.resetPose(initialPose);
        }

        mDrive.setKinematicLimits(Constants.Drive.kUncappedKinematicLimits);

        Logger.recordOutput("Choreo/Path", trajectory.getPoses());
        Logger.recordOutput("Choreo/TargetPose", initialPose);
    }

    @Override
    public void execute() {
        var trajectory = shouldFlip() ? mFlippedTrajectory : mTrajectory;
        var state = trajectory.sample(mTimer.get());
        var mutatedPose = mPoseMutator.apply(state.getPose());
        var mutatedState = new ChoreoTrajectoryState(
                state.timestamp,
                mutatedPose.getX(),
                mutatedPose.getY(),
                mutatedPose.getRotation().getRadians(),
                state.velocityX,
                state.velocityY,
                state.angularVelocity);
        var chassisSpeeds = mControlFunction.apply(mRobotState.getPose2d(), mutatedState);
        mDrive.setVelocity(chassisSpeeds);

        if (!mRemainingEventMarkers.isEmpty()
                && mTimer.hasElapsed(mRemainingEventMarkers.peek().timestamp())) {
            var event = mRemainingEventMarkers.pop();
            for (var runningCommand : mRunningEventCommands.entrySet()) {
                if (!runningCommand.getValue()) {
                    continue;
                }

                if (!Collections.disjoint(
                        runningCommand.getKey().getRequirements(),
                        event.command().getRequirements())) {
                    runningCommand.getKey().end(true);
                    runningCommand.setValue(false);
                }
            }

            event.command().initialize();
            mRunningEventCommands.put(event.command(), true);
        }

        for (var runningCommand : mRunningEventCommands.entrySet()) {
            if (!runningCommand.getValue()) {
                continue;
            }

            runningCommand.getKey().execute();

            if (runningCommand.getKey().isFinished()) {
                runningCommand.getKey().end(false);
                runningCommand.setValue(false);
            }
        }

        Logger.recordOutput("Choreo/TargetPose", mutatedState.getPose());
    }

    @Override
    public void end(boolean interrupted) {
        mTimer.stop();
        if (interrupted) {
            mDrive.stop();
        } else {
            var trajectory = shouldFlip() ? mFlippedTrajectory : mTrajectory;
            mDrive.setVelocity(trajectory.getFinalState().getChassisSpeeds());
        }

        for (var runningCommand : mRunningEventCommands.entrySet()) {
            if (runningCommand.getValue()) {
                runningCommand.getKey().end(true);
            }
        }

        Logger.recordOutput("Choreo/Path", new Pose2d[] {});
    }

    @Override
    public boolean isFinished() {
        return mTimer.hasElapsed(mTrajectory.getTotalTime());
    }

    private boolean shouldFlip() {
        return Configuration.isRedAlliance();
    }
}
