package com.team1701.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class DriveChoreoTrajectory extends Command {
    private final Drive mDrive;
    private final ChoreoTrajectory mTrajectory;
    private final ChoreoTrajectory mFlippedTrajectory;
    private final ChoreoControlFunction mControlFunction;
    private final Timer mTimer = new Timer();
    private final boolean mResetPose;

    DriveChoreoTrajectory(Drive drive, ChoreoTrajectory trajectory, boolean resetPose) {
        mDrive = drive;
        mTrajectory = trajectory;
        mFlippedTrajectory = trajectory.flipped();
        mControlFunction = Choreo.choreoSwerveController(
                new PIDController(Constants.Drive.kPathTranslationKp, 0.0, 0.0),
                new PIDController(Constants.Drive.kPathTranslationKp, 0.0, 0.0),
                new PIDController(Constants.Drive.kPathRotationKp, 0.0, 0.0));
        mResetPose = resetPose;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        mTimer.restart();

        var trajectory = shouldFlip() ? mFlippedTrajectory : mTrajectory;
        if (mResetPose) {
            PoseEstimator.getInstance().setPose(trajectory.getInitialPose());
        }

        mDrive.setKinematicLimits(Constants.Drive.kUncappedKinematicLimits);

        Logger.recordOutput("Choreo/Path", trajectory.getPoses());
        Logger.recordOutput("Choreo/TargetPose", trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        var trajectory = shouldFlip() ? mFlippedTrajectory : mTrajectory;
        var state = trajectory.sample(mTimer.get());
        var chassisSpeeds = mControlFunction.apply(PoseEstimator.getInstance().getPose2d(), state);
        mDrive.setVelocity(chassisSpeeds);

        Logger.recordOutput("Choreo/TargetPose", state.getPose());
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

        Logger.recordOutput("Choreo/Path", new Pose2d[] {});
    }

    @Override
    public boolean isFinished() {
        return mTimer.hasElapsed(mTrajectory.getTotalTime());
    }

    private boolean shouldFlip() {
        return Configuration.getAlliance() == Alliance.Red;
    }
}
