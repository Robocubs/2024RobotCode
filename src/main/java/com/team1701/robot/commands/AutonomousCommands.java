package com.team1701.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import static com.team1701.lib.commands.LoggedCommands.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutonomousCommands {
    private final Drive mDrive;

    public AutonomousCommands(Drive drive) {
        mDrive = drive;

        NamedCommands.registerCommand("printHello", print("Hello from autonomous path"));
    }

    private Command resetPose(Pose2d pose) {
        return runOnce(() -> PoseEstimator.getInstance().setPose(pose)).withName("ResetPose");
    }

    private Command driveToPose(Pose2d pose) {
        return driveToPose(pose, Constants.Drive.kFastTrapezoidalKinematicLimits, true);
    }

    private Command driveToPose(Pose2d pose, boolean finishAtPose) {
        return driveToPose(pose, Constants.Drive.kFastTrapezoidalKinematicLimits, finishAtPose);
    }

    private Command driveToPose(Pose2d pose, KinematicLimits kinematicLimits) {
        return driveToPose(pose, kinematicLimits, true);
    }

    private Command driveToPose(Pose2d pose, KinematicLimits kinematicLimits, boolean finishAtPose) {
        return DriveCommands.driveToPose(mDrive, pose, kinematicLimits, finishAtPose);
    }

    private Command followPath(String pathName) {
        return followPath(pathName, false);
    }

    private Command followPath(String pathName, boolean resetPose) {
        var path = PathPlannerPath.fromPathFile(pathName);
        if (path == null) {
            return none();
        }

        var command = AutoBuilder.followPathWithEvents(path);
        if (resetPose) {
            command = command.beforeStarting(resetPose(path.getPreviewStartingHolonomicPose()));
            command.setName("ResetPoseAndFollowPathWithEvents");
        }

        return command;
    }

    public Command demo() {
        return loggedSequence(
                        print("Starting demo"),
                        followPath("demo1", true),
                        driveToPose(new Pose2d(2.0, 1.0, Rotation2d.fromRadians(-Math.PI * 2.0 / 3.0))),
                        driveToPose(new Pose2d(10.0, 1.0, GeometryUtil.kRotationHalfPi)),
                        driveToPose(
                                new Pose2d(2.0, 5.0, GeometryUtil.kRotationIdentity),
                                Constants.Drive.kSlowKinematicLimits),
                        followPath("demo2"),
                        driveToPose(new Pose2d(10.0, 5.0, GeometryUtil.kRotationMinusHalfPi), false))
                .withName("AutonomousDemo");
    }
}
