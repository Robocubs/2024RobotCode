package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveCommands {
    public static Command driveWithJoysticks(
            Drive drive,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            Supplier<KinematicLimits> kinematicLimits) {
        return new DriveWithJoysticks(drive, throttle, strafe, rotation, kinematicLimits);
    }

    public static Command driveToPose(
            Drive drive,
            Supplier<Pose2d> poseSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            KinematicLimits kinematicLimits,
            boolean finishAtPose) {
        return new DriveToPose(drive, poseSupplier, robotPoseSupplier, kinematicLimits, finishAtPose);
    }

    public static RotateToFieldHeading rotateRelativeToRobot(
            Drive drive,
            Supplier<Rotation2d> targetFieldHeading,
            KinematicLimits kinematicLimits,
            boolean finishAtRotation) {
        return new RotateToFieldHeading(drive, targetFieldHeading, kinematicLimits, finishAtRotation);
    }

    public static Command swerveLock(Drive drive) {
        return Commands.runOnce(drive::engageSwerveLock, drive)
                .andThen(Commands.idle(drive))
                .withName("SwerveLock");
    }
}
