package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveCommands {
    public static DriveWithJoysticks driveWithJoysticks(
            Drive drive,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            Supplier<KinematicLimits> kinematicLimits) {
        return new DriveWithJoysticks(drive, throttle, strafe, rotation, kinematicLimits);
    }

    public static DriveToPose driveToPose(
            Drive drive, Pose2d pose, KinematicLimits kinematicLimits, boolean finishAtPose) {
        return new DriveToPose(drive, pose, kinematicLimits, finishAtPose);
    }

    public static Command swerveLock(Drive drive) {
        var command = Commands.runOnce(drive::engageSwerveLock, drive).andThen(Commands.idle(drive));
        command.setName("SwerveLock");
        return command;
    }
}
