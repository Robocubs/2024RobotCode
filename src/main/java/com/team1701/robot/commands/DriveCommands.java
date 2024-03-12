package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommands {
    public static Command driveWithJoysticks(
            Drive drive,
            Supplier<Rotation2d> headingSupplier,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            Supplier<KinematicLimits> kinematicLimits) {
        return new DriveWithJoysticks(drive, headingSupplier, throttle, strafe, rotation, kinematicLimits);
    }

    public static Command driveToPose(
            Drive drive,
            Supplier<Pose2d> poseSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            KinematicLimits kinematicLimits,
            boolean finishAtPose) {
        return new DriveToPose(drive, poseSupplier, robotPoseSupplier, kinematicLimits, finishAtPose);
    }

    public static Command rotateToSpeaker(
            Drive drive, RobotState state, KinematicLimits kinematicLimits, boolean finishAtRotation) {
        return rotateToFieldHeading(
                drive, state::getSpeakerHeading, state::getHeading, kinematicLimits, finishAtRotation);
    }

    public static Command rotateToFieldHeading(
            Drive drive,
            Supplier<Rotation2d> targetFieldHeading,
            Supplier<Rotation2d> robotHeadingSupplier,
            KinematicLimits kinematicLimits,
            boolean finishAtRotation) {
        return new RotateToFieldHeading(
                drive, targetFieldHeading, robotHeadingSupplier, kinematicLimits, finishAtRotation);
    }

    public static Command swerveLock(Drive drive) {
        return Commands.runOnce(drive::engageSwerveLock, drive)
                .andThen(Commands.idle(drive))
                .withName("SwerveLock");
    }

    public static Command driveToAmp(Drive drive, Supplier<Pose2d> poseSupplier, KinematicLimits kinematicLimits) {
        return new DriveToPose(
                        drive,
                        () -> Configuration.isBlueAlliance()
                                ? FieldConstants.kBlueAmpDrivePose
                                : FieldConstants.kRedAmpDrivePose,
                        poseSupplier,
                        kinematicLimits,
                        true)
                .withName("DriveToAmp");
    }

    public static Command driveToPiece(
            Drive drive,
            RobotState robotState,
            KinematicLimits kinematicLimits,
            CommandXboxController driverController) {
        return new DriveToPose(
                drive,
                () -> robotState
                        .getDetectedNoteForPickup()
                        .map(note -> note.pose().toPose2d())
                        .map(pose -> new Pose2d(
                                        pose.getTranslation(),
                                        pose.getRotation().plus(GeometryUtil.kRotationPi))
                                .transformBy(Constants.Robot.kIntakeToRobot))
                        .orElseGet(robotState::getPose2d),
                robotState::getPose2d,
                kinematicLimits,
                true);
    }

    public static Command driveWithVelocity(Supplier<ChassisSpeeds> velocity, Drive drive) {
        return Commands.run(() -> drive.setVelocity(velocity.get()), drive)
                .finallyDo(() -> drive.stop())
                .withName("DriveWithVelocity");
    }
}
