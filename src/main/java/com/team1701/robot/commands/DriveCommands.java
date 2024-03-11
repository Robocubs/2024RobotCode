package com.team1701.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import com.team1701.lib.commands.LoggedCommands;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommands {
    public static Command driveWithJoysticks(
            Drive drive,
            Supplier<Rotation2d> headingSupplier,
            DoubleSupplier throttleSupplier,
            DoubleSupplier strafeSupplier,
            DoubleSupplier rotationSupplier,
            Supplier<KinematicLimits> kinematicLimitsSupplier) {
        return Commands.runEnd(
                () -> {
                    var kinematicLimits = kinematicLimitsSupplier.get();
                    drive.setKinematicLimits(kinematicLimits);

                    var translationVelocities = calculateDriveWithJoysticksVelocities(
                            throttleSupplier.getAsDouble(),
                            strafeSupplier.getAsDouble(),
                            kinematicLimits.maxDriveVelocity());
                    var rotation =
                            MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.Controls.kDriverDeadband);
                    var rotationRadiansPerSecond = Math.copySign(rotation * rotation, rotation)
                            * kinematicLimits.maxDriveVelocity()
                            / Constants.Drive.kModuleRadius;

                    drive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                            translationVelocities.getX(),
                            translationVelocities.getY(),
                            rotationRadiansPerSecond,
                            headingSupplier.get()));
                },
                drive::stop,
                drive);
    }

    private static Translation2d calculateDriveWithJoysticksVelocities(
            double throttle, double strafe, double maxVelocity) {
        var translationSign = Configuration.isBlueAlliance() ? 1.0 : -1.0;
        var magnitude = Math.hypot(throttle, strafe);
        return magnitude < Constants.Controls.kDriverDeadband
                ? GeometryUtil.kTranslationIdentity
                : new Translation2d(throttle * maxVelocity * translationSign, strafe * maxVelocity * translationSign);
    }

    public static Command driveToPose(
            Drive drive,
            Supplier<Pose2d> poseSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            KinematicLimits kinematicLimits,
            boolean finishAtPose) {
        return new DriveToPose(drive, poseSupplier, robotPoseSupplier, kinematicLimits, finishAtPose);
    }

    public static Command driveToPose(
            Drive drive,
            Supplier<Pose2d> poseSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            KinematicLimits kinematicLimits,
            boolean finishAtPose,
            CommandXboxController driverController) {
        return new DriveToPose(drive, poseSupplier, robotPoseSupplier, kinematicLimits, finishAtPose, driverController);
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
                Configuration.isBlueAlliance()
                        ? () -> FieldConstants.kBlueAmpDrivePose
                        : () -> FieldConstants.kRedAmpDrivePose,
                poseSupplier,
                kinematicLimits,
                true);
    }

    public static Command driveToPiece(
            Drive drive,
            RobotState robotState,
            KinematicLimits kinematicLimits,
            CommandXboxController driverController) {
        return Commands.defer(
                () -> {
                    var robotPose = robotState.getPose2d();
                    var robotTranslation = robotPose.getTranslation();
                    var robotRotationReverse = robotPose.getRotation().plus(GeometryUtil.kRotationPi);
                    return Stream.of(robotState.getDetectedNotePoses2d())
                            .filter(notePose -> GeometryUtil.isNear(
                                    robotRotationReverse,
                                    notePose.getTranslation()
                                            .minus(robotTranslation)
                                            .getAngle(),
                                    Rotation2d.fromDegrees(45)))
                            .min((notePose1, notePose2) -> Double.compare(
                                    robotTranslation.getDistance(notePose1.getTranslation()),
                                    robotTranslation.getDistance(notePose2.getTranslation())))
                            .map(pose -> LoggedCommands.logged(DriveCommands.driveToPose(
                                            drive,
                                            () -> new Pose2d(
                                                    pose.getTranslation(),
                                                    pose.getRotation().plus(GeometryUtil.kRotationPi)),
                                            () -> robotState.getPose2d(),
                                            kinematicLimits,
                                            true,
                                            driverController)
                                    .withName("DriveToPiecePose")))
                            .orElse(LoggedCommands.logged(Commands.none().withName("NoneCommand")))
                            .withName("DriveToPiece");
                },
                Set.of(drive));
    }

    public static Command shootAndMoveWithJoysticks(
            Drive drive,
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            DoubleSupplier throttle,
            DoubleSupplier strafe) {
        var maxDriveVelocity = Constants.Drive.kFastSmoothKinematicLimits.maxDriveVelocity();
        return Commands.parallel(
                new RotateToSpeakerAndMove(
                        drive,
                        robotState,
                        () -> calculateDriveWithJoysticksVelocities(
                                throttle.getAsDouble(), strafe.getAsDouble(), maxDriveVelocity)),
                ShootCommands.shoot(shooter, indexer, robotState, true));
    }

    public static Command driveWithVelocity(Supplier<ChassisSpeeds> velocity, Drive drive) {
        return Commands.run(() -> drive.setVelocity(velocity.get()), drive)
                .finallyDo(() -> drive.stop())
                .withName("DriveWithVelocity");
    }
}
