package com.team1701.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
                                    headingSupplier.get(),
                                    kinematicLimits.maxDriveVelocity());
                            var rotation = MathUtil.applyDeadband(
                                    rotationSupplier.getAsDouble(), Constants.Controls.kDriverDeadband);
                            var rotationRadiansPerSecond = Math.copySign(rotation * rotation, rotation)
                                    * kinematicLimits.maxDriveVelocity()
                                    / Constants.Drive.kModuleRadius;

                            drive.setVelocity(new ChassisSpeeds(
                                    translationVelocities.getX(),
                                    translationVelocities.getY(),
                                    rotationRadiansPerSecond));
                        },
                        drive::stop,
                        drive)
                .withName("DriveWithJoysticks");
    }

    public static Translation2d calculateDriveWithJoysticksVelocities(
            double throttle, double strafe, Rotation2d heading, double maxVelocity) {
        var translationSign = Configuration.isBlueAlliance() ? 1.0 : -1.0;
        var magnitude = Math.hypot(throttle, strafe);
        return magnitude < Constants.Controls.kDriverDeadband
                ? GeometryUtil.kTranslationIdentity
                : new Translation2d(throttle * maxVelocity * translationSign, strafe * maxVelocity * translationSign)
                        .rotateBy(heading.unaryMinus());
    }

    public static Command driveToPose(
            Drive drive,
            RobotState robotState,
            Supplier<Pose2d> poseSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            KinematicLimits kinematicLimits,
            double translationTolerance,
            boolean finishAtPose) {
        return new DriveToPose(
                drive,
                robotState,
                poseSupplier,
                robotPoseSupplier,
                kinematicLimits,
                translationTolerance,
                finishAtPose);
    }

    public static Command rotateToSpeaker(
            Drive drive, RobotState state, KinematicLimits kinematicLimits, boolean finishAtRotation) {
        return rotateToFieldHeading(
                drive,
                () -> state.getSpeakerHeading()
                        .minus(state.getShootingState().setpoint.releaseAngle()),
                state::getHeading,
                state::getSpeakerHeadingTolerance,
                kinematicLimits,
                finishAtRotation);
    }

    public static Command rotateToPassTarget(
            Drive drive,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            RobotState state,
            KinematicLimits kinematicLimits) {
        return new RotateToFieldHeading(
                        drive,
                        () -> calculateDriveWithJoysticksVelocities(
                                        throttle.getAsDouble(),
                                        strafe.getAsDouble(),
                                        drive.getFieldRelativeHeading(),
                                        kinematicLimits.maxDriveVelocity())
                                .rotateBy(state.getHeading()),
                        state::getPassingHeading,
                        state::getHeading,
                        () -> new Rotation2d(.01),
                        kinematicLimits,
                        false)
                .withName("RotateToPassTarget");
    }

    public static Command rotateToFieldHeading(
            Drive drive,
            Supplier<Rotation2d> targetFieldHeading,
            Supplier<Rotation2d> robotHeadingSupplier,
            Supplier<Rotation2d> headingTolerance,
            KinematicLimits kinematicLimits,
            boolean finishAtRotation) {
        return new RotateToFieldHeading(
                drive, targetFieldHeading, robotHeadingSupplier, headingTolerance, kinematicLimits, finishAtRotation);
    }

    public static Command stop(Drive drive) {
        return Commands.runOnce(drive::stop, drive)
                .andThen(Commands.idle(drive))
                .withName("StopDrive");
    }

    public static Command swerveLock(Drive drive) {
        return Commands.runOnce(drive::engageSwerveLock, drive)
                .andThen(Commands.idle(drive))
                .withName("SwerveLock");
    }

    public static Command driveToAmp(
            Drive drive,
            RobotState robotState,
            Supplier<Pose2d> poseSupplier,
            KinematicLimits kinematicLimits,
            boolean oppositeAlliance) {
        return new DriveToPose(
                        drive,
                        robotState,
                        () -> Configuration.isBlueAlliance() ^ oppositeAlliance
                                ? FieldConstants.kBlueAmpDrivePose
                                : FieldConstants.kRedAmpDrivePose,
                        poseSupplier,
                        kinematicLimits,
                        0.01,
                        true)
                .withName("DriveToAmp");
    }

    public static Command driveToNote(Drive drive, RobotState robotState, KinematicLimits kinematicLimits) {
        return driveToNote(
                drive,
                robotState,
                () -> robotState.getDetectedNoteForPickup().map(note -> note.pose()
                        .toPose2d()),
                kinematicLimits,
                false);
    }

    public static Command driveToNote(
            Drive drive,
            RobotState robotState,
            Supplier<Optional<Pose2d>> notePose,
            KinematicLimits kinematicLimits,
            boolean finishIfHasPiece) {
        return new DriveToPose(
                        drive,
                        robotState,
                        () -> notePose.get()
                                .map(pose -> new Pose2d(
                                                pose.getTranslation(),
                                                pose.getRotation().plus(GeometryUtil.kRotationPi))
                                        .transformBy(Constants.Robot.kIntakeToRobot))
                                .orElseGet(robotState::getPose2d),
                        robotState::getPose2d,
                        kinematicLimits,
                        0.1,
                        true,
                        finishIfHasPiece)
                .withName("DriveToNote");
    }

    // v2
    public static Command shootAndMoveWithJoysticks(
            Drive drive,
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            DoubleSupplier throttle,
            DoubleSupplier strafe) {
        var maxDriveVelocity = Constants.Drive.kFastSmoothKinematicLimits.maxDriveVelocity();
        return Commands.parallel(
                new RotateToSpeakerAndMove(drive, robotState, () -> calculateDriveWithJoysticksVelocities(
                                throttle.getAsDouble(),
                                strafe.getAsDouble(),
                                drive.getFieldRelativeHeading(),
                                maxDriveVelocity)
                        .rotateBy(robotState.getHeading())),
                ShootCommands.shoot(shooter, indexer, robotState));
    }

    // v3
    public static Command shootAndMove(
            Drive drive,
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            DoubleSupplier throttle,
            DoubleSupplier strafe) {
        var maxDriveVelocity = 2;
        return new ShootAndMove(
                drive,
                shooter,
                indexer,
                robotState,
                () -> calculateDriveWithJoysticksVelocities(
                                throttle.getAsDouble(),
                                strafe.getAsDouble(),
                                drive.getFieldRelativeHeading(),
                                maxDriveVelocity)
                        .rotateBy(robotState.getHeading()),
                false);
    }

    public static Command driveWithVelocity(Supplier<ChassisSpeeds> velocity, Drive drive) {
        return Commands.run(() -> drive.setVelocity(velocity.get()), drive)
                .finallyDo(() -> drive.stop())
                .withName("DriveWithVelocity");
    }

    public static Command passANote(
            Drive drive,
            Shooter shooter,
            Indexer indexer,
            RobotState robotState,
            DoubleSupplier throttleSupplier,
            DoubleSupplier strafeSupplier) {
        return new PassANote(
                        drive,
                        shooter,
                        indexer,
                        robotState,
                        () -> calculateDriveWithJoysticksVelocities(
                                        throttleSupplier.getAsDouble(),
                                        strafeSupplier.getAsDouble(),
                                        drive.getFieldRelativeHeading(),
                                        Constants.Drive.kFastTrapezoidalKinematicLimits.maxDriveVelocity())
                                .rotateBy(robotState.getHeading()),
                        () -> new Rotation2d(.03))
                .withName("PassANote");
    }
}
