package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.PathBuilder;
import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static com.team1701.lib.commands.LoggedCommands.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutonomousCommands {
    private final RobotState mRobotState;
    private final Drive mDrive;
    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final PathBuilder mPathBuilder = new PathBuilder();

    public static record AutonomousCommand(Command command, Pose2d[] path) {
        public AutonomousCommand(Command command) {
            this(command, new Pose2d[] {});
        }
    }

    public AutonomousCommands(RobotState robotState, Drive drive, Shooter shooter, Indexer indexer) {
        mRobotState = robotState;
        mDrive = drive;
        mShooter = shooter;
        mIndexer = indexer;

        NamedCommands.registerCommand("printHello", print("Hello from autonomous path"));
    }

    private Pose2d autoFlipPose(Pose2d pose) {
        var flippedPose = GeometryUtil.flipX(pose, FieldConstants.kFieldLongLengthMeters);
        return Configuration.isRedAlliance() ? flippedPose : pose;
    }

    private Command resetPose(Pose2d pose) {
        return resetPose(() -> autoFlipPose(pose));
    }

    private Command resetPose(Supplier<Pose2d> pose) {
        return runOnce(() -> mRobotState.resetPose(pose.get())).withName("ResetPose");
    }

    private Command timedDriveWithVelocity(ChassisSpeeds speeds, double seconds) {
        return race(DriveCommands.driveWithVelocity(() -> speeds, mDrive), waitSeconds(seconds))
                .withName("TimedDriveWithVelocity");
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
        mPathBuilder.addPose(pose);
        return DriveCommands.driveToPose(
                mDrive, () -> autoFlipPose(pose), mRobotState::getPose2d, kinematicLimits, finishAtPose);
    }

    private Command driveToPoseAndPreWarm(Pose2d pose, KinematicLimits kinematicLimits) {
        var targetSpeed = Constants.Shooter.kShooterSpeedInterpolator.get(
                mRobotState.getSpeakerDistanceFromPose(new Pose3d(pose)));

        var targetAngle = Constants.Shooter.kShooterAngleInterpolator.get(
                mRobotState.getSpeakerDistanceFromPose(new Pose3d(pose)));

        return Commands.deadline(
                driveToPose(pose, kinematicLimits),
                Commands.run(
                        () -> {
                            mShooter.setUnifiedSpeed(targetSpeed);
                            mShooter.setRotationAngle(Rotation2d.fromRadians(targetAngle));
                        },
                        mShooter));
    }

    private Command followPath(String pathName) {
        return followPath(pathName, false);
    }

    private Command followPath(String pathName, boolean resetPose) {
        var path = PathPlannerPath.fromPathFile(pathName);
        if (path == null) {
            return idle(); // Prevent auto mode from continuing if path failed to load
        }

        mPathBuilder.addPath(path);

        var command = sequence(
                        runOnce(() -> mDrive.setKinematicLimits(Constants.Drive.kFastKinematicLimits)),
                        resetPose ? resetPose(path.getPreviewStartingHolonomicPose()) : none(),
                        AutoBuilder.followPath(path))
                .withName("DrivePathPlannerPath");
        command.addRequirements(mDrive);
        return command;
    }

    private Command followChoreoPath(String pathName) {
        return followChoreoPath(pathName, false);
    }

    private Command followChoreoPath(String pathName, boolean resetPose) {
        var trajectory = Choreo.getTrajectory(pathName);
        if (trajectory == null) {
            return idle().withName(
                            "Empty Idle Command"); // Prevent auto mode from continuing if trajectory failed to load
        }

        mPathBuilder.addPath(trajectory.getPoses());

        return new DriveChoreoTrajectory(mDrive, trajectory, mRobotState, resetPose);
    }

    private Pose2d getFirstPose(String pathName) {
        var trajectory = Choreo.getTrajectory(pathName);
        return trajectory.getInitialPose();
    }

    private Command followChoreoPathAndPreWarm(String pathName) {
        return followChoreoPathAndPreWarm(pathName, false, false);
    }

    private Command followChoreoPathAndPreWarm(String pathName, boolean resetPose, boolean waitForNote) {
        var trajectory = Choreo.getTrajectory(pathName);
        if (trajectory == null) {
            return idle().withName(
                            "Empty Idle Command"); // Prevent auto mode from continuing if trajectory failed to load
        }

        mPathBuilder.addPath(trajectory.getPoses());

        var targetSpeed = Constants.Shooter.kShooterSpeedInterpolator.get(
                mRobotState.getSpeakerDistanceFromPose(new Pose3d(autoFlipPose(trajectory.getFinalPose()))));

        var targetAngle = Constants.Shooter.kShooterAngleInterpolator.get(
                mRobotState.getSpeakerDistanceFromPose(new Pose3d(autoFlipPose(trajectory.getFinalPose()))));

        return Commands.deadline(
                        new DriveChoreoTrajectory(mDrive, trajectory, mRobotState, resetPose),
                        Commands.run(
                                () -> {
                                    if (mRobotState.hasNote() || !waitForNote) {
                                        mShooter.setUnifiedSpeed(targetSpeed);
                                        mShooter.setRotationAngle(Rotation2d.fromRadians(targetAngle));
                                    }
                                },
                                mShooter))
                .withName("FollowAndPreWarm");
    }

    private Command aimAndShoot() {
        return ShootCommands.aimAndShootInSpeaker(mShooter, mIndexer, mDrive, mRobotState);
    }

    public AutonomousCommand demo() {
        var command = loggedSequence(
                        print("Starting demo"),
                        followPath("demo1", true),
                        aimAndShoot(),
                        driveToPose(new Pose2d(2.0, 1.0, Rotation2d.fromRadians(-Math.PI * 2.0 / 3.0))),
                        driveToPose(new Pose2d(10.0, 1.0, GeometryUtil.kRotationHalfPi)),
                        driveToPose(
                                new Pose2d(2.0, 5.0, GeometryUtil.kRotationIdentity),
                                Constants.Drive.kSlowKinematicLimits),
                        followPath("demo2"),
                        driveToPose(new Pose2d(10.0, 5.0, GeometryUtil.kRotationMinusHalfPi), false))
                .withName("AutonomousDemo");

        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand shootAndBackup() {
        var command = loggedSequence(
                        print("Started shoot and backup"),
                        timedDriveWithVelocity(new ChassisSpeeds(-1, 0, 0), 2.0),
                        aimAndShoot(),
                        timedDriveWithVelocity(new ChassisSpeeds(-1, 0, 0), 2.0))
                .withName("ShootAndBackupAuto");

        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand fourPiece() {
        var command = loggedSequence(
                        print("Started four piece auto"),
                        followChoreoPathAndPreWarm("FourPiece.1", true, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FourPiece.2", false, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FourPiece.3", false, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FourPiece.4", false, false),
                        aimAndShoot())
                .withName("FourPieceAuto");

        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand fourPieceAmp() {
        var command = loggedSequence(
                        print("Started four piece near amp auto"),
                        followChoreoPathAndPreWarm("FourPieceAmp.1", true, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FourPieceAmp.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FourPieceAmp.3"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FourPieceAmp.4"),
                        aimAndShoot())
                .withName("FourPieceAmpAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand sourceFourPieceTwoOne() {
        var command = loggedSequence(
                        print("Started source four piece two one auto"),
                        driveToPoseAndPreWarm(
                                new Pose2d(
                                        new Translation2d(2.46260666847229, 2.526517391204834),
                                        new Rotation2d(2.2091161460921795)),
                                Constants.Drive.kFastTrapezoidalKinematicLimits),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceFourPieceTwoOne.1"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceFourPieceTwoOne.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceFourPieceTwoOne.3"),
                        aimAndShoot())
                .withName("SourceFourPieceTwoOneAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand middleToMiddle() {
        var command = loggedSequence(
                        print("Started middle to middle auto"),
                        followChoreoPathAndPreWarm("MiddleToMiddle.1", true, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("MiddleToMiddle.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("MiddleToMiddle.3"),
                        aimAndShoot())
                .withName("MiddleToMiddleAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand fiveMiddleToMiddle() {
        var command = loggedSequence(
                        print("Started five middle to middle auto"),
                        followChoreoPathAndPreWarm("FiveMiddleToMiddle.1", true, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveMiddleToMiddle.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveMiddleToMiddle.3"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveMiddleToMiddle.4"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveMiddleToMiddle.5"),
                        aimAndShoot())
                .withName("FiveMiddleToMiddleAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand greedyMiddle() {
        var command = loggedSequence(
                        print("Started greedy middle auto"),
                        followChoreoPathAndPreWarm("GreedyMiddle.1", true, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("GreedyMiddle.2", false, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("GreedyMiddle.3", false, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("GreedyMiddle.4"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("GreedyMiddle.5"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("GreedyMiddle.6"),
                        aimAndShoot(),
                        followChoreoPath("GreedyMiddle.7"))
                .withName("GreedyMiddleAuto");

        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand sourceSideMiddleThree() {
        var command = loggedSequence(
                        print("Started source side middle three auto"),
                        driveToPoseAndPreWarm(
                                getFirstPose("SourceSideMiddleThree.1"),
                                Constants.Drive.kFastTrapezoidalKinematicLimits),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceSideMiddleThree.1"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceSideMiddleThree.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceSideMiddleThree.3"),
                        aimAndShoot())
                .withName("SourceThreeMiddleAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand fivePieceAmp() {
        var command = loggedSequence(
                        print("Started five piece near amp auto"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveAmp.1", false, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveAmp.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveAmp.3"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveAmp.4"),
                        aimAndShoot())
                .withName("FivePieceAmpAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand straightToMiddle() {
        var command = loggedSequence(
                        print("Started straight to middle auto"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("StraightToMiddle.1", false, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("StraightToMiddle.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("StraightToMiddle.3"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("StraightToMiddle.4"))
                .withName("StraightToMiddleAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand sourceFourUnderStage() {
        var command = loggedSequence(
                        print("Started source four under stage auto"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceFourUnderStage.1"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceFourUnderStage.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceFourUnderStage.3"),
                        aimAndShoot())
                .withName("SourceFourUnderStageAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }
}
