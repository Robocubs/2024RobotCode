package com.team1701.robot.commands;

import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.ChoreoEventMarker;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.PathBuilder;
import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.autonomous.AutoNote;
import com.team1701.robot.autonomous.AutoNoteSeeker;
import com.team1701.robot.commands.ShootAndDriveToPose.FinishedState;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSetpoint;
import com.team1701.robot.util.FieldUtil;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import static com.team1701.lib.commands.LoggedCommands.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutonomousCommands {
    private static final KinematicLimits kAutoTrapezoidalKinematicLimits =
            new KinematicLimits(4.535, 6.733, 14.535); // From Choreo at 60 amps

    private final RobotState mRobotState;
    private final Drive mDrive;
    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final PathBuilder mPathBuilder = new PathBuilder();
    private final AutoNoteSeeker mAutoNoteSeeker;

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
        mAutoNoteSeeker = new AutoNoteSeeker(robotState);

        NamedCommands.registerCommand("printHello", print("Hello from autonomous path"));
        NamedCommands.registerCommand("seekA", setNoteToSeek(AutoNote.A));
        NamedCommands.registerCommand("seekB", setNoteToSeek(AutoNote.B));
        NamedCommands.registerCommand("seekC", setNoteToSeek(AutoNote.C));
        NamedCommands.registerCommand("seek1", setNoteToSeek(AutoNote.M1));
        NamedCommands.registerCommand("seek2", setNoteToSeek(AutoNote.M2));
        NamedCommands.registerCommand("seek3", setNoteToSeek(AutoNote.M3));
        NamedCommands.registerCommand("seek4", setNoteToSeek(AutoNote.M4));
        NamedCommands.registerCommand("seek5", setNoteToSeek(AutoNote.M5));
    }

    private Pose2d autoFlipPose(Pose2d pose) {
        var flippedPose = GeometryUtil.flipX(pose, FieldConstants.kFieldLongLengthMeters);
        return Configuration.isRedAlliance() ? flippedPose : pose;
    }

    private Command idleShooter() {
        return ShootCommands.stop(mShooter);
    }

    private Command index() {
        return IntakeCommands.idleIndexer(mIndexer);
    }

    private Command stopRoutine() {
        return parallel(DriveCommands.stop(mDrive), idleShooter(), index()).withName("StopRoutine");
    }

    private Command resetPose(Pose2d pose) {
        return resetPose(() -> autoFlipPose(pose));
    }

    private Command resetPose(Supplier<Pose2d> pose) {
        return runOnce(() -> mRobotState.resetPose(pose.get())).withName("ResetPose");
    }

    private Command warmShooter(ShooterSetpoint setpoint) {
        return warmShooter(setpoint, false);
    }

    private Command warmShooter(ShooterSetpoint setpoint, boolean waitForNote) {
        return run(
                        () -> {
                            if (!waitForNote || mRobotState.hasNote()) {
                                mShooter.setRollerSpeeds(setpoint.speeds());
                            } else {
                                mShooter.stopRollers();
                            }

                            mShooter.setRotationAngle(
                                    mRobotState.hasNote() ? setpoint.angle() : Constants.Shooter.kLoadingAngle);
                        },
                        mShooter)
                .withName("WarmShooter");
    }

    private Command setNoteToSeek(AutoNote note) {
        return runOnce(() -> mAutoNoteSeeker.setNote(note))
                .ignoringDisable(true)
                .withName("SetNoteToSeek");
    }

    private Command timedDriveWithVelocity(ChassisSpeeds speeds, double seconds) {
        return DriveCommands.driveWithVelocity(() -> speeds, mDrive)
                .withTimeout(seconds)
                .withName("TimedDriveWithVelocity");
    }

    private Command driveToPoseWhileShooting(Pose2d pose, FinishedState finishedState) {
        if (pose == null || pose.equals(GeometryUtil.kPoseIdentity)) {
            return stopRoutine();
        }

        mPathBuilder.addPose(pose);
        return new ShootAndDriveToPose(
                mDrive, mShooter, mIndexer, mRobotState, () -> autoFlipPose(pose), finishedState);
    }

    private Command driveToPoseAndPreWarm(Pose2d pose) {
        if (pose == null || pose.equals(GeometryUtil.kPoseIdentity)) {
            return stopRoutine();
        }

        var setpoint = ShooterUtil.calculateSetpoint(FieldUtil.getDistanceToSpeaker(pose.getTranslation()));
        mPathBuilder.addPose(pose);
        return DriveCommands.driveToPose(
                        mDrive,
                        mRobotState,
                        () -> setpoint.applyReleaseAngle(autoFlipPose(pose)),
                        mRobotState::getPose2d,
                        kAutoTrapezoidalKinematicLimits,
                        true)
                .deadlineWith(index(), warmShooter(setpoint))
                .withName("DriveToPoseAndPreWarm");
    }

    private Command followPath(String pathName) {
        return followPath(pathName, false);
    }

    private Command followPath(String pathName, boolean resetPose) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            e.printStackTrace();
            return stopRoutine();
        }

        mPathBuilder.addPath(path);

        return parallel(
                        runOnce(() -> mDrive.setKinematicLimits(Constants.Drive.kFastKinematicLimits)),
                        resetPose ? resetPose(path.getPreviewStartingHolonomicPose()) : none())
                .andThen(AutoBuilder.followPath(path))
                .deadlineWith(index(), idleShooter())
                .withName("DrivePathPlannerPath");
    }

    private Command followChoreoPath(String pathName) {
        return followChoreoPath(pathName, false);
    }

    private Command followChoreoPath(String pathName, boolean resetPose) {
        var trajectory = Choreo.getTrajectory(pathName);
        if (trajectory == null) {
            return stopRoutine();
        }

        mPathBuilder.addPath(trajectory.getPoses());

        var eventMarkers = ChoreoEventMarker.loadFromFile(pathName);
        return new DriveChoreoTrajectory(mDrive, mRobotState, trajectory, eventMarkers, resetPose)
                .deadlineWith(index(), idleShooter())
                .withName("FollowChoreo");
    }

    private Pose2d getFirstPose(String pathName) {
        var trajectory = Choreo.getTrajectory(pathName);
        return trajectory == null ? GeometryUtil.kPoseIdentity : trajectory.getInitialPose();
    }

    private Command followChoreoPathAndSeekNote(String pathName) {
        var trajectory = Choreo.getTrajectory(pathName);
        if (trajectory == null) {
            return stopRoutine();
        }

        mPathBuilder.addPath(trajectory.getPoses());

        var eventMarkers = ChoreoEventMarker.loadFromFile(pathName);
        return new DriveAndSeekNote(
                        mDrive,
                        mRobotState,
                        new DriveChoreoTrajectory(mDrive, mRobotState, trajectory, eventMarkers, false),
                        mAutoNoteSeeker::getDetectedNoteToSeek,
                        kAutoTrapezoidalKinematicLimits)
                .deadlineWith(index(), idleShooter())
                .finallyDo(mAutoNoteSeeker::clear)
                .withName("FollowChoreoAndSeekNote");
    }

    private Command followChoreoPathAndPreWarm(String pathName) {
        return followChoreoPathAndPreWarm(pathName, false, false);
    }

    private Command followChoreoPathAndPreWarm(String pathName, boolean resetPose, boolean waitForNote) {
        var trajectory = Choreo.getTrajectory(pathName);
        if (trajectory == null) {
            return stopRoutine();
        }

        mPathBuilder.addPath(trajectory.getPoses());

        var shooterSetpoint = ShooterUtil.calculateSetpoint(FieldUtil.getDistanceToSpeaker(trajectory.getFinalPose()));
        var eventMarkers = ChoreoEventMarker.loadFromFile(pathName);
        return new DriveChoreoTrajectory(
                        mDrive, mRobotState, trajectory, eventMarkers, shooterSetpoint::applyReleaseAngle, resetPose)
                .deadlineWith(index(), warmShooter(shooterSetpoint, waitForNote))
                .withName("FollowChoreoAndPreWarm");
    }

    private Command followChoreoPathAndShoot(String pathName, boolean resetPose, double timeout) {
        var trajectory = Choreo.getTrajectory(pathName);
        if (trajectory == null) {
            return stopRoutine();
        }

        mPathBuilder.addPath(trajectory.getPoses());

        var shooterSetpoint = ShooterUtil.calculateSetpoint(FieldUtil.getDistanceToSpeaker(trajectory.getFinalPose()));
        var eventMarkers = ChoreoEventMarker.loadFromFile(pathName);
        return new DriveChoreoTrajectory(
                        mDrive, mRobotState, trajectory, eventMarkers, shooterSetpoint::applyReleaseAngle, resetPose)
                .deadlineWith(ShootCommands.shoot(mShooter, mIndexer, mRobotState)
                        .withTimeout(timeout)
                        .andThen(forceShoot(), index()))
                .withName("FollowChoreoAndShootWithTimeout");
    }

    private Command forceShoot() {
        return ShootCommands.forceShoot(mShooter, mIndexer, mRobotState);
    }

    private Command aimAndShoot() {
        return ShootCommands.aimAndShootInSpeaker(mShooter, mIndexer, mDrive, mRobotState);
    }

    private Command pauseDrive(String pathName) {
        return new PauseDrive(mDrive, mRobotState, () -> getFirstPose(pathName));
    }

    public AutonomousCommand demo() {
        var command = loggedSequence(
                        print("Starting demo"),
                        followPath("demo1", true),
                        aimAndShoot(),
                        driveToPoseAndPreWarm(new Pose2d(2.0, 1.0, Rotation2d.fromRadians(-Math.PI * 2.0 / 3.0))),
                        driveToPoseAndPreWarm(new Pose2d(10.0, 1.0, GeometryUtil.kRotationHalfPi)),
                        driveToPoseAndPreWarm(new Pose2d(2.0, 5.0, GeometryUtil.kRotationIdentity)),
                        followPath("demo2"),
                        driveToPoseAndPreWarm(new Pose2d(10.0, 5.0, GeometryUtil.kRotationMinusHalfPi)))
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
                        driveToPoseAndPreWarm(new Pose2d(
                                new Translation2d(2.46260666847229, 2.526517391204834),
                                new Rotation2d(2.2091161460921795))),
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

    public AutonomousCommand sourceSideMiddleThree() {
        var command = loggedSequence(
                        print("Started source side middle three auto"),
                        driveToPoseAndPreWarm(getFirstPose("SourceSideMiddleThree.1")),
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
                        followChoreoPathAndPreWarm("SourceFourUnderStage.1"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceFourUnderStage.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceFourUnderStage.3"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("SourceFourUnderStage.4"),
                        aimAndShoot())
                .withName("SourceFourUnderStageAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand fivePieceAmpAndMove() {
        var command = loggedSequence(
                        print("Started five piece amp and move auto"),
                        followChoreoPathAndShoot("FiveAmpMove.1", true, 0.6710751070512755),
                        forceShoot(),
                        followChoreoPathAndPreWarm("FiveAmpMove.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveAmpMove.3"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveAmpMove.4"),
                        aimAndShoot())
                .withName("FivePieceAmpAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand centerMove() {
        var command = loggedSequence(
                        print("Started center move auto"),
                        followChoreoPathAndShoot("CenterMove.1", true, 1.5),
                        forceShoot())
                .withName("CenterMoveAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand fiveMiddleMove() {
        var command = loggedSequence(
                        print("Started five middle move auto"),
                        driveToPoseWhileShooting(
                                getFirstPose("FiveMiddleToMiddle.3"), FinishedState.END_AFTER_SHOOTING_AND_MOVING),
                        followChoreoPathAndPreWarm("FiveMiddleToMiddle.3"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveMiddleToMiddle.4"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("FiveMiddleToMiddle.5"),
                        aimAndShoot())
                .withName("FiveMiddleMoveAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand centerMoveDTP() {
        var command = loggedSequence(
                        print("Started center move DTP auto"),
                        driveToPoseWhileShooting(getFirstPose("CenterMove.1"), FinishedState.END_AFTER_MOVING),
                        forceShoot())
                .withName("CenterMoveAuto");
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

    /* Phase 2 Autons */

    public AutonomousCommand source54CSeek() {
        var command = loggedSequence(
                        print("Started source 54C seek auto"),
                        driveToPoseWhileShooting(
                                getFirstPose("Source54CSeek.2"), FinishedState.END_AFTER_SHOOTING_AND_MOVING),
                        followChoreoPathAndSeekNote("Source54CSeek.2"),
                        pauseDrive("Source54CSeek.3"),
                        followChoreoPathAndPreWarm("Source54CSeek.3"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("Source54CSeek.4"),
                        pauseDrive("Source54CSeek.5"),
                        followChoreoPathAndPreWarm("Source54CSeek.5"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("Source54CSeek.6"),
                        aimAndShoot())
                .withName("Source45CSeekAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand amp123Amp() {
        var command = loggedSequence(
                        print("Started Amp123Seek auto"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("Amp123Amp.1"),
                        pauseDrive("Amp123Amp.2"),
                        followChoreoPathAndPreWarm("Amp123Amp.2"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("Amp123Amp.3"),
                        pauseDrive("Amp123Amp.4"),
                        followChoreoPathAndPreWarm("Amp123Amp.4"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("Amp123Amp.5"),
                        pauseDrive("Amp123Amp.6"),
                        followChoreoPathAndPreWarm("Amp123Amp.6"),
                        aimAndShoot())
                .withName("Amp 123 Amp Auto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand centerB342Stage() {
        var command = loggedSequence(
                        print("Started centerB342Stage auto"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("CenterB342Stage.1"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("CenterB342Stage.2"),
                        pauseDrive("CenterB342Stage.3"),
                        followChoreoPathAndPreWarm("CenterB342Stage.3"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("CenterB342Stage.4"),
                        pauseDrive("CenterB342Stage.5"),
                        followChoreoPathAndPreWarm("CenterB342Stage.5"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("CenterB342Stage.6"),
                        pauseDrive("CenterB342Stage.7"),
                        followChoreoPathAndPreWarm("CenterB342Stage.7"),
                        aimAndShoot())
                .withName("Center B342 Stage Auto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand source543Stage() {
        var command = loggedSequence(
                        print("Started source 543 stage auto"),
                        driveToPoseWhileShooting(
                                getFirstPose("Source543Stage.2"), FinishedState.END_AFTER_SHOOTING_AND_MOVING),
                        followChoreoPathAndSeekNote("Source543Stage.2"),
                        pauseDrive("Source543Stage.3"),
                        followChoreoPathAndPreWarm("Source543Stage.3"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("Source543Stage.4"),
                        pauseDrive("Source543Stage.5"),
                        followChoreoPathAndPreWarm("Source543Stage.5"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("Source543Stage.6"),
                        followChoreoPathAndPreWarm("Source543Stage.7"),
                        aimAndShoot())
                .withName("Source453stageAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand centerB231Center() {
        var command = loggedSequence(
                        print("Started centerB231Stage auto"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("CenterB231Center.1", false, false),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("CenterB231Center.2"),
                        pauseDrive("CenterB231Center.3"),
                        followChoreoPathAndPreWarm("CenterB231Center.3"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("CenterB231Center.4"),
                        pauseDrive("CenterB231Center.5"),
                        followChoreoPathAndPreWarm("CenterB231Center.5"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("CenterB231Center.6"),
                        pauseDrive("CenterB231Center.7"),
                        followChoreoPathAndPreWarm("CenterB231Center.7"),
                        aimAndShoot())
                .withName("Center B231 Stage Auto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand centerBA123Amp() {
        var command = loggedSequence(
                        print("Started centerB21AAmp auto"),
                        followChoreoPathAndPreWarm("CenterBA123Amp.1", true, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("CenterBA123Amp.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("CenterBA123Amp.3"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("CenterBA123Amp.4"),
                        pauseDrive("CenterBA123Amp.5"),
                        followChoreoPathAndPreWarm("CenterBA123Amp.5"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("CenterBA123Amp.6"),
                        pauseDrive("CenterBA123Amp.7"),
                        followChoreoPathAndPreWarm("CenterBA123Amp.7"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("CenterBA123Amp.8"))
                .withName("Center BA123 Amp Auto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }
}
