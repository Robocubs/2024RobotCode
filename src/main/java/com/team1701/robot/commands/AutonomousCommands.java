package com.team1701.robot.commands;

import java.util.Optional;
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
import com.team1701.robot.subsystems.shooter.Shooter.ShooterSpeeds;
import com.team1701.robot.util.FieldUtil;
import com.team1701.robot.util.ShooterUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
        NamedCommands.registerCommand("seekSB", setNoteToSeek(AutoNote.SB));
        NamedCommands.registerCommand("seekAB", setNoteToSeek(AutoNote.AB));
        NamedCommands.registerCommand("seekSR", setNoteToSeek(AutoNote.SR));
        NamedCommands.registerCommand("seekAR", setNoteToSeek(AutoNote.AR));
    }

    private Pose2d autoFlipPose(Pose2d pose) {
        return Configuration.isRedAlliance() ? GeometryUtil.flipX(pose, FieldConstants.kFieldLongLengthMeters) : pose;
    }

    private Rotation2d autoFlipRotation(Rotation2d rotation) {
        return Configuration.isRedAlliance() ? GeometryUtil.flipX(rotation) : rotation;
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

    private Command clearNoteToSeek() {
        return runOnce(() -> mAutoNoteSeeker.clear()).ignoringDisable(true).withName("ClearNoteToSeek");
    }

    private Command timedDriveWithVelocity(ChassisSpeeds speeds, double seconds) {
        return DriveCommands.driveWithVelocity(() -> speeds, mDrive)
                .withTimeout(seconds)
                .withName("TimedDriveWithVelocity");
    }

    private Command rotateToHeadingAndSeek(Supplier<Rotation2d> heading, Supplier<AutoNote> note) {
        return clearNoteToSeek()
                .andThen(setNoteToSeek(note.get()))
                .andThen(new DriveAndSeekNote(
                        mDrive,
                        mRobotState,
                        DriveCommands.rotateToFieldHeading(
                                mDrive,
                                heading,
                                mRobotState::getHeading,
                                () -> Rotation2d.fromDegrees(7),
                                kAutoTrapezoidalKinematicLimits,
                                false),
                        mAutoNoteSeeker::getDetectedNoteToSeek,
                        kAutoTrapezoidalKinematicLimits))
                .finallyDo(mAutoNoteSeeker::clear)
                .withName("RotateToHeadingAndSeek");
    }

    private Command rotateToFieldHeading(
            Supplier<Rotation2d> heading, Supplier<Rotation2d> tolerance, boolean finishAtRotation) {
        return DriveCommands.rotateToFieldHeading(
                mDrive, heading, mRobotState::getHeading, tolerance, kAutoTrapezoidalKinematicLimits, finishAtRotation);
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

        var setpoint = ShooterUtil.calculateShooterSetpoint(FieldUtil.getDistanceToSpeaker(pose.getTranslation()));
        mPathBuilder.addPose(pose);
        return DriveCommands.driveToPose(
                        mDrive,
                        mRobotState,
                        () -> setpoint.applyReleaseAngle(autoFlipPose(pose)),
                        mRobotState::getPose2d,
                        kAutoTrapezoidalKinematicLimits,
                        0.01,
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

    private Command driveBackPreWarmAndShoot(String pathName) {
        return loggedSequence(
                pauseDrive(pathName),
                followChoreoPathAndPreWarm(pathName),
                aimAndShoot(),
                runOnce(() -> mRobotState.setUseAutonFallback(false)));
    }

    private class NextNote {
        Pose2d notePose;

        public NextNote() {
            notePose = GeometryUtil.kPoseIdentity;
        }
    }

    private Command driveToNextPiece(AutoNote note) {
        NextNote nextNote = new NextNote();
        return loggedSequence(
                setNoteToSeek(note),
                runOnce(() -> nextNote.notePose = new Pose2d(
                                note.pose().getTranslation(),
                                mRobotState
                                        .getPose2d()
                                        .getTranslation()
                                        .minus(note.pose().getTranslation())
                                        .getAngle())
                        .transformBy(Constants.Robot.kIntakeToRobot)),
                new DriveAndSeekNote(
                        mDrive,
                        mRobotState,
                        new DriveToPose(
                                mDrive,
                                mRobotState,
                                () -> nextNote.notePose,
                                mRobotState::getPose2d,
                                kAutoTrapezoidalKinematicLimits,
                                0.1,
                                true,
                                true),
                        mAutoNoteSeeker::getDetectedNoteToSeek,
                        kAutoTrapezoidalKinematicLimits));
    }

    private Pose2d getFirstPose(String pathName) {
        var trajectory = Choreo.getTrajectory(pathName);
        return trajectory == null ? GeometryUtil.kPoseIdentity : trajectory.getInitialPose();
    }

    private Rotation2d getInitialVelocityHeading(String pathName) {
        var trajectory = Choreo.getTrajectory(pathName);
        if (trajectory == null) {
            return GeometryUtil.kRotationIdentity;
        }

        // Velocity of initial state is 0
        var firstState = trajectory.sample(Constants.kLoopPeriodSeconds);
        return new Rotation2d(firstState.velocityX, firstState.velocityY);
    }

    private Pose2d getFinalPose(String pathName) {
        var trajectory = Choreo.getTrajectory(pathName);
        return trajectory == null ? GeometryUtil.kPoseIdentity : trajectory.getFinalPose();
    }

    // private Command driveAndDynamicallyShoot(SequentialCommandGroup) {

    // }

    private Command preSpit() {
        return Commands.run(() -> {
            mShooter.setRollerSpeeds(new ShooterSpeeds(50));
            mShooter.setRotationAngle(Constants.Shooter.kShooterLowerLimit);
        });
    }

    private Command efficientlyPreWarmShootAndDrive(String pathName, String returnPath, AutoNote nextNote) {
        return race(
                        driveBackPreWarmAndShoot(pathName).andThen(followChoreoPathAndSeekNote(returnPath)),
                        waitSeconds(.5)
                                .andThen(either(
                                        idle(),
                                        runOnce(() -> mRobotState.setUseAutonFallback(true)),
                                        mRobotState::hasNote)))
                .andThen(either(driveToNextPiece(nextNote), none(), mRobotState::getUseAutonFallback));
    }

    private Command followChoreoPathAndSeekNote(String pathName) {
        var trajectory = Choreo.getTrajectory(pathName);
        if (trajectory == null) {
            return stopRoutine();
        }

        mPathBuilder.addPath(trajectory.getPoses());

        var eventMarkers = ChoreoEventMarker.loadFromFile(pathName);

        for (ChoreoEventMarker i : eventMarkers) {
            var c = i.command();
            if (c.getName() == "seekAB" || c.getName() == "seekSB") {
                var newName = c.getName();
                newName = newName.substring(0, newName.length() - 1) + "R";
                c.setName(newName);
            }
        }

        return clearNoteToSeek()
                .andThen(new DriveAndSeekNote(
                        mDrive,
                        mRobotState,
                        new DriveChoreoTrajectory(mDrive, mRobotState, trajectory, eventMarkers, false),
                        mAutoNoteSeeker::getDetectedNoteToSeek,
                        kAutoTrapezoidalKinematicLimits))
                .deadlineWith(index(), idleShooter())
                .finallyDo(mAutoNoteSeeker::clear)
                .withName("FollowChoreoAndSeekNote");
    }

    private Command followChoreoPathSeekNoteAndSpit(String pathName, double dropTime) {
        var trajectory = Choreo.getTrajectory(pathName);
        if (trajectory == null) {
            return stopRoutine();
        }

        mPathBuilder.addPath(trajectory.getPoses());

        var eventMarkers = ChoreoEventMarker.loadFromFile(pathName);
        var command = clearNoteToSeek()
                .andThen(new DriveAndSeekNote(
                        mDrive,
                        mRobotState,
                        new DriveChoreoTrajectory(mDrive, mRobotState, trajectory, eventMarkers, true),
                        mAutoNoteSeeker::getDetectedNoteToSeek,
                        kAutoTrapezoidalKinematicLimits))
                .finallyDo(mAutoNoteSeeker::clear);

        return Commands.parallel(preSpit().withTimeout(dropTime).andThen(spitNote()), command)
                .withName("followChoreoPathSeekNoteAndSpit");
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

        var shooterSetpoint =
                ShooterUtil.calculateShooterSetpoint(FieldUtil.getDistanceToSpeaker(trajectory.getFinalPose()));
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

        var shooterSetpoint =
                ShooterUtil.calculateShooterSetpoint(FieldUtil.getDistanceToSpeaker(trajectory.getFinalPose()));
        var eventMarkers = ChoreoEventMarker.loadFromFile(pathName);
        return new DriveChoreoTrajectory(
                        mDrive, mRobotState, trajectory, eventMarkers, shooterSetpoint::applyReleaseAngle, resetPose)
                .deadlineWith(ShootCommands.shoot(mShooter, mIndexer, mRobotState)
                        .withTimeout(timeout)
                        .andThen(forceShoot(), index()))
                .withName("FollowChoreoAndShootWithTimeout");
    }

    private Command followChoreoPathAndShootNoTimeout(String pathName, boolean resetPose) {
        var trajectory = Choreo.getTrajectory(pathName);
        if (trajectory == null) {
            return stopRoutine();
        }

        mPathBuilder.addPath(trajectory.getPoses());

        var shooterSetpoint =
                ShooterUtil.calculateShooterSetpoint(FieldUtil.getDistanceToSpeaker(trajectory.getFinalPose()));
        var eventMarkers = ChoreoEventMarker.loadFromFile(pathName);
        return Commands.deadline(
                        new DriveChoreoTrajectory(
                                mDrive,
                                mRobotState,
                                trajectory,
                                eventMarkers,
                                shooterSetpoint::applyReleaseAngle,
                                resetPose),
                        ShootCommands.shootForMoving(mShooter, mIndexer, mRobotState))
                .withName("FollowChoreoAndShootNoTimeout");
    }

    private Command forceShoot() {
        return ShootCommands.forceShoot(mShooter, mIndexer, mRobotState);
    }

    private Command aimAndShoot() {
        return ShootCommands.aimAndShootInSpeaker(mShooter, mIndexer, mDrive, mRobotState);
    }

    private Command pauseDrive(String pathName) {
        var moduleHeading = getInitialVelocityHeading(pathName);
        var robotPose = getFirstPose(pathName).getRotation();
        return new PauseDrive(
                        mDrive, mRobotState, () -> autoFlipRotation(moduleHeading), () -> autoFlipRotation(robotPose))
                .withName("PauseDrive");
    }

    private Command spitNote() {
        return ShootCommands.spitNote(mShooter, mIndexer, mRobotState);
    }

    private Command driveToNote(Supplier<AutoNote> note) {
        return DriveCommands.driveToNote(
                mDrive, mRobotState, () -> Optional.of(note.get().pose()), kAutoTrapezoidalKinematicLimits, false);
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

    public AutonomousCommand source4321CenterStage() {
        var command = loggedSequence(
                        print("Started source 4321 center stage auto"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("Source4321CenterStage.1"),
                        efficientlyPreWarmShootAndDrive(
                                "Source4321CenterStage.2", "Source4321CenterStage.3", AutoNote.M3),
                        efficientlyPreWarmShootAndDrive(
                                "Source4321CenterStage.4", "Source4321CenterStage.5", AutoNote.M2),
                        efficientlyPreWarmShootAndDrive(
                                "Source4321CenterStage.6", "Source4321CenterStage.7", AutoNote.M1))
                .withName("Source 4321 CenterStage Auto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand ampA123amp() {
        var command = loggedSequence(
                        print("Started AmpA123Amp auto"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("AmpA123Amp.1"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("AmpA123Amp.2"),
                        efficientlyPreWarmShootAndDrive("AmpA123Amp.3", "AmpA123Amp.4", AutoNote.M2),
                        efficientlyPreWarmShootAndDrive("AmpA123Amp.5", "AmpA123Amp.6", AutoNote.M3),
                        driveBackPreWarmAndShoot("AmpA123Amp.7"))
                .withName("Amp A123 Amp Auto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand greedyMiddle() {
        var command = loggedSequence(
                        print("Started greedy middle auto"),
                        followChoreoPathAndPreWarm("GreedyMiddle.1", true, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("GreedyMiddle.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("GreedyMiddle.3"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("GreedyMiddle.4"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("GreedyMiddle.5"),
                        efficientlyPreWarmShootAndDrive("GreedyMiddle.6", "GreedyMiddle.7", AutoNote.M2),
                        efficientlyPreWarmShootAndDrive("GreedyMiddle.8", "GreedyMiddle.9", AutoNote.M3))
                .withName("GreedyMiddleAuto");

        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand source54CSeek() {
        var command = loggedSequence(
                        print("Started source 54C seek auto"),
                        driveToPoseWhileShooting(
                                getFirstPose("Source54CSeek.2"), FinishedState.END_AFTER_SHOOTING_AND_MOVING),
                        followChoreoPathAndSeekNote("Source54CSeek.2"),
                        efficientlyPreWarmShootAndDrive("Source54CSeek.3", "Source54CSeek.4", AutoNote.M4),
                        driveBackPreWarmAndShoot("Source54CSeek.5"),
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
                        efficientlyPreWarmShootAndDrive("Amp123Amp.2", "Amp123Amp.3", AutoNote.M2),
                        efficientlyPreWarmShootAndDrive("Amp123Amp.4", "Amp123Amp.5", AutoNote.M3),
                        driveBackPreWarmAndShoot("Amp123Amp.6"))
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
                        efficientlyPreWarmShootAndDrive("CenterB342Stage.3", "CenterB342Stage.4", AutoNote.M4),
                        efficientlyPreWarmShootAndDrive("CenterB342Stage.5", "CenterB342Stage.6", AutoNote.M2),
                        driveBackPreWarmAndShoot("CenterB342Stage.7"))
                .withName("Center B342 Stage Auto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand source543Stage() {
        var command = loggedSequence(
                        print("Started source 543 stage auto"),
                        driveToPoseWhileShooting(
                                getFirstPose("Source543Stage.2"), FinishedState.END_AFTER_SHOOTING_AND_MOVING),
                        followChoreoPathAndSeekNote("Source543Stage.2"),
                        efficientlyPreWarmShootAndDrive("Source543Stage.3", "Source543Stage.4", AutoNote.M4),
                        efficientlyPreWarmShootAndDrive("Source543Stage.5", "Source543Stage.6", AutoNote.M3),
                        driveBackPreWarmAndShoot("Source543Stage.7"))
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
                        efficientlyPreWarmShootAndDrive("CenterB231Center.3", "CenterB231Center.4", AutoNote.M3),
                        efficientlyPreWarmShootAndDrive("CenterB231Center.5", "CenterB231Center.6", AutoNote.M1),
                        driveBackPreWarmAndShoot("CenterB231Center.7"))
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
                        efficientlyPreWarmShootAndDrive("CenterBA123Amp.5", "CenterBA123Amp.6", AutoNote.M2),
                        efficientlyPreWarmShootAndDrive("CenterBA123Amp.7", "CenterBA123Amp.8", AutoNote.M3))
                .withName("Center BA123 Amp Auto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand centerBC123center() {
        var command = loggedSequence(
                        print("Started center BC123 center auto"),
                        followChoreoPathAndPreWarm("CenterBC123Center.1", true, false),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("CenterBC123Center.2"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("CenterBC123Center.3"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("CenterBC123Center.4"),
                        efficientlyPreWarmShootAndDrive("CenterBC123Center.5", "CenterBC123Center.6", AutoNote.M2),
                        efficientlyPreWarmShootAndDrive("CenterBC123Center.7", "CenterBC123Center.8", AutoNote.M3))
                .withName("CenterBC123CenterAuto");

        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand source543Source() {
        var command = loggedSequence(
                        print("Started source 543 source auto"),
                        driveToPoseWhileShooting(
                                getFirstPose("Source543Source.2"), FinishedState.END_AFTER_SHOOTING_AND_MOVING),
                        followChoreoPathAndSeekNote("Source543Source.2"),
                        efficientlyPreWarmShootAndDrive("Source543Source.3", "Source543Source.4", AutoNote.M4),
                        efficientlyPreWarmShootAndDrive("Source543Source.5", "Source543Source.6", AutoNote.M3),
                        driveBackPreWarmAndShoot("Source543Source.7"))
                .withName("Source453sourceAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand inverseGreedy() {
        var command = loggedSequence(
                        print("Started inverse greedy auto"),
                        aimAndShoot(),
                        followChoreoPathAndSeekNote("InverseGreedy.1"),
                        efficientlyPreWarmShootAndDrive("InverseGreedy.2", "InverseGreedy.3", AutoNote.M1),
                        followChoreoPathAndPreWarm("InverseGreedy.4"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("InverseGreedy.5"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("InverseGreedy.6"),
                        aimAndShoot(),
                        followChoreoPathAndPreWarm("InverseGreedy.7"),
                        aimAndShoot())
                .withName("InverseGreedyAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand sourceDrop543source() {
        var command = loggedSequence(
                        print("Started source drop 543 source auto"),
                        followChoreoPathSeekNoteAndSpit("SourceDrop543Source.1", 0.7),
                        efficientlyPreWarmShootAndDrive("SourceDrop543Source.2", "SourceDrop543Source.3", AutoNote.M4),
                        efficientlyPreWarmShootAndDrive("SourceDrop543Source.4", "SourceDrop543Source.5", AutoNote.M3),
                        driveBackPreWarmAndShoot("SourceDrop543Source.6"),
                        rotateToHeadingAndSeek(
                                () -> autoFlipRotation(GeometryUtil.kRotationHalfPi),
                                () -> Configuration.isRedAlliance() ? AutoNote.SR : AutoNote.SB),
                        aimAndShoot())
                .withName("SourceDrop543SourceAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand sourceDrop54source() {
        var command = loggedSequence(
                        print("Started source drop 54 source auto"),
                        followChoreoPathSeekNoteAndSpit("SourceDrop543Source.1", 0.7),
                        efficientlyPreWarmShootAndDrive("SourceDrop543Source.2", "SourceDrop543Source.3", AutoNote.M4),
                        driveBackPreWarmAndShoot("SourceDrop543Source.4"),
                        rotateToHeadingAndSeek(
                                () -> autoFlipRotation(GeometryUtil.kRotationHalfPi),
                                () -> Configuration.isRedAlliance() ? AutoNote.SR : AutoNote.SB),
                        aimAndShoot())
                .withName("SourceDrop543SourceAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand ampDrop123Amp() {
        var command = loggedSequence(
                        print("Started Amp Drop 123 Amp"),
                        followChoreoPathSeekNoteAndSpit("AmpDrop123Amp.1", 1.0),
                        efficientlyPreWarmShootAndDrive("AmpDrop123Amp.2", "AmpDrop123Amp.3", AutoNote.M2),
                        efficientlyPreWarmShootAndDrive("AmpDrop123Amp.4", "AmpDrop123Amp.5", AutoNote.M3),
                        driveBackPreWarmAndShoot("AmpDrop123Amp.6"),
                        rotateToHeadingAndSeek(
                                () -> autoFlipRotation(GeometryUtil.kRotationMinusHalfPi),
                                () -> Configuration.isRedAlliance() ? AutoNote.AR : AutoNote.AB),
                        aimAndShoot())
                .withName("Amp Drop 123 Amp Auto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand ampDrop12Amp() {
        var command = loggedSequence(
                        print("Started Amp Drop 12 Amp"),
                        followChoreoPathSeekNoteAndSpit("AmpDrop123Amp.1", 1.0),
                        efficientlyPreWarmShootAndDrive("AmpDrop123Amp.2", "AmpDrop123Amp.3", AutoNote.M2),
                        driveBackPreWarmAndShoot("AmpDrop123Amp.4"),
                        rotateToHeadingAndSeek(
                                () -> autoFlipRotation(GeometryUtil.kRotationMinusHalfPi),
                                () -> Configuration.isRedAlliance() ? AutoNote.AR : AutoNote.AB),
                        aimAndShoot())
                .withName("Amp Drop 12 Amp Auto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand ampDrop231Amp() {
        var command = loggedSequence(
                        print("Started Amp Drop 231 Amp"),
                        followChoreoPathSeekNoteAndSpit("AmpDrop231Amp.1", 1.0),
                        efficientlyPreWarmShootAndDrive("AmpDrop231Amp.2", "AmpDrop231Amp.3", AutoNote.M3),
                        efficientlyPreWarmShootAndDrive("AmpDrop231Amp.4", "AmpDrop231Amp.5", AutoNote.M1),
                        driveBackPreWarmAndShoot("AmpDrop231Amp.6"),
                        rotateToHeadingAndSeek(
                                () -> autoFlipRotation(Rotation2d.fromRadians(1.0)),
                                () -> Configuration.isRedAlliance() ? AutoNote.SR : AutoNote.SB),
                        aimAndShoot())
                .withName("Amp Drop 231 Amp Auto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }

    public AutonomousCommand sourceDrop45source() {
        var command = loggedSequence(
                        print("Started source drop 45 source auto"),
                        followChoreoPathSeekNoteAndSpit("SourceDrop45Source.1", 0.7),
                        efficientlyPreWarmShootAndDrive("SourceDrop45Source.2", "SourceDrop45Source.3", AutoNote.M5),
                        driveBackPreWarmAndShoot("SourceDrop45Source.4"),
                        rotateToHeadingAndSeek(
                                () -> autoFlipRotation(GeometryUtil.kRotationHalfPi),
                                () -> Configuration.isRedAlliance() ? AutoNote.SR : AutoNote.SB),
                        aimAndShoot())
                .withName("SourceDrop45SourceAuto");
        return new AutonomousCommand(command, mPathBuilder.buildAndClear());
    }
}
