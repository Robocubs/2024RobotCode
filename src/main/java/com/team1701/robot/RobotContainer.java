// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1701.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.team1701.lib.alerts.TriggeredAlert;
import com.team1701.lib.drivers.cameras.apriltag.AprilTagCameraIO;
import com.team1701.lib.drivers.cameras.apriltag.AprilTagCameraIOCubVision;
import com.team1701.lib.drivers.cameras.apriltag.AprilTagCameraIOPhotonCamera;
import com.team1701.lib.drivers.cameras.neural.DetectorCameraIO;
import com.team1701.lib.drivers.cameras.neural.DetectorCameraIOLimelight;
import com.team1701.lib.drivers.cameras.neural.DetectorCameraIOSim;
import com.team1701.lib.drivers.digitalinputs.DigitalIO;
import com.team1701.lib.drivers.digitalinputs.DigitalIOSensor;
import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderIOAnalog;
import com.team1701.lib.drivers.encoders.EncoderIORevThroughBore;
import com.team1701.lib.drivers.gyros.GyroIO;
import com.team1701.lib.drivers.gyros.GyroIOPigeon2;
import com.team1701.lib.drivers.gyros.GyroIOSim;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration.Mode;
import com.team1701.robot.commands.AutonomousCommands;
import com.team1701.robot.commands.CharacterizationCommands;
import com.team1701.robot.commands.DriveCommands;
import com.team1701.robot.commands.IntakeCommands;
import com.team1701.robot.commands.ShootCommands;
import com.team1701.robot.controls.RumbleController;
import com.team1701.robot.controls.StreamDeck;
import com.team1701.robot.controls.StreamDeck.StreamDeckButton;
import com.team1701.robot.simulation.NoteSimulator;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.RobotState.ScoringMode;
import com.team1701.robot.subsystems.climb.Climb;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.drive.SwerveModule.SwerveModuleIO;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import com.team1701.robot.subsystems.leds.LED;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.subsystems.vision.Vision;
import com.team1701.robot.util.SparkMotorFactory;
import com.team1701.robot.util.SparkMotorFactory.MotorUsage;
import com.team1701.robot.util.TalonFxMotorFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static com.team1701.robot.commands.DriveCommands.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class RobotContainer {
    private final RobotState mRobotState = new RobotState();
    protected final Drive mDrive;
    protected final Vision mVision;
    protected final Shooter mShooter;
    protected final Indexer mIndexer;
    protected final Intake mIntake;
    protected final Climb mClimb;
    protected final LED mLED;

    private final CommandXboxController mDriverController = new CommandXboxController(0);
    private final StreamDeck mStreamDeck = new StreamDeck();
    private final LoggedDashboardChooser<Command> autonomousModeChooser = new LoggedDashboardChooser<>("Auto Mode");
    private final Map<String, Pose2d[]> mAutonomousPaths = new HashMap<>();

    public RobotContainer() {
        Optional<Drive> drive = Optional.empty();
        Optional<Vision> vision = Optional.empty();
        Optional<Shooter> shooter = Optional.empty();
        Optional<Indexer> indexer = Optional.empty();
        Optional<Intake> intake = Optional.empty();
        Optional<Climb> climb = Optional.empty();

        if (Configuration.getMode() != Mode.REPLAY) {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    drive = Optional.of(new Drive(
                            new GyroIOPigeon2(30),
                            new SwerveModuleIO[] {
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFxFoc(10),
                                        TalonFxMotorFactory.createSteerMotorIOTalonFxFoc(11),
                                        new EncoderIOAnalog(0),
                                        Rotation2d.fromRadians(-2.262)),
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFxFoc(12),
                                        TalonFxMotorFactory.createSteerMotorIOTalonFxFoc(13),
                                        new EncoderIOAnalog(1),
                                        Rotation2d.fromRadians(-3.069)),
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFxFoc(14),
                                        TalonFxMotorFactory.createSteerMotorIOTalonFxFoc(15),
                                        new EncoderIOAnalog(2),
                                        Rotation2d.fromRadians(-1.291)),
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFxFoc(16),
                                        TalonFxMotorFactory.createSteerMotorIOTalonFxFoc(17),
                                        new EncoderIOAnalog(3),
                                        Rotation2d.fromRadians(-5.639)),
                            },
                            mRobotState));

                    vision = Optional.of(new Vision(
                            mRobotState,
                            new AprilTagCameraIO[] {
                                new AprilTagCameraIOCubVision(Constants.Vision.kFrontLeftCameraConfig),
                                new AprilTagCameraIOCubVision(Constants.Vision.kFrontRightCameraConfig),
                                new AprilTagCameraIOCubVision(Constants.Vision.kBackLeftCameraConfig),
                                new AprilTagCameraIOCubVision(Constants.Vision.kBackRightCameraConfig),
                                new AprilTagCameraIOCubVision(Constants.Vision.kSniperCameraConfig)
                            },
                            new DetectorCameraIO[] {new DetectorCameraIOLimelight(Constants.Vision.kLimelightConfig)}));

                    // TODO: update IDs
                    shooter = Optional.of(new Shooter(
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterRightUpperRollerMotorId,
                                    MotorUsage.SHOOTER_ROLLER,
                                    false),
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterRightLowerRollerMotorId, MotorUsage.SHOOTER_ROLLER, true),
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterRotationMotorId, MotorUsage.ROTATION, false),
                            new EncoderIORevThroughBore(Constants.Shooter.kShooterThroughBoreEncoderId, true)));

                    indexer = Optional.of(new Indexer(
                            SparkMotorFactory.createIndexerMotorIOSparkFlex(Constants.Indexer.kIndexerMotorId),
                            new DigitalIOSensor(Constants.Indexer.kIndexerEntranceSensorId, true),
                            new DigitalIOSensor(Constants.Indexer.kIndexerExitSensorId, false)));
                    intake = Optional.of(new Intake(
                            SparkMotorFactory.createIntakeMotorIOSparkFlex(Constants.Intake.kIntakeMotorId),
                            new DigitalIOSensor(Constants.Intake.kIntakeEntranceSensorId, false),
                            new DigitalIOSensor(Constants.Intake.kIntakeExitSensorId, false)));
                    climb = Optional.of(new Climb(
                            SparkMotorFactory.createArmClimbMotorIOSparkFlex(
                                    Constants.Climb.kLeftWinchId, MotorUsage.WINCH, true),
                            SparkMotorFactory.createArmClimbMotorIOSparkFlex(
                                    Constants.Climb.kRightWinchId, MotorUsage.WINCH, false)));
                    break;
                case SIMULATION_BOT:
                    var noteSimulator = new NoteSimulator(mRobotState, Constants.Vision.kLimelightConfig);
                    var noteSimulatorSensors = noteSimulator.getSensors();

                    var gyroIO = new GyroIOSim(mRobotState::getHeading);
                    var simDrive = new Drive(
                            gyroIO,
                            Stream.generate(() -> SwerveModuleIO.createSim(DCMotor.getKrakenX60(1), DCMotor.getNEO(1)))
                                    .limit(Constants.Drive.kNumModules)
                                    .toArray(SwerveModuleIO[]::new),
                            mRobotState);
                    gyroIO.setYawSupplier(
                            () -> simDrive.getVelocity().omegaRadiansPerSecond, Constants.kLoopPeriodSeconds);

                    drive = Optional.of(simDrive);

                    vision = Optional.of(new Vision(
                            mRobotState,
                            new AprilTagCameraIO[] {
                                new AprilTagCameraIOPhotonCamera(Constants.Vision.kFrontLeftCameraConfig),
                                new AprilTagCameraIOPhotonCamera(Constants.Vision.kFrontRightCameraConfig),
                                new AprilTagCameraIOPhotonCamera(Constants.Vision.kBackLeftCameraConfig),
                                new AprilTagCameraIOPhotonCamera(Constants.Vision.kBackRightCameraConfig),
                                new AprilTagCameraIOPhotonCamera(Constants.Vision.kSniperCameraConfig)
                            },
                            new DetectorCameraIO[] {
                                new DetectorCameraIOSim(
                                        Constants.Vision.kLimelightConfig, noteSimulator::getDetectedObjects)
                            }));

                    var rotationMotor = Shooter.createRotationMotorSim(DCMotor.getNeoVortex(1));
                    shooter = Optional.of(new Shooter(
                            Shooter.createRollerMotorSim(DCMotor.getNeoVortex(1)),
                            Shooter.createRollerMotorSim(DCMotor.getNeoVortex(1)),
                            rotationMotor,
                            Shooter.createEncoderSim(rotationMotor)));

                    indexer = Optional.of(new Indexer(
                            new MotorIOSim(
                                    DCMotor.getNeoVortex(1),
                                    Constants.Indexer.kIndexerReduction,
                                    0.001,
                                    Constants.kLoopPeriodSeconds),
                            noteSimulatorSensors.indexerEntranceSensor(),
                            noteSimulatorSensors.indexerExitSensor()));

                    intake = Optional.of(new Intake(
                            new MotorIOSim(
                                    DCMotor.getNeoVortex(1),
                                    Constants.Intake.kReduction,
                                    0.001,
                                    Constants.kLoopPeriodSeconds),
                            noteSimulatorSensors.intakeEntranceSensor(),
                            noteSimulatorSensors.intakeExitSensor()));

                    climb = Optional.of(new Climb(
                            Climb.createWinchMotorIOSim(DCMotor.getNeoVortex(1)),
                            Climb.createWinchMotorIOSim(DCMotor.getNeoVortex(1))));

                    noteSimulator.bindSubsystems(intake.get(), indexer.get(), shooter.get());

                    break;
                case SIMULATION_VISION:
                    vision = Optional.of(new Vision(
                            mRobotState,
                            new AprilTagCameraIO[] {
                                new AprilTagCameraIOCubVision(Constants.Vision.kFrontLeftCameraConfig),
                                new AprilTagCameraIOCubVision(Constants.Vision.kFrontRightCameraConfig),
                                new AprilTagCameraIOCubVision(Constants.Vision.kBackLeftCameraConfig),
                                new AprilTagCameraIOCubVision(Constants.Vision.kBackRightCameraConfig),
                                new AprilTagCameraIOCubVision(Constants.Vision.kSniperCameraConfig)
                            },
                            new DetectorCameraIO[] {() -> Constants.Vision.kLimelightConfig}));

                    // new DetectorCameraIO[] {new DetectorCameraIOLimelight(Constants.Vision.kLimelightConfig)}));
                    break;
                default:
                    break;
            }
        }

        mDrive = drive.orElseGet(() -> new Drive(
                new GyroIO() {},
                Stream.generate(() -> new SwerveModuleIO(
                                new MotorIO() {}, new MotorIO() {}, new EncoderIO() {}, new Rotation2d() {}))
                        .limit(Constants.Drive.kNumModules)
                        .toArray(SwerveModuleIO[]::new),
                mRobotState));

        mVision = vision.orElseGet(() -> new Vision(
                mRobotState,
                new AprilTagCameraIO[] {
                    () -> Constants.Vision.kFrontLeftCameraConfig,
                    () -> Constants.Vision.kFrontRightCameraConfig,
                    () -> Constants.Vision.kBackLeftCameraConfig,
                    () -> Constants.Vision.kBackRightCameraConfig,
                    () -> Constants.Vision.kSniperCameraConfig
                },
                new DetectorCameraIO[] {() -> Constants.Vision.kLimelightConfig}));

        mShooter = shooter.orElseGet(
                () -> new Shooter(new MotorIO() {}, new MotorIO() {}, new MotorIO() {}, new EncoderIO() {}));

        mIndexer = indexer.orElseGet(() -> new Indexer(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));

        mIntake = intake.orElseGet(() -> new Intake(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));

        mClimb = climb.orElseGet(() -> new Climb(new MotorIO() {}, new MotorIO() {}));

        mLED = new LED(mRobotState);

        mRobotState.addSubsystems(this.mShooter, this.mIndexer, this.mIntake);

        SmartDashboard.putData(mDrive);
        SmartDashboard.putData(mShooter);
        SmartDashboard.putData(mIndexer);
        SmartDashboard.putData(mIntake);
        SmartDashboard.putData(mClimb);

        setupControllerBindings();
        setupAutonomous();
        setupStateTriggers();
    }

    private void setupControllerBindings() {
        TriggeredAlert.error(
                "Driver controller disconnected",
                () -> !DriverStation.isJoystickConnected(
                                mDriverController.getHID().getPort())
                        || !DriverStation.getJoystickIsXbox(
                                mDriverController.getHID().getPort()));

        /* RUMBLE */

        var rumbleController = new RumbleController(mDriverController.getHID());

        var teleopEnabled = new Trigger(DriverStation::isTeleopEnabled);

        teleopEnabled
                .and(() -> DriverStation.isFMSAttached() && Timer.getMatchTime() < 30.5)
                .onTrue(rumbleController.rumblePulses(3).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        mDriverController
                .y()
                .and(() -> !mRobotState.getDetectedNoteForPickup().isPresent() || mRobotState.hasNote())
                .onTrue(rumbleController.rumblePulses(2));

        /* DEFAULT COMMANDS */

        mDrive.setDefaultCommand(driveWithJoysticks(
                mDrive,
                mDrive::getFieldRelativeHeading,
                () -> -mDriverController.getLeftY(),
                () -> -mDriverController.getLeftX(),
                () -> -mDriverController.getRightX(),
                () -> mDriverController.getHID().getRightBumper()
                        ? Constants.Drive.kSlowKinematicLimits
                        : Constants.Drive.kFastKinematicLimits));

        mIndexer.setDefaultCommand(IntakeCommands.idleIndexer(mIndexer));

        mIntake.setDefaultCommand(IntakeCommands.idleIntake(mIntake, mRobotState));

        mShooter.setDefaultCommand(ShootCommands.idleShooterCommand(mShooter, mRobotState));

        mClimb.setDefaultCommand(Commands.startEnd(mClimb::stop, () -> {}, mClimb)
                .andThen(idle(mClimb))
                .withName("IdleClimbCommand"));

        /* DRIVER CONTROLLER BINDINGS */

        mDriverController
                .b()
                .whileTrue(IntakeCommands.rejectAndDrive(
                        mIntake, mIndexer, mDrive, mDriverController, () -> mRobotState.getHeading()));

        mDriverController
                .x()
                .onTrue(runOnce(() -> mDrive.zeroGyroscope(
                                Configuration.isBlueAlliance()
                                        ? GeometryUtil.kRotationIdentity
                                        : GeometryUtil.kRotationPi))
                        .withName("ZeroGyroscopeToHeading"));

        mDriverController
                .y()
                .whileTrue(DriveCommands.driveToPiece(
                        mDrive, mRobotState, Constants.Drive.kFastTrapezoidalKinematicLimits, mDriverController));

        // Passing
        mDriverController
                .leftBumper()
                .and(() -> mRobotState.getScoringMode().equals(ScoringMode.SPEAKER))
                .whileTrue(ShootCommands.passANote(
                        mDrive,
                        mShooter,
                        mIndexer,
                        mRobotState,
                        () -> -mDriverController.getLeftY(),
                        () -> -mDriverController.getLeftX()));

        // Drive to Amp
        mDriverController
                .leftBumper()
                .and(() -> mRobotState.getScoringMode().equals(ScoringMode.AMP))
                .and(() -> mRobotState.inWing() || mRobotState.getPose2d().getY() > 6.4)
                .whileTrue(DriveCommands.driveToAmp(
                        mDrive, mRobotState::getPose2d, Constants.Drive.kMediumTrapezoidalKinematicLimits, false));

        mDriverController
                .povDown()
                .and(() -> mRobotState.getScoringMode().equals(ScoringMode.AMP))
                .whileTrue(DriveCommands.driveToAmp(
                        mDrive, mRobotState::getPose2d, Constants.Drive.kMediumTrapezoidalKinematicLimits, true));
        mDriverController.leftTrigger().whileTrue(swerveLock(mDrive));

        // Shoot (and optionally move)
        mDriverController
                .rightTrigger()
                .and(() -> mRobotState.getScoringMode().equals(ScoringMode.SPEAKER))
                .whileTrue(DriveCommands.shootAndMove(
                        mDrive,
                        mShooter,
                        mIndexer,
                        mRobotState,
                        () -> -mDriverController.getLeftY(),
                        () -> -mDriverController.getLeftX()));

        // Amp Shot
        mDriverController
                .rightTrigger()
                .and(() -> mRobotState.getScoringMode().equals(ScoringMode.AMP))
                .whileTrue(ShootCommands.scoreInAmp(mShooter, mIndexer, mDrive, mRobotState));

        /* STREAMDECK BUTTONS */

        var stopIntakingCommand = IntakeCommands.stopIntake(mIntake, mIndexer)
                .ignoringDisable(true)
                .withName("StreamDeckStopIntakeButton");

        var rejectCommand = IntakeCommands.reverse(mIntake, mIndexer).withName("StreamDeckRejectButton");

        var forwardCommand = startEnd(
                        () -> {
                            mIntake.setForward();
                            mIndexer.setForwardLoad();
                        },
                        () -> {
                            mIndexer.stop();
                            mIntake.stop();
                        },
                        mIntake,
                        mIndexer)
                .withName("StreamDeckForwardButton");
        var leftClimbCommand = Commands.runEnd(() -> mClimb.setLeftClimbPosition(), () -> mClimb.stop(), mClimb)
                .withName("StreamDeckLeftClimbCommand");
        var rightClimbCommand = Commands.runEnd(() -> mClimb.setRightClimbPosition(), () -> mClimb.stop(), mClimb)
                .withName("StreamDeckRightClimbCommand");
        var centerClimbCommand = Commands.runEnd(() -> mClimb.setMidClimbPosition(), () -> mClimb.stop(), mClimb)
                .withName("StreamDeckCenterClimbCommand");
        var setSpeakerModeCommand = runOnce(() -> mRobotState.setScoringMode(ScoringMode.SPEAKER))
                .ignoringDisable(true)
                .withName("SetSpeakerScoringMode");
        var setAmpModeCommand = runOnce(() -> mRobotState.setScoringMode(ScoringMode.AMP))
                .ignoringDisable(true)
                .withName("SetAmpScoringMode");
        var setClimbModeCommand = runOnce(() -> mRobotState.setScoringMode(ScoringMode.CLIMB))
                .ignoringDisable(true)
                .withName("SetClimbScoringMode");
        var shooterUpCommand = run(
                        () -> {
                            mShooter.setShooterUp();
                        },
                        mShooter)
                .withName("StreamDeckShooterUpCommand");
        var shooterDownCommand = startEnd(
                        () -> {
                            mShooter.setShooterDown();
                        },
                        () -> {
                            mShooter.stopRotation();
                        },
                        mShooter)
                .withName("StreamDeckShooterDownCommand");
        var manualShootCommand =
                ShootCommands.manualShoot(mShooter, mIndexer, mRobotState).withName("StreamDeckShootCommand");
        var extendWinchCommand = run(
                        () -> {
                            mClimb.extendWinch();
                        },
                        mClimb)
                .withName("StreamDeckExtendWinchCommand");
        var retractWinchCommand = run(
                        () -> {
                            mClimb.retractWinch();
                        },
                        mClimb)
                .withName("StreamDeckRetractWinchCommand");
        var stopShooterCommand = run(
                        () -> {
                            mShooter.stopRollers();
                            mShooter.stopRotation();
                        },
                        mShooter)
                .withName("StreamDeckStopShootCommand");

        mStreamDeck.configureButton(config -> config.add(
                        StreamDeckButton.kSpeakerModeButton, () -> mRobotState.getScoringMode() == ScoringMode.SPEAKER)
                .add(StreamDeckButton.kAmpModeButton, () -> mRobotState.getScoringMode() == ScoringMode.AMP)
                .add(StreamDeckButton.kClimbModeButton, () -> mRobotState.getScoringMode() == ScoringMode.CLIMB)
                .add(StreamDeckButton.kStopIntakeButton, stopIntakingCommand::isScheduled)
                .add(StreamDeckButton.kRejectButton, rejectCommand::isScheduled)
                .add(StreamDeckButton.kForwardButton, forwardCommand::isScheduled)
                .add(StreamDeckButton.kLeftClimbButton, leftClimbCommand::isScheduled)
                .add(StreamDeckButton.kRightClimbButton, rightClimbCommand::isScheduled)
                .add(StreamDeckButton.kCenterClimbButton, centerClimbCommand::isScheduled)
                .add(StreamDeckButton.kShooterUpButton, shooterUpCommand::isScheduled)
                .add(StreamDeckButton.kShooterDownButton, shooterDownCommand::isScheduled)
                .add(StreamDeckButton.kShootButton, manualShootCommand::isScheduled)
                .add(StreamDeckButton.kExtendWinchButton, extendWinchCommand::isScheduled)
                .add(StreamDeckButton.kRetractWinchButton, retractWinchCommand::isScheduled)
                .add(StreamDeckButton.kStopShootButton, stopShooterCommand::isScheduled));

        mStreamDeck.button(StreamDeckButton.kStopIntakeButton).toggleOnTrue(stopIntakingCommand);

        mStreamDeck.button(StreamDeckButton.kRejectButton).whileTrue(rejectCommand);
        mStreamDeck.button(StreamDeckButton.kForwardButton).whileTrue(forwardCommand);

        mStreamDeck.button(StreamDeckButton.kLeftClimbButton).onTrue(leftClimbCommand);
        mStreamDeck.button(StreamDeckButton.kRightClimbButton).onTrue(rightClimbCommand);
        mStreamDeck.button(StreamDeckButton.kCenterClimbButton).onTrue(centerClimbCommand);

        mStreamDeck
                .button(StreamDeckButton.kShooterUpButton)
                .whileTrue(shooterUpCommand)
                .onFalse(stopShooterCommand);
        mStreamDeck
                .button(StreamDeckButton.kShooterDownButton)
                .whileTrue(shooterDownCommand)
                .onFalse(stopShooterCommand);
        mStreamDeck
                .button(StreamDeckButton.kShootButton)
                .whileTrue(manualShootCommand)
                .onFalse(stopShooterCommand);
        mStreamDeck.button(StreamDeckButton.kStopShootButton).toggleOnTrue(stopShooterCommand);

        mStreamDeck.button(StreamDeckButton.kSpeakerModeButton).onTrue(setSpeakerModeCommand);
        mStreamDeck.button(StreamDeckButton.kAmpModeButton).onTrue(setAmpModeCommand);
        mStreamDeck.button(StreamDeckButton.kClimbModeButton).onTrue(setClimbModeCommand);

        mStreamDeck
                .button(StreamDeckButton.kExtendWinchButton)
                .whileTrue(extendWinchCommand)
                .onFalse(runOnce(() -> mClimb.stop()));
        mStreamDeck
                .button(StreamDeckButton.kRetractWinchButton)
                .whileTrue(retractWinchCommand)
                .onFalse(runOnce(() -> mClimb.stop()));

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void setupAutonomous() {
        AutoBuilder.configureHolonomic(
                mRobotState::getPose2d,
                mRobotState::resetPose,
                mDrive::getVelocity,
                mDrive::setVelocity,
                Constants.Drive.kPathFollowerConfig,
                Configuration::isRedAlliance,
                mDrive);

        PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));
        PathPlannerLogging.setLogActivePathCallback(
                poses -> Logger.recordOutput("PathPlanner/Path", poses.toArray(Pose2d[]::new)));

        // TODO: Create wrapper class for autonomous chooser
        var commands = new AutonomousCommands(mRobotState, mDrive, mShooter, mIndexer);
        var demoCommand = commands.demo();
        var fourPieceCommand = commands.fourPiece();
        var fourPieceAmpSideCommand = commands.fourPieceAmp();
        var sourceFourPieceTwoOneCommand = commands.sourceFourPieceTwoOne();
        var shootAndBackupCommand = commands.shootAndBackup();
        var middleToMiddleCommand = commands.middleToMiddle();
        var fiveMiddleToMiddleCommand = commands.fiveMiddleToMiddle();
        var greedyMiddleCommand = commands.greedyMiddle();
        var sourceMiddleThreeCommand = commands.sourceSideMiddleThree();
        var fivePieceAmpCommand = commands.fivePieceAmp();
        var straightToMiddleCommand = commands.straightToMiddle();
        var sourceFourUnderStage = commands.sourceFourUnderStage();
        var fiveAmpSideMove = commands.fivePieceAmpAndMove();

        mAutonomousPaths.put("Shoot and Backup", shootAndBackupCommand.path());
        mAutonomousPaths.put("Four Piece", fourPieceCommand.path());
        mAutonomousPaths.put("Four Piece Amp Side", fourPieceAmpSideCommand.path());
        mAutonomousPaths.put("Source Four Piece Two One Auto", sourceFourPieceTwoOneCommand.path());
        mAutonomousPaths.put("Middle To Middle Auto", middleToMiddleCommand.path());
        mAutonomousPaths.put("Five Middle To Middle Auto", fiveMiddleToMiddleCommand.path());
        mAutonomousPaths.put("Greedy Middle Auto", greedyMiddleCommand.path());
        mAutonomousPaths.put("Source Middle Three Auto", sourceMiddleThreeCommand.path());
        mAutonomousPaths.put("Five Piece Amp Auto", fivePieceAmpCommand.path());
        mAutonomousPaths.put("Straight To Middle", straightToMiddleCommand.path());
        mAutonomousPaths.put("Source Four Under Stage", sourceFourUnderStage.path());
        mAutonomousPaths.put("Five Piece Amp Move", fiveAmpSideMove.path());

        autonomousModeChooser.addDefaultOption("Shoot and Backup", shootAndBackupCommand.command());
        autonomousModeChooser.addOption("Four Piece", fourPieceCommand.command());
        autonomousModeChooser.addOption("Four Piece Amp Side", fourPieceAmpSideCommand.command());
        autonomousModeChooser.addOption("Source Four Piece Two One Auto", sourceFourPieceTwoOneCommand.command());
        autonomousModeChooser.addOption("Middle To Middle Auto", middleToMiddleCommand.command());
        autonomousModeChooser.addOption("Five Middle To Middle Auto", fiveMiddleToMiddleCommand.command());
        autonomousModeChooser.addOption("Greedy Middle Auto", greedyMiddleCommand.command());
        autonomousModeChooser.addOption("Source Middle Three Auto", sourceMiddleThreeCommand.command());
        autonomousModeChooser.addOption("Five Piece Amp Auto", fivePieceAmpCommand.command());
        autonomousModeChooser.addOption("Straight To Middle", straightToMiddleCommand.command());
        autonomousModeChooser.addOption("Source Four Under Stage", sourceFourUnderStage.command());
        autonomousModeChooser.addOption("Five Piece Amp Move", fiveAmpSideMove.command());

        autonomousModeChooser.addOption(
                "Drive Characterization",
                CharacterizationCommands.runDriveCharacterization(mDrive)
                        .deadlineWith(Commands.idle(mShooter, mIndexer, mIntake))
                        .withName("DriveCharacterization"));
        autonomousModeChooser.addOption(
                "Shooter Characterization",
                CharacterizationCommands.runShooterCharacterization(mShooter)
                        .deadlineWith(Commands.idle(mDrive, mIndexer, mIntake))
                        .withName("ShooterCharacterization"));

        if (Configuration.getMode() == Mode.SIMULATION) {
            mAutonomousPaths.put("Demo", demoCommand.path());
            autonomousModeChooser.addOption("Demo", demoCommand.command());
        }

        autonomousModeChooser.getSendableChooser().onChange(this::logAutonomousPath);

        var logAutonomousPathCommand =
                runOnce(this::logAutonomousPath).ignoringDisable(true).withName("LogAutonomousPath");
        new Trigger(Configuration::isBlueAlliance)
                .onTrue(logAutonomousPathCommand)
                .onFalse(logAutonomousPathCommand);

        logAutonomousPath();
    }

    private void logAutonomousPath() {
        logAutonomousPath(autonomousModeChooser.getSendableChooser().getSelected());
    }

    private void logAutonomousPath(String name) {
        if (!mAutonomousPaths.containsKey(name)) {
            Logger.recordOutput("Autonomous/PathPose2d", new Pose2d[] {});
            mRobotState.setPath();
            return;
        }

        var path = mAutonomousPaths.get(name);

        if (Configuration.isRedAlliance()) {
            path = GeometryUtil.flipX(path, FieldConstants.kFieldLongLengthMeters);
        }

        Logger.recordOutput("Autonomous/PathPose2ds", path);
        mRobotState.setPath(path);
    }

    private void setupStateTriggers() {
        var teleopTrigger = new Trigger(DriverStation::isTeleopEnabled);
        teleopTrigger.onTrue(
                runOnce(() -> mDrive.zeroGyroscope(mRobotState.getHeading())).withName("ZeroGyroscopeToPose"));
    }

    public Optional<Command> getAutonomousCommand() {
        return Optional.ofNullable(autonomousModeChooser.get());
    }

    public Command getZeroCommand() {
        return runOnce(() -> {
                    mShooter.zeroShooter();
                    mDrive.zeroModules();
                })
                .ignoringDisable(true)
                .withName("ZeroEncoders");
    }

    public RobotState getRobotState() {
        return mRobotState;
    }

    public CommandXboxController getDriverController() {
        return mDriverController;
    }
}
