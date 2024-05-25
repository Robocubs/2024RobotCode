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
import com.team1701.robot.commands.DriveCommands;
import com.team1701.robot.commands.DriveWithAssist;
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
                                        Rotation2d.fromRadians(-3.869)),
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

                    shooter = Optional.of(new Shooter(
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterRightUpperRollerMotorId,
                                    MotorUsage.SHOOTER_ROLLER,
                                    false),
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterRightLowerRollerMotorId, MotorUsage.SHOOTER_ROLLER, true),
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterRotationMotorId, MotorUsage.ROTATION, false),
                            new EncoderIORevThroughBore(Constants.Shooter.kShooterThroughBoreEncoderId, true),
                            mRobotState));

                    indexer = Optional.of(new Indexer(
                            SparkMotorFactory.createIndexerMotorIOSparkFlex(Constants.Indexer.kIndexerMotorId),
                            new DigitalIOSensor(Constants.Indexer.kIndexerEntranceSensorId, true),
                            new DigitalIOSensor(Constants.Indexer.kIndexerExitSensorId, false)));
                    intake = Optional.of(new Intake(
                            SparkMotorFactory.createIntakeMotorIOSparkFlex(Constants.Intake.kIntakeMotorId),
                            new DigitalIOSensor(Constants.Intake.kIntakeEntranceSensorId, true),
                            new DigitalIOSensor(Constants.Intake.kIntakeExitSensorId, true)));
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
                            Stream.of(
                                            Constants.Drive.kFrontLeftModuleEncoderOffset,
                                            Constants.Drive.kFrontRightModuleEncoderOffset,
                                            Constants.Drive.kBackLeftModuleEncoderOffset,
                                            Constants.Drive.kBackRightModuleEncoderOffset)
                                    .map(rotation -> SwerveModuleIO.createSim(
                                            DCMotor.getKrakenX60(1), DCMotor.getNEO(1), rotation))
                                    .limit(Constants.Drive.kNumModules)
                                    .toArray(SwerveModuleIO[]::new),
                            mRobotState);
                    gyroIO.setYawSupplier(
                            () -> simDrive.getVelocity().omegaRadiansPerSecond, Constants.kLoopPeriodSeconds);

                    drive = Optional.of(simDrive);

                    vision = Optional.of(new Vision(
                            mRobotState,
                            new AprilTagCameraIO[] {
                                () -> Constants.Vision.kFrontLeftCameraConfig,
                                () -> Constants.Vision.kFrontRightCameraConfig,
                                () -> Constants.Vision.kBackLeftCameraConfig,
                                () -> Constants.Vision.kBackRightCameraConfig,
                                () -> Constants.Vision.kSniperCameraConfig
                            },
                            // new AprilTagCameraIO[] {
                            //     new AprilTagCameraIOPhotonCamera(Constants.Vision.kFrontLeftCameraConfig),
                            //     new AprilTagCameraIOPhotonCamera(Constants.Vision.kFrontRightCameraConfig),
                            //     new AprilTagCameraIOPhotonCamera(Constants.Vision.kBackLeftCameraConfig),
                            //     new AprilTagCameraIOPhotonCamera(Constants.Vision.kBackRightCameraConfig),
                            //     new AprilTagCameraIOPhotonCamera(Constants.Vision.kSniperCameraConfig)
                            // },
                            new DetectorCameraIO[] {
                                new DetectorCameraIOSim(
                                        Constants.Vision.kLimelightConfig, noteSimulator::getDetectedObjects)
                            }));

                    var rotationMotor = Shooter.createRotationMotorSim(DCMotor.getNeoVortex(1));
                    shooter = Optional.of(new Shooter(
                            Shooter.createRollerMotorSim(DCMotor.getNeoVortex(1)),
                            Shooter.createRollerMotorSim(DCMotor.getNeoVortex(1)),
                            rotationMotor,
                            Shooter.createEncoderSim(rotationMotor),
                            mRobotState));

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
        } else {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    drive = Optional.of(new Drive(
                            new GyroIO() {},
                            Stream.of(
                                            Constants.Drive.kFrontLeftModuleEncoderOffset,
                                            Constants.Drive.kFrontRightModuleEncoderOffset,
                                            Constants.Drive.kBackLeftModuleEncoderOffset,
                                            Constants.Drive.kBackRightModuleEncoderOffset)
                                    .map(rotation -> new SwerveModuleIO(
                                            new MotorIO() {}, new MotorIO() {}, new EncoderIO() {}, rotation))
                                    .toArray(SwerveModuleIO[]::new),
                            mRobotState));
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

        mShooter = shooter.orElseGet(() ->
                new Shooter(new MotorIO() {}, new MotorIO() {}, new MotorIO() {}, new EncoderIO() {}, mRobotState));

        mIndexer = indexer.orElseGet(() -> new Indexer(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));

        mIntake = intake.orElseGet(() -> new Intake(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));

        mClimb = climb.orElseGet(() -> new Climb(new MotorIO() {}, new MotorIO() {}));

        mLED = new LED(mRobotState);

        mRobotState.addSubsystems(this.mDrive, this.mShooter, this.mIndexer, this.mIntake);

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

        teleopEnabled
                .and(() -> DriverStation.isFMSAttached() && Timer.getMatchTime() < 20)
                .whileTrue(rumbleController.rumblePulses(4));

        teleopEnabled.and(mIntake::hasNote).whileTrue(rumbleController.rumblePulses(2));

        mDriverController
                .y()
                .and(() -> !mRobotState.getDetectedNoteForPickup().isPresent() || mRobotState.hasNote())
                .onTrue(rumbleController.rumblePulses(2));

        /* DEFAULT COMMANDS */

        mDrive.setDefaultCommand(Commands.either(
                new DriveWithAssist(
                        mDrive,
                        mRobotState,
                        () -> -mDriverController.getLeftY(),
                        () -> -mDriverController.getLeftX(),
                        () -> -mDriverController.getRightX(),
                        () -> mDriverController.getHID().getRightBumper()
                                ? Constants.Drive.kSlowKinematicLimits
                                : Constants.Drive.kFastKinematicLimits),
                DriveCommands.driveWithJoysticks(
                        mDrive,
                        mDrive::getFieldRelativeHeading,
                        () -> -mDriverController.getLeftY(),
                        () -> -mDriverController.getLeftX(),
                        () -> -mDriverController.getRightX(),
                        () -> mDriverController.getHID().getRightBumper()
                                ? Constants.Drive.kSlowKinematicLimits
                                : Constants.Drive.kFastKinematicLimits),
                mDrive::useDriveAssist));

        // mDrive.setDefaultCommand(driveWithJoysticks(
        //         mDrive,
        //         mDrive::getFieldRelativeHeading,
        //         () -> -mDriverController.getLeftY(),
        //         () -> -mDriverController.getLeftX(),
        //         () -> -mDriverController.getRightX(),
        //         () -> mDriverController.getHID().getRightBumper()
        //                 ? Constants.Drive.kSlowKinematicLimits
        //                 : Constants.Drive.kFastKinematicLimits));

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
                .start()
                .onTrue(runOnce(() -> mDrive.zeroGyroscope(
                                Configuration.isBlueAlliance()
                                        ? GeometryUtil.kRotationIdentity
                                        : GeometryUtil.kRotationPi))
                        .withName("ZeroGyroscopeToHeading"));

        mDriverController
                .y()
                .whileTrue(DriveCommands.driveToNote(
                        mDrive, mRobotState, Constants.Drive.kFastTrapezoidalKinematicLimits));

        // Passing
        mDriverController
                .leftBumper()
                .and(() -> mRobotState.getScoringMode().equals(ScoringMode.SPEAKER))
                .whileTrue(DriveCommands.passANote(
                        mDrive,
                        mShooter,
                        mIndexer,
                        mRobotState,
                        () -> -mDriverController.getLeftY(),
                        () -> -mDriverController.getLeftX()));

        // Pass Low
        mDriverController
                .x()
                .whileTrue(ShootCommands.passLow(
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
                        mDrive,
                        mRobotState,
                        mRobotState::getPose2d,
                        Constants.Drive.kMediumTrapezoidalKinematicLimits,
                        false));

        mDriverController
                .povDown()
                .and(() -> mRobotState.getScoringMode().equals(ScoringMode.AMP))
                .whileTrue(DriveCommands.driveToAmp(
                        mDrive,
                        mRobotState,
                        mRobotState::getPose2d,
                        Constants.Drive.kMediumTrapezoidalKinematicLimits,
                        true));
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
                .whileTrue(ShootCommands.scoreInAmp(mShooter, mIndexer, mRobotState));

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
        var setDriveAssist = runOnce(() -> mDrive.toggleDriveAssist(), mDrive);
        var setClimbModeCommand = runOnce(() -> mRobotState.setScoringMode(ScoringMode.CLIMB))
                .ignoringDisable(true)
                .withName("SetClimbScoringMode");
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
        var stopShooterCommand = ShootCommands.stop(mShooter).withName("StreamDeckStopShootCommand");
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

        mStreamDeck.configureButton(config -> config.add(
                        StreamDeckButton.kSpeakerModeButton, () -> mRobotState.getScoringMode() == ScoringMode.SPEAKER)
                .add(StreamDeckButton.kAmpModeButton, () -> mRobotState.getScoringMode() == ScoringMode.AMP)
                .add(StreamDeckButton.kDriveAssistButton, mDrive::useDriveAssist)
                .add(StreamDeckButton.kStopIntakeButton, stopIntakingCommand::isScheduled)
                .add(StreamDeckButton.kRejectButton, rejectCommand::isScheduled)
                .add(StreamDeckButton.kForwardButton, forwardCommand::isScheduled)
                .add(StreamDeckButton.kLeftClimbButton, leftClimbCommand::isScheduled)
                .add(StreamDeckButton.kRightClimbButton, rightClimbCommand::isScheduled)
                .add(StreamDeckButton.kCenterClimbButton, centerClimbCommand::isScheduled)
                .add(StreamDeckButton.kRaiseShooterButton, shooterUpCommand::isScheduled)
                .add(StreamDeckButton.kLowerShooterButton, shooterDownCommand::isScheduled)
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
                .button(StreamDeckButton.kShootButton)
                .whileTrue(manualShootCommand)
                .onFalse(stopShooterCommand);
        mStreamDeck.button(StreamDeckButton.kStopShootButton).toggleOnTrue(stopShooterCommand);

        mStreamDeck.button(StreamDeckButton.kSpeakerModeButton).onTrue(setSpeakerModeCommand);
        mStreamDeck.button(StreamDeckButton.kAmpModeButton).onTrue(setAmpModeCommand);
        mStreamDeck.button(StreamDeckButton.kDriveAssistButton).onTrue(setDriveAssist);

        mStreamDeck
                .button(StreamDeckButton.kRaiseShooterButton)
                .whileTrue(shooterUpCommand)
                .onFalse(stopShooterCommand);
        mStreamDeck
                .button(StreamDeckButton.kLowerShooterButton)
                .whileTrue(shooterDownCommand)
                .onFalse(stopShooterCommand);

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

        var commands = new AutonomousCommands(mRobotState, mDrive, mShooter, mIndexer);
        var demoCommand = commands.demo();
        var shootAndBackupCommand = commands.shootAndBackup();
        var greedyMiddleCommand = commands.greedyMiddle();
        var source4321CenterStageCommand = commands.source4321CenterStage();
        var ampA123ampCommand = commands.ampA123amp();
        var source54CSeek = commands.source54CSeek();
        var amp123Amp = commands.amp123Amp();
        var centerB342Stage = commands.centerB342Stage();
        var source543Stage = commands.source543Stage();
        var centerB231Center = commands.centerB231Center();
        var centerBA123Amp = commands.centerBA123Amp();
        var centerBC123center = commands.centerBC123center();
        var source543source = commands.source543Source();
        var inverseGreedy = commands.inverseGreedy();

        mAutonomousPaths.put("Shoot and Backup", shootAndBackupCommand.path());
        mAutonomousPaths.put("Greedy Middle Auto", greedyMiddleCommand.path());
        mAutonomousPaths.put("Source 4321 CenterStage", source4321CenterStageCommand.path());
        mAutonomousPaths.put("Amp A123 Amp", ampA123ampCommand.path());
        mAutonomousPaths.put("Source 54C Seek", source54CSeek.path());
        mAutonomousPaths.put("Amp123Amp", amp123Amp.path());
        mAutonomousPaths.put("Center B342 Stage", centerB342Stage.path());
        mAutonomousPaths.put("Source 543 Stage", source543Stage.path());
        mAutonomousPaths.put("Center B231 Center", centerB231Center.path());
        mAutonomousPaths.put("Center BA123 Amp", centerBA123Amp.path());
        mAutonomousPaths.put("Center BC123 Center", centerBC123center.path());
        mAutonomousPaths.put("Source 543 Source", source543source.path());
        mAutonomousPaths.put("Inverse Greedy Auto", inverseGreedy.path());

        autonomousModeChooser.addDefaultOption("Shoot and Backup", shootAndBackupCommand.command());
        autonomousModeChooser.addOption("Greedy Middle Auto", greedyMiddleCommand.command());
        autonomousModeChooser.addOption("Source 4321 CenterStage", source4321CenterStageCommand.command());
        autonomousModeChooser.addOption("Amp A123 Amp", ampA123ampCommand.command());
        autonomousModeChooser.addOption("Source 54C Seek", source54CSeek.command());
        autonomousModeChooser.addOption("Amp123Amp", amp123Amp.command());
        autonomousModeChooser.addOption("Center B342 Stage", centerB342Stage.command());
        autonomousModeChooser.addOption("Source 543 Stage", source543Stage.command());
        autonomousModeChooser.addOption("Center B231 Center", centerB231Center.command());
        autonomousModeChooser.addOption("Center BA123 Amp", centerBA123Amp.command());
        autonomousModeChooser.addOption("Center BC123 Center", centerBC123center.command());
        autonomousModeChooser.addOption("Source 543 Source", source543source.command());
        autonomousModeChooser.addOption("Inverse Greedy Auto", inverseGreedy.command());

        // autonomousModeChooser.addOption(
        //         "Drive Characterization",
        //         CharacterizationCommands.runDriveCharacterization(mDrive)
        //                 .deadlineWith(Commands.idle(mShooter, mIndexer, mIntake))
        //                 .withName("DriveCharacterization"));
        // autonomousModeChooser.addOption(
        //         "Shooter Characterization",
        //         CharacterizationCommands.runShooterCharacterization(mShooter)
        //                 .deadlineWith(Commands.idle(mDrive, mIndexer, mIntake))
        //                 .withName("ShooterCharacterization"));

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
