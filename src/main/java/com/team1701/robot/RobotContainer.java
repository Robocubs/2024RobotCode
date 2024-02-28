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
import com.team1701.lib.drivers.digitalinputs.DigitalIO;
import com.team1701.lib.drivers.digitalinputs.DigitalIOSensor;
import com.team1701.lib.drivers.digitalinputs.DigitalIOSim;
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
import com.team1701.robot.commands.ArmCommands;
import com.team1701.robot.commands.AutonomousCommands;
import com.team1701.robot.commands.DriveCommands;
import com.team1701.robot.commands.IndexCommand;
import com.team1701.robot.commands.IntakeCommand;
import com.team1701.robot.commands.IntakeCommands;
import com.team1701.robot.commands.ManualShoot;
import com.team1701.robot.commands.ShootCommands;
import com.team1701.robot.controls.StreamDeck;
import com.team1701.robot.controls.StreamDeck.StreamDeckButton;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.states.RobotState.ScoringMode;
import com.team1701.robot.subsystems.arm.Arm;
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
import edu.wpi.first.wpilibj2.command.Command;
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
    protected final Arm mArm;
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
        Optional<Arm> arm = Optional.empty();
        Optional<Climb> climb = Optional.empty();

        if (Configuration.getMode() != Mode.REPLAY) {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    drive = Optional.of(new Drive(
                            new GyroIOPigeon2(30),
                            new SwerveModuleIO[] {
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFx(10),
                                        TalonFxMotorFactory.createSteerMotorIOTalonFx(11),
                                        new EncoderIOAnalog(0),
                                        Rotation2d.fromRadians(-2.262)),
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFx(12),
                                        TalonFxMotorFactory.createSteerMotorIOTalonFx(13),
                                        new EncoderIOAnalog(1),
                                        Rotation2d.fromRadians(-3.069)),
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFx(14),
                                        TalonFxMotorFactory.createSteerMotorIOTalonFx(15),
                                        new EncoderIOAnalog(2),
                                        Rotation2d.fromRadians(-1.291)),
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFx(16),
                                        TalonFxMotorFactory.createSteerMotorIOTalonFx(17),
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
                                    Constants.Shooter.kShooterLeftUpperRollerMotorId, MotorUsage.SHOOTER_ROLLER, true),
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterLeftLowerRollerMotorId, MotorUsage.SHOOTER_ROLLER, false),
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterRotationMotorId, MotorUsage.ROTATION, false),
                            // new MotorIO() {},
                            // new MotorIO() {},
                            // new MotorIO() {},
                            // new MotorIO() {},
                            // new MotorIO() {},
                            new EncoderIORevThroughBore(Constants.Shooter.kShooterThroughBoreEncoderId, true)));

                    indexer = Optional.of(new Indexer(
                            SparkMotorFactory.createIndexerMotorIOSparkFlex(Constants.Indexer.kIndexerMotorId),
                            new DigitalIOSensor(Constants.Indexer.kIndexerEntranceSensorId, false),
                            new DigitalIOSensor(Constants.Indexer.kIndexerExitSensorId, true)));
                    intake = Optional.of(new Intake(
                            SparkMotorFactory.createIntakeMotorIOSparkFlex(Constants.Intake.kIntakeMotorId),
                            new DigitalIOSensor(Constants.Intake.kIntakeEntranceSensorId, true),
                            new DigitalIOSensor(Constants.Intake.kIntakeExitSensorId, true)));
                    arm = Optional.of(new Arm(
                            SparkMotorFactory.createArmClimbMotorIOSparkFlex(
                                    Constants.Arm.kRotationMotorId, MotorUsage.ROTATION, false),
                            new EncoderIORevThroughBore(Constants.Arm.kEncoderId, false)));
                    // climb = Optional.of(new Climb(
                    //         SparkMotorFactory.createArmClimbMotorIOSparkFlex(
                    //                 Constants.Winch.kLeftWinchId, MotorUsage.WINCH, true),
                    //         SparkMotorFactory.createArmClimbMotorIOSparkFlex(
                    //                 Constants.Winch.kRightWinchId, MotorUsage.WINCH, false))); //TODO: determine
                    // inversion
                    break;
                case SIMULATION_BOT:
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
                            new DetectorCameraIO[] {() -> Constants.Vision.kLimelightConfig}));

                    var rotationMotor = Shooter.createRotationMotorSim(DCMotor.getNeoVortex(1));
                    shooter = Optional.of(new Shooter(
                            Shooter.createRollerMotorSim(DCMotor.getNeoVortex(1)),
                            Shooter.createRollerMotorSim(DCMotor.getNeoVortex(1)),
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
                            new DigitalIOSim(() -> false),
                            new DigitalIOSim(() -> false)));
                    intake = Optional.of(new Intake(
                            new MotorIOSim(
                                    DCMotor.getNeoVortex(1),
                                    Constants.Intake.kReduction,
                                    0.001,
                                    Constants.kLoopPeriodSeconds),
                            new DigitalIOSim(() -> false),
                            new DigitalIOSim(() -> false)));
                    var armRotationMotorSim = Shooter.createRotationMotorSim(DCMotor.getNeoVortex(1));
                    arm = Optional.of(new Arm(armRotationMotorSim, Arm.createEncoderSim(armRotationMotorSim)));
                    climb = Optional.of(new Climb(
                            Climb.createWinchMotorIOSim(DCMotor.getNeoVortex(1)),
                            Climb.createWinchMotorIOSim(DCMotor.getNeoVortex(1))));
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

        mShooter = shooter.orElseGet(() -> new Shooter(
                new MotorIO() {},
                new MotorIO() {},
                new MotorIO() {},
                new MotorIO() {},
                new MotorIO() {},
                new EncoderIO() {}));

        mIndexer = indexer.orElseGet(() -> new Indexer(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));

        mIntake = intake.orElseGet(() -> new Intake(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));

        mArm = arm.orElseGet(() -> new Arm(new MotorIO() {}, new EncoderIO() {}));

        mClimb = climb.orElseGet(() -> new Climb(new MotorIO() {}, new MotorIO() {}));

        mLED = new LED(mRobotState);

        mRobotState.addSubsystems(this.mShooter, this.mIndexer, this.mIntake);

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

        /* DEFAULT COMMANDS */

        mDrive.setDefaultCommand(driveWithJoysticks(
                mDrive,
                mDrive::getFieldRelativeHeading,
                () -> -mDriverController.getLeftY(),
                () -> -mDriverController.getLeftX(),
                () -> -mDriverController.getRightX(),
                () -> mDriverController.rightBumper().getAsBoolean()
                        ? Constants.Drive.kSlowKinematicLimits
                        : Constants.Drive.kFastKinematicLimits));

        mIndexer.setDefaultCommand(new IndexCommand(mIndexer, () -> true));

        mIntake.setDefaultCommand(new IntakeCommand(mIntake, mRobotState));

        mShooter.setDefaultCommand(ShootCommands.idleShooterCommand(mShooter, mIndexer, mDrive, mRobotState));
        // mShooter.setDefaultCommand(Commands.run(
        //         () -> mShooter.setRotationAngle(
        //                 Rotation2d.fromRadians(Constants.Shooter.kTunableShooterAngleRadians.get())),
        //         mShooter));

        mArm.setDefaultCommand(ArmCommands.idleArmCommand(mArm, mRobotState));

        mClimb.setDefaultCommand(Commands.startEnd(mClimb::stop, () -> {}, mClimb));

        /* DRIVER CONTROLLER BINDINGS */

        mDriverController
                .b()
                .whileTrue(IntakeCommands.rejectAndDrive(mIntake, mIndexer, mDrive, () -> mRobotState.getHeading()));

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
                        mDrive, mRobotState, Constants.Drive.kSlowTrapezoidalKinematicLimits));

        // Drive while Rotating to Speaker
        mDriverController
                .leftBumper()
                .and(() -> mRobotState.getScoringMode().equals(ScoringMode.SPEAKER))
                .whileTrue(DriveCommands.slowlyDriveToSpeaker(
                        mDrive,
                        mRobotState::getSpeakerHeading,
                        mRobotState::getHeading,
                        () -> -mDriverController.getLeftY(),
                        () -> -mDriverController.getLeftX()));

        // Drive to Amp
        mDriverController
                .leftBumper()
                .and(() -> mRobotState.getScoringMode().equals(ScoringMode.AMP))
                .whileTrue(DriveCommands.driveToAmp(
                        mDrive, mRobotState::getPose2d, Constants.Drive.kFastTrapezoidalKinematicLimits));

        mDriverController.leftTrigger().whileTrue(swerveLock(mDrive));

        // Aim and Shoot
        mDriverController
                .rightTrigger()
                .and(() -> mRobotState.getScoringMode().equals(ScoringMode.SPEAKER))
                .whileTrue(ShootCommands.aimAndShootInSpeaker(mShooter, mIndexer, mDrive, mRobotState));

        // Shoot while Slowly Rotating
        // mDriverController
        //         .rightTrigger()
        //         .and(() -> mRobotState.getScoringMode().equals(ScoringMode.SPEAKER))
        //         .and(() -> mDrive.getKinematicLimits().equals(Constants.Drive.kSlowKinematicLimits))
        //         .whileTrue(ShootCommands.shoot(mShooter, mIndexer, mRobotState));

        // Amp Shot
        mDriverController
                .rightTrigger()
                .and(() -> mRobotState.getScoringMode().equals(ScoringMode.AMP))
                .whileTrue(ShootCommands.scoreInAmp(mShooter, mIndexer, mDrive, mArm, mRobotState));

        // mDriverController
        //         .a()
        //         .whileTrue((DriveCommands.driveToPose(
        //                 mDrive,
        //                 () -> new Pose2d(
        //                         new Translation2d(0.9079903960227966, 4.299035549163818),
        //                         Rotation2d.fromRadians(2.079867230915455)),
        //                 mRobotState::getPose2d,
        //                 Constants.Drive.kSlowKinematicLimits,
        //                 false)));

        // mDriverController
        //         .b()
        //         .whileTrue(startEnd(
        //                         () -> mDriverController.getHID().setRumble(RumbleType.kBothRumble, .5),
        //                         () -> mDriverController.getHID().setRumble(RumbleType.kBothRumble, 0))
        //                 .ignoringDisable(true));

        /* STREAMDECK BUTTONS */

        var stopIntakingCommand = runOnce(() -> IntakeCommands.stopIntake(mIntake, mIndexer), mIntake, mIndexer)
                .ignoringDisable(false)
                .withName("StreamDeckStopIntakingButton");
        var rejectCommand = run(() -> IntakeCommands.reverse(mIntake, mIndexer), mIndexer, mIntake)
                .ignoringDisable(false)
                .withName("StreamDeckRejectButton");
        var forwardCommand = run(
                        () -> {
                            mIntake.setForward();
                            mIndexer.setForwardLoad();
                        },
                        mIntake,
                        mIndexer)
                .ignoringDisable(false)
                .withName("StreamDeckForwardButton");
        var armUpCommand = startEnd(
                        () -> {
                            mArm.setArmUp();
                        },
                        () -> mArm.stop(),
                        mArm)
                .ignoringDisable(false)
                .withName("StreamDeckArmUpButton");
        var armDownCommand = startEnd(
                        () -> {
                            mArm.setArmDown();
                        },
                        () -> mArm.stop(),
                        mArm)
                .ignoringDisable(false)
                .withName("StreamDeckAmDownButton");
        var setSpeakerModeCommand =
                runOnce(() -> mRobotState.setScoringMode(ScoringMode.SPEAKER)).withName("SetSpeakerScoringMode");
        var setAmpModeCommand =
                runOnce(() -> mRobotState.setScoringMode(ScoringMode.AMP)).withName("SetAmpScoringMode");
        var setClimbModeCommand =
                runOnce(() -> mRobotState.setScoringMode(ScoringMode.CLIMB)).withName("SetClimbScoringMode");
        var armHomeCommand = run(
                        () -> {
                            mArm.rotateHome();
                        },
                        mArm)
                .ignoringDisable(false)
                .withName("StreamDeckArmHomeButton");
        var shooterUpCommand = run(
                        () -> {
                            mShooter.setShooterUp();
                        },
                        mShooter)
                .ignoringDisable(false)
                .withName("StreamDeckShooterUpCommand");
        var shooterDownCommand = run(
                        () -> {
                            mShooter.setShooterDown();
                        },
                        mShooter)
                .ignoringDisable(false)
                .withName("StreamDeckShooterDownCommand");
        var manualShootCommand = run(() -> new ManualShoot(mShooter, mIndexer))
                .ignoringDisable(false)
                .withName("StreamDeckShootCommand");
        var extendWinchCommand = run(
                        () -> {
                            mClimb.setPercentOutput(10);
                        },
                        mClimb)
                .ignoringDisable(false)
                .withName("StreamDeckExtendWinchCommand");
        var retractWinchCommand = run(
                        () -> {
                            mClimb.retractWinch(-10);
                        },
                        mClimb)
                .ignoringDisable(false)
                .withName("StreamDeckRetractWinchCommand");
        var stopShooterCommand =
                run(() -> Commands.idle(mShooter)).ignoringDisable(false).withName("StreamDeckStopShootCommand");

        mStreamDeck.configureButton(config -> config.add(
                        StreamDeckButton.kSpeakerModeButton, () -> mRobotState.getScoringMode() == ScoringMode.SPEAKER)
                .add(StreamDeckButton.kAmpModeButton, () -> mRobotState.getScoringMode() == ScoringMode.AMP)
                .add(StreamDeckButton.kClimbModeButton, () -> mRobotState.getScoringMode() == ScoringMode.CLIMB)
                .add(StreamDeckButton.kStopIntakeButton, stopIntakingCommand::isScheduled)
                .add(StreamDeckButton.kRejectButton, rejectCommand::isScheduled)
                .add(StreamDeckButton.kForwardButton, forwardCommand::isScheduled)
                .add(StreamDeckButton.kArmUpButton, armUpCommand::isScheduled)
                .add(StreamDeckButton.kArmDownButton, armDownCommand::isScheduled)
                .add(StreamDeckButton.kArmHomeButton, armHomeCommand::isScheduled)
                .add(StreamDeckButton.kShooterUpButton, shooterUpCommand::isScheduled)
                .add(StreamDeckButton.kShooterDownButton, shooterDownCommand::isScheduled)
                .add(StreamDeckButton.kShootButton, manualShootCommand::isScheduled)
                .add(StreamDeckButton.kExtendWinchButton, extendWinchCommand::isScheduled)
                .add(StreamDeckButton.kRetractWinchButton, retractWinchCommand::isScheduled)
                .add(StreamDeckButton.kStopShootButton, stopShooterCommand::isScheduled));

        mStreamDeck.button(StreamDeckButton.kStopIntakeButton).onTrue(stopIntakingCommand);
        // mStreamDeck.button(StreamDeckButton.kStopIntakeButton).onFalse(run(() -> stopIntakingCommand.cancel()));

        mStreamDeck.button(StreamDeckButton.kRejectButton).whileTrue(rejectCommand)
        // .onFalse(stopIntakingCommand)
        ;

        mStreamDeck.button(StreamDeckButton.kForwardButton).whileTrue(forwardCommand)
        // .onFalse(stopIntakingCommand)
        ;

        mStreamDeck.button(StreamDeckButton.kArmUpButton).whileTrue(armUpCommand);
        mStreamDeck.button(StreamDeckButton.kArmDownButton).whileTrue(armDownCommand);
        mStreamDeck.button(StreamDeckButton.kArmHomeButton).whileTrue(armHomeCommand);

        mStreamDeck
                .button(StreamDeckButton.kShooterUpButton)
                .whileTrue(shooterUpCommand)
                .onFalse(stopShooterCommand);

        /* */
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
        var shootAndBackupCommand = commands.shootAndBackup();
        mAutonomousPaths.put("Demo", demoCommand.path());
        mAutonomousPaths.put("Four Piece", fourPieceCommand.path());
        mAutonomousPaths.put("Shoot and Backup", shootAndBackupCommand.path());

        autonomousModeChooser.addDefaultOption("Demo", demoCommand.command());
        autonomousModeChooser.addOption("Four Piece", fourPieceCommand.command());
        autonomousModeChooser.addOption("Shoot and Backup", shootAndBackupCommand.command());

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
            return;
        }

        var path = mAutonomousPaths.get(name);

        if (Configuration.isRedAlliance()) {
            path = GeometryUtil.flipX(path, FieldConstants.kFieldLongLengthMeters);
        }

        Logger.recordOutput("Autonomous/PathPose2ds", path);
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
                    mShooter.zeroShooterRotation();
                    mArm.zeroArmRotation();
                    mDrive.zeroModules();
                })
                .ignoringDisable(true)
                .withName("ZeroEncoders");
    }

    public RobotState getRobotState() {
        return mRobotState;
    }
}
