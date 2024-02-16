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
import com.team1701.lib.drivers.cameras.neural.DetectorCameraIOLimelight;
import com.team1701.lib.drivers.digitalinputs.DigitalIO;
import com.team1701.lib.drivers.digitalinputs.DigitalIOSim;
import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderIOAnalog;
import com.team1701.lib.drivers.gyros.GyroIO;
import com.team1701.lib.drivers.gyros.GyroIOPigeon2;
import com.team1701.lib.drivers.gyros.GyroIOSim;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration.Mode;
import com.team1701.robot.commands.AutonomousCommands;
import com.team1701.robot.commands.DriveCommands;
import com.team1701.robot.commands.IndexCommand;
import com.team1701.robot.commands.IntakeCommand;
import com.team1701.robot.commands.ShootCommands;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.drive.SwerveModule.SwerveModuleIO;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.subsystems.vision.Vision;
import com.team1701.robot.util.TalonFxMotorFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static com.team1701.robot.commands.DriveCommands.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class RobotContainer {
    private final RobotState mRobotState = new RobotState();
    public final Drive mDrive;
    public final Shooter mShooter;
    public final Vision mVision;
    private final Indexer mIndexer;
    private final Intake mIntake;

    private final CommandXboxController mDriverController = new CommandXboxController(0);
    private final LoggedDashboardChooser<Command> autonomousModeChooser = new LoggedDashboardChooser<>("Auto Mode");
    private final Map<String, Pose2d[]> mAutonomousPaths = new HashMap<>();

    public RobotContainer() {
        Optional<Drive> drive = Optional.empty();
        Optional<Vision> vision = Optional.empty();
        Optional<Shooter> shooter = Optional.empty();
        Optional<Indexer> indexer = Optional.empty();
        Optional<Intake> intake = Optional.empty();

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

                    // TODO: update IDs
                    // shooter = Optional.of(new Shooter(
                    //         SparkMotorFactory.createShooterMotorIOSparkFlex(
                    //                 Constants.Shooter.kShooterRightUpperRollerMotorId, ShooterMotorUsage.ROLLER),
                    //         SparkMotorFactory.createShooterMotorIOSparkFlex(
                    //                 Constants.Shooter.kShooterRightLowerRollerMotorId, ShooterMotorUsage.ROLLER),
                    //         SparkMotorFactory.createShooterMotorIOSparkFlex(
                    //                 Constants.Shooter.kShooterLeftUpperRollerMotorId, ShooterMotorUsage.ROLLER),
                    //         SparkMotorFactory.createShooterMotorIOSparkFlex(
                    //                 Constants.Shooter.kShooterLeftLowerRollerMotorId, ShooterMotorUsage.ROLLER),
                    //         SparkMotorFactory.createShooterMotorIOSparkFlex(
                    //                 Constants.Shooter.kShooterRotationMotorId, ShooterMotorUsage.ROTATION),
                    //         new EncoderIOAnalog(Constants.Shooter.kShooterThroughBoreEncoderId)));
                    // indexer = Optional.of(new Indexer(
                    //         SparkMotorFactory.createIndexerMotorIOSparkFlex(Constants.Indexer.kIndexerMotorId),
                    //         new DigitalIOSensor(Constants.Indexer.kIndexerEntranceSensorId),
                    //         new DigitalIOSensor(Constants.Indexer.kIndexerExitSensorId)));
                    // intake = Optional.of(new Intake(
                    //         SparkMotorFactory.createIntakeMotorIOSparkFlex(Constants.Intake.kIntakeMotorId),
                    //         new DigitalIOSensor(Constants.Intake.kIntakeEntranceSensorId),
                    //         new DigitalIOSensor(Constants.Intake.kIntakeExitSensorId)));

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
                                    Constants.Indexer.kReduction,
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
                    break;
                default:
                    break;
            }

            vision = Optional.of(new Vision(
                    mRobotState,
                    new AprilTagCameraIOCubVision(Constants.Vision.kFrontLeftCameraConfig),
                    new AprilTagCameraIOCubVision(Constants.Vision.kFrontRightCameraConfig),
                    new AprilTagCameraIOCubVision(Constants.Vision.kBackLeftCameraConfig),
                    new AprilTagCameraIOCubVision(Constants.Vision.kBackRightCameraConfig)));
            vision.ifPresent(
                    v -> v.constructDetectorCameras(new DetectorCameraIOLimelight(Constants.Vision.kLimelightConfig)));
        }

        this.mDrive = drive.orElseGet(() -> new Drive(
                new GyroIO() {},
                Stream.generate(() -> new SwerveModuleIO(
                                new MotorIO() {}, new MotorIO() {}, new EncoderIO() {}, new Rotation2d() {}))
                        .limit(Constants.Drive.kNumModules)
                        .toArray(SwerveModuleIO[]::new),
                mRobotState));

        this.mVision = vision.orElseGet(() -> new Vision(
                mRobotState,
                new AprilTagCameraIO() {},
                new AprilTagCameraIO() {},
                new AprilTagCameraIO() {},
                new AprilTagCameraIO() {}));

        this.mShooter = shooter.orElseGet(() -> new Shooter(
                new MotorIO() {},
                new MotorIO() {},
                new MotorIO() {},
                new MotorIO() {},
                new MotorIO() {},
                new EncoderIO() {}));

        this.mIndexer = indexer.orElseGet(() -> new Indexer(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));

        this.mIntake = intake.orElseGet(() -> new Intake(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));

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

        mDrive.setDefaultCommand(driveWithJoysticks(
                mDrive,
                () -> -mDriverController.getLeftY(),
                () -> -mDriverController.getLeftX(),
                () -> -mDriverController.getRightX(),
                () -> mDriverController.rightTrigger().getAsBoolean()
                        ? Constants.Drive.kSlowKinematicLimits
                        : Constants.Drive.kFastKinematicLimits));

        // TODO: update should load when intake is completed
        mIndexer.setDefaultCommand(new IndexCommand(mIndexer, mRobotState::hasNote));

        mIntake.setDefaultCommand(new IntakeCommand(mIntake, mRobotState));

        mShooter.setDefaultCommand(ShootCommands.idleShooterCommand(mShooter, mRobotState));

        mDriverController
                .x()
                .onTrue(runOnce(() -> mDrive.zeroGyroscope(
                                Configuration.isBlueAlliance()
                                        ? GeometryUtil.kRotationIdentity
                                        : GeometryUtil.kRotationPi))
                        .withName("ZeroGyroscopeToHeading"));
        mDriverController
                .rightBumper()
                .whileTrue(
                        DriveCommands.rotateToSpeaker(mDrive, mRobotState, Constants.Drive.kFastKinematicLimits, true));
        mDriverController.leftBumper().whileTrue(swerveLock(mDrive));

        mDriverController.leftTrigger().onTrue(ShootCommands.aimAndShoot(mShooter, mIndexer, mDrive, mRobotState));

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
        mAutonomousPaths.put("Demo", demoCommand.path());
        mAutonomousPaths.put("Four Piece", fourPieceCommand.path());

        autonomousModeChooser.addDefaultOption("Demo", demoCommand.command());
        autonomousModeChooser.addOption("Four Piece", fourPieceCommand.command());
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

        Logger.recordOutput("Autonomous/PathPose2d", path);
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
                    // TODO: uncomment line
                    //mShooter.zeroShooterRotation();
                    mDrive.zeroModules();
                })
                .ignoringDisable(true)
                .withName("ZeroEncoders");
    }

    public RobotState getRobotState() {
        return mRobotState;
    }
}
