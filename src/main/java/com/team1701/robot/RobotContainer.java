// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1701.robot;

import java.util.Optional;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.team1701.lib.alerts.TriggeredAlert;
import com.team1701.lib.drivers.cameras.AprilTagCameraIOCubVision;
import com.team1701.lib.drivers.digitalinputs.DigitalIO;
import com.team1701.lib.drivers.digitalinputs.DigitalIOSensor;
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
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.drive.SwerveModule.SwerveModuleIO;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.subsystems.vision.Vision;
import com.team1701.robot.util.SparkMotorFactory;
import com.team1701.robot.util.SparkMotorFactory.ShooterMotorUsage;
import com.team1701.robot.util.TalonFxMotorFactory;
import edu.wpi.first.math.geometry.Pose2d;
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
    private final Drive mDrive;
    private final Shooter mShooter;
    private final Indexer mIndexer;
    private final Intake mIntake;
    public final Vision mVision;

    private final RobotState mRobotState = new RobotState();

    private final CommandXboxController mDriverController = new CommandXboxController(0);
    private final LoggedDashboardChooser<Command> autonomousModeChooser = new LoggedDashboardChooser<>("Auto Mode");

    public RobotContainer() {
        Optional<Drive> drive = Optional.empty();
        Optional<Vision> vision = Optional.empty();
        Optional<Shooter> shooter = Optional.empty();
        Optional<Indexer> indexer = Optional.empty();
        Optional<Intake> intake = Optional.empty();

        if (Configuration.getMode() != Mode.REPLAY) {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    shooter = Optional.of(new Shooter(
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterRightUpperRollerMotorId, ShooterMotorUsage.ROLLER),
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterRightLowerRollerMotorId, ShooterMotorUsage.ROLLER),
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterLeftUpperRollerMotorId, ShooterMotorUsage.ROLLER),
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterLeftLowerRollerMotorId, ShooterMotorUsage.ROLLER),
                            SparkMotorFactory.createShooterMotorIOSparkFlex(
                                    Constants.Shooter.kShooterRotationMotorId, ShooterMotorUsage.ROTATION),
                            new EncoderIOAnalog(Constants.Shooter.kShooterThroughBoreEncoderId)));
                    indexer = Optional.of(new Indexer(
                            SparkMotorFactory.createIndexerMotorIOSparkFlex(Constants.Indexer.kIndexerMotorId),
                            new DigitalIOSensor(Constants.Indexer.kIndexerEntranceSensorId),
                            new DigitalIOSensor(Constants.Indexer.kIndexerExitSensorId)));
                    intake = Optional.of(new Intake(
                            SparkMotorFactory.createIntakeMotorIOSparkFlex(Constants.Intake.kIntakeMotorId),
                            new DigitalIOSensor(Constants.Intake.kIntakeEntranceSensorId),
                            new DigitalIOSensor(Constants.Intake.kIntakeExitSensorId)));
                    drive = Optional.of(new Drive(
                            new GyroIOPigeon2(30),
                            new SwerveModuleIO[] {
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFx(10),
                                        SparkMotorFactory.createSteerMotorIOSparkMax(11),
                                        new EncoderIOAnalog(0)),
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFx(12),
                                        SparkMotorFactory.createSteerMotorIOSparkMax(13),
                                        new EncoderIOAnalog(1)),
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFx(16),
                                        SparkMotorFactory.createSteerMotorIOSparkMax(17),
                                        new EncoderIOAnalog(3)),
                                new SwerveModuleIO(
                                        TalonFxMotorFactory.createDriveMotorIOTalonFx(14),
                                        SparkMotorFactory.createSteerMotorIOSparkMax(15),
                                        new EncoderIOAnalog(2)),
                            },
                            mRobotState));

                    break;
                case SIMULATION_BOT:
                    var rotationMotor = Shooter.createRotationMotorSim(DCMotor.getNeoVortex(1));
                    shooter = Optional.of(new Shooter(
                            Shooter.createRollerMotorSim(DCMotor.getNeoVortex(1)),
                            Shooter.createRollerMotorSim(DCMotor.getNeoVortex(1)),
                            Shooter.createRollerMotorSim(DCMotor.getNeoVortex(1)),
                            Shooter.createRollerMotorSim(DCMotor.getNeoVortex(1)),
                            rotationMotor,
                            Shooter.createEncoderSim(rotationMotor)));

                    indexer = Optional.of(new Indexer(
                            new MotorIOSim(DCMotor.getNeoVortex(1), 1, 0.001, Constants.kLoopPeriodSeconds),
                            new DigitalIOSim(() -> false),
                            new DigitalIOSim(() -> false)));

                    intake = Optional.of(new Intake(
                            new MotorIOSim(DCMotor.getNeoVortex(1), 1, 0.001, Constants.kLoopPeriodSeconds),
                            new DigitalIOSim(() -> false),
                            new DigitalIOSim(() -> false)));

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

                    break;
                default:
                    break;
            }

            vision = Optional.of(new Vision(
                    mRobotState,
                    new AprilTagCameraIOCubVision(
                            Constants.Vision.kFrontLeftCameraName, Constants.Vision.kFrontLeftCameraID),
                    new AprilTagCameraIOCubVision(
                            Constants.Vision.kFrontRightCameraName, Constants.Vision.kFrontRightCameraID),
                    new AprilTagCameraIOCubVision(
                            Constants.Vision.kBackLeftCameraName, Constants.Vision.kBackLeftCameraID),
                    new AprilTagCameraIOCubVision(
                            Constants.Vision.kBackRightCameraName, Constants.Vision.kBackLeftCameraID)));
        }

        this.mVision = vision.orElseGet(() -> new Vision(
                mRobotState,
                new AprilTagCameraIOCubVision(
                        Constants.Vision.kFrontLeftCameraName, Constants.Vision.kFrontLeftCameraID),
                new AprilTagCameraIOCubVision(
                        Constants.Vision.kFrontRightCameraName, Constants.Vision.kFrontRightCameraID),
                new AprilTagCameraIOCubVision(Constants.Vision.kBackLeftCameraName, Constants.Vision.kBackLeftCameraID),
                new AprilTagCameraIOCubVision(
                        Constants.Vision.kBackRightCameraName, Constants.Vision.kBackLeftCameraID)));

        this.mShooter = shooter.orElseGet(() -> new Shooter(
                new MotorIO() {},
                new MotorIO() {},
                new MotorIO() {},
                new MotorIO() {},
                new MotorIO() {},
                new EncoderIO() {}));

        this.mIndexer = indexer.orElseGet(() -> new Indexer(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));

        this.mIntake = intake.orElseGet(() -> new Intake(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));

        this.mDrive = drive.orElseGet(() -> new Drive(
                new GyroIO() {},
                Stream.generate(() -> new SwerveModuleIO(new MotorIO() {}, new MotorIO() {}, new EncoderIO() {}))
                        .limit(Constants.Drive.kNumModules)
                        .toArray(SwerveModuleIO[]::new),
                mRobotState));
        setupControllerBindings();
        setupAutonomous();
        setupStateTriggers();
    }

    private void setupControllerBindings() {
        var robotState = mRobotState;
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

        // mShooter.setDefaultCommand(ShootCommands.idleShooterCommand(mShooter, robotState));

        // TODO: update should load when intake is completed
        mIndexer.setDefaultCommand(new IndexCommand(mIndexer, () -> true));

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
                        DriveCommands.rotateToSpeaker(mDrive, robotState, Constants.Drive.kFastKinematicLimits, true));
        mDriverController.leftTrigger().whileTrue(swerveLock(mDrive));

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
        autonomousModeChooser.addDefaultOption("Demo", commands.demo());
        autonomousModeChooser.addOption("Four Piece", commands.fourPiece());
        autonomousModeChooser.addOption("Four Piece Planner", new PathPlannerAuto("FourPiece"));
    }

    private void setupStateTriggers() {
        var teleopTrigger = new Trigger(DriverStation::isTeleopEnabled);
        teleopTrigger.onTrue(
                runOnce(() -> mDrive.zeroGyroscope(mRobotState.getHeading())).withName("ZeroGyroscopeToPose"));
    }

    public Optional<Command> getAutonomousCommand() {
        return Optional.ofNullable(autonomousModeChooser.get());
    }

    public RobotState getRobotState() {
        return mRobotState;
    }
}
