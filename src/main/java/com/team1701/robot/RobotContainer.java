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
import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderIOAnalog;
import com.team1701.lib.drivers.gyros.GyroIO;
import com.team1701.lib.drivers.gyros.GyroIOPigeon2;
import com.team1701.lib.drivers.gyros.GyroIOSim;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration.Mode;
import com.team1701.robot.commands.AutonomousCommands;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.drive.DriveMotorFactory;
import com.team1701.robot.subsystems.drive.SwerveModule.SwerveModuleIO;
import com.team1701.robot.subsystems.shooter.Shooter;
import com.team1701.robot.subsystems.shooter.ShooterMotorFactory;
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
    private final RobotState mRobotState = new RobotState();
    public final Drive mDrive;
    public final Shooter mShooter;
    // public final Vision mVision;

    private final CommandXboxController mDriverController = new CommandXboxController(0);
    private final LoggedDashboardChooser<Command> autonomousModeChooser = new LoggedDashboardChooser<>("Auto Mode");

    public RobotContainer() {
        Optional<Drive> drive = Optional.empty();
        // Optional<Vision> vision = Optional.empty();
        Optional<Shooter> shooter = Optional.empty();

        if (Configuration.getMode() != Mode.REPLAY) {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    drive = Optional.of(new Drive(
                            new GyroIOPigeon2(10),
                            new SwerveModuleIO[] {
                                new SwerveModuleIO(
                                        DriveMotorFactory.createDriveMotorIOSparkMax(10),
                                        DriveMotorFactory.createSteerMotorIOSparkMax(11),
                                        new EncoderIOAnalog(0)),
                                new SwerveModuleIO(
                                        DriveMotorFactory.createDriveMotorIOSparkMax(12),
                                        DriveMotorFactory.createSteerMotorIOSparkMax(13),
                                        new EncoderIOAnalog(1)),
                                new SwerveModuleIO(
                                        DriveMotorFactory.createDriveMotorIOSparkMax(16),
                                        DriveMotorFactory.createSteerMotorIOSparkMax(17),
                                        new EncoderIOAnalog(3)),
                                new SwerveModuleIO(
                                        DriveMotorFactory.createDriveMotorIOSparkMax(14),
                                        DriveMotorFactory.createSteerMotorIOSparkMax(15),
                                        new EncoderIOAnalog(2)),
                            },
                            mRobotState));

                    // TODO: update ID
                    shooter = Optional.of(new Shooter(
                            ShooterMotorFactory.createDriveMotorIOSparkFlex(Constants.Shooter.kShooterDeviceId)));
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
                    shooter = Optional.of(new Shooter(Shooter.createSim(DCMotor.getNeoVortex(1))));
                    break;
                default:
                    break;
            }

            /*vision = Optional.of(new Vision(
            new AprilTagCameraIOPhotonCamera(Constants.Vision.kFrontLeftCameraName),
            new AprilTagCameraIOPhotonCamera(Constants.Vision.kFrontRightCameraName),
            new AprilTagCameraIOPhotonCamera(Constants.Vision.kBackLeftCameraName),
            new AprilTagCameraIOPhotonCamera(Constants.Vision.kBackRightCameraName))); */
        }

        this.mDrive = drive.orElseGet(() -> new Drive(
                new GyroIO() {},
                Stream.generate(() -> new SwerveModuleIO(new MotorIO() {}, new MotorIO() {}, new EncoderIO() {}))
                        .limit(Constants.Drive.kNumModules)
                        .toArray(SwerveModuleIO[]::new),
                mRobotState));

        /*  this.mVision = vision.orElseGet(() -> new Vision(
        new AprilTagCameraIO() {},
        new AprilTagCameraIO() {},
        new AprilTagCameraIO() {},
        new AprilTagCameraIO() {})); */

        this.mShooter = shooter.orElseGet(
                () -> new Shooter(ShooterMotorFactory.createDriveMotorIOSparkFlex(Constants.Shooter.kShooterDeviceId)));

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
        mDriverController
                .x()
                .onTrue(runOnce(() -> mDrive.zeroGyroscope(
                                Configuration.isBlueAlliance()
                                        ? GeometryUtil.kRotationIdentity
                                        : GeometryUtil.kRotationPi))
                        .withName("ZeroGyroscopeToHeading"));
        mDriverController.leftTrigger().whileTrue(swerveLock(mDrive));
        TriggeredAlert.info("Driver right bumper pressed", mDriverController.rightBumper());

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

        var commands = new AutonomousCommands(mRobotState, mDrive);
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
}
