// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1701.robot;

import java.nio.file.Paths;
import java.util.Optional;

import com.team1701.lib.commands.CommandLogger;
import com.team1701.robot.states.RobotState.ScoringMode;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Optional<Command> mAutonomousCommand = Optional.empty();
    private RobotContainer mRobotContainer;
    private Command mZeroCommand;
    private boolean mHasZeroed = false;

    @Override
    public void robotInit() {
        // Record metadata
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Configuration.getMode()) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIMULATION:
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                var logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        setUseTiming(Configuration.getMode() != Configuration.Mode.REPLAY);
        LogTable.disableProtobufWarning();
        Logger.start();

        // Build robot container
        mRobotContainer = new RobotContainer();

        mZeroCommand = mRobotContainer.getZeroCommand();

        // Enable command logging
        SmartDashboard.putData(CommandScheduler.getInstance());

        // Launch web server
        Javalin.create(config -> {
                    config.staticFiles.add(
                            Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "web")
                                    .toString(),
                            Location.EXTERNAL);
                })
                .get("/", ctx -> ctx.redirect("/dashboard"))
                .start(5800);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        CommandLogger.getInstance().periodic();
        mRobotContainer.getRobotState().periodic();
    }

    @Override
    public void disabledInit() {
        mRobotContainer.getDriverController().getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        if (!mHasZeroed) {
            mZeroCommand.schedule();
            mHasZeroed = true;
        }
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        mRobotContainer.getRobotState().setScoringMode(ScoringMode.SPEAKER);
        mAutonomousCommand = mRobotContainer.getAutonomousCommand();
        mAutonomousCommand.ifPresent((command) -> command.schedule());
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        mAutonomousCommand.ifPresent(Command::cancel);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
