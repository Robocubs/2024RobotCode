package com.team1701.robot;

import com.team1701.lib.alerts.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Configuration {
    private static final RobotType kRobot = RobotType.SIMULATION_BOT;

    private static boolean mInvalidRobotAlerted = false;

    public static RobotType getRobot() {
        if (Robot.isReal() && (kRobot == RobotType.SIMULATION_BOT || kRobot == RobotType.SIMULATION_VISION)) {
            if (!mInvalidRobotAlerted) {
                Alert.warning("Invalid robot configured. Using competition bot as default.")
                        .enable();
                mInvalidRobotAlerted = true;
            }

            return RobotType.COMPETITION_BOT;
        }

        return kRobot;
    }

    public static Mode getMode() {
        switch (getRobot()) {
            case COMPETITION_BOT:
                return Robot.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIMULATION_BOT:
            case SIMULATION_VISION:
                return Mode.SIMULATION;
            default:
                return Mode.REAL;
        }
    }

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    public static enum RobotType {
        COMPETITION_BOT,
        SIMULATION_BOT,
        SIMULATION_VISION,
    }

    public static enum Mode {
        REAL,
        REPLAY,
        SIMULATION
    }
}
