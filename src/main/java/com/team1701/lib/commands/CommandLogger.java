package com.team1701.lib.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;

public class CommandLogger {
    private static CommandLogger mInstance = null;

    private final Map<String, Integer> commandCounts = new HashMap<>(100);
    private final List<String> commandEvents = new ArrayList<>(100);

    public static CommandLogger getInstance() {
        if (mInstance == null) {
            synchronized (CommandLogger.class) {
                if (mInstance == null) {
                    mInstance = new CommandLogger();
                }
            }
        }

        return mInstance;
    }

    private CommandLogger() {
        CommandScheduler.getInstance().onCommandInitialize(this::logInitialized);
        CommandScheduler.getInstance().onCommandFinish(this::logFinished);
        CommandScheduler.getInstance().onCommandInterrupt(this::logInterrupted);
    }

    public void logInitialized(Command command) {
        logCommand(command, true, "Initialized");
    }

    public void logFinished(Command command) {
        logCommand(command, false, "Finished");
    }

    public void logInterrupted(Command command) {
        logCommand(command, false, "Interrupted");
    }

    public void periodic() {
        Logger.recordOutput("Command/Events", commandEvents.toArray(String[]::new));
        commandEvents.clear();
    }

    private void logCommand(Command command, boolean active, String event) {
        var name = command.getName();
        var count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
        commandCounts.put(name, count);

        var hash = Integer.toHexString(command.hashCode());
        Logger.recordOutput("Command/" + name + "/Active/" + hash, active);
        Logger.recordOutput("Command/" + name + "/ActiveCount", count);

        commandEvents.add(name + "_" + hash + "_" + event);
    }
}
