package com.team1701.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class LoggedCommand extends WrapperCommand {
    private final Command mCommand;

    LoggedCommand(Command command) {
        super(command);
        mCommand = command;
    }

    @Override
    public void initialize() {
        super.initialize();
        CommandLogger.getInstance().logInitialized(mCommand);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            CommandLogger.getInstance().logInterrupted(mCommand);
        } else {
            CommandLogger.getInstance().logFinished(mCommand);
        }
    }
}
