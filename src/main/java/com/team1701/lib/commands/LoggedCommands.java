package com.team1701.lib.commands;

import java.util.stream.Stream;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LoggedCommands {
    public static Command logged(Command command) {
        return (command instanceof LoggedCommand) ? command : new LoggedCommand(command);
    }

    public static Command loggedSequence(Command... commands) {
        return Commands.sequence(Stream.of(commands).map(LoggedCommands::logged).toArray(Command[]::new));
    }
}
