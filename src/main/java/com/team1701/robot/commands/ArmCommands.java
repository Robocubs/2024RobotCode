package com.team1701.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ArmCommands {
    public static Command moveToAmp() {
        return Commands.waitSeconds(1).withName("MoveArmToAmp");
    }

    public static Command retractArm() {
        return Commands.waitSeconds(1).withName("RetractArm");
    }
}
