package com.team1701.robot.commands;

import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.robot.Constants;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.lib.drivers.digitalinputs.DigitalIO;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command{
    private static final MotorIO mIntakeMotor;
    private static final DigitalIO mIntakeEnterSensor;
    private static final DigitalIO mIntakeExitSensor;
    IntakeCommand(MotorIO intakeMotor, DigitalIO intakeEntranceSensor, DigitalIO intakeExitSensor){
        mIntakeEnterSensor = intakeEntranceSensor;
        mIntakeExitSensor = intakeExitSensor;
        mIntakeMotor = intakeMotor;
    } 
    public void execute() {
        if (mIntakeEnterSensor) {
            Intake.setForward();
        } else{
            Intake.stop();
        }
    }
}
