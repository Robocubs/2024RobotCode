package com.team1701.robot.subsystems.intake;

import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final MotorIO mIntakeMotor;
    private final MotorInputsAutoLogged mIntakeMotorInputs = new MotorInputsAutoLogged();
    private final IntakeIO mIntakeIO;
    private final IntakeInputsAutoLogged mIntakeInputs = new IntakeInputsAutoLogged();
    @AutoLogOutput(key = "Intake/HasPiece")
    private final boolean mIntakeHasPiece = false;

    public Intake(MotorIO intakeMotor, IntakeIO intakeIO) {
        mIntakeMotor = intakeMotor;
        mIntakeIO = intakeIO;
    }

    @Override
    public void periodic() {
        mIntakeMotor.updateInputs(mIntakeMotorInputs);
        Logger.processInputs("Intake/Motor", mIntakeMotorInputs);
        // TODO: update, process inputs for IntakeIO
    }

    @AutoLogOutput
    public boolean hasNote() {
        return mIntakeInputs.inputSensor || mIntakeInputs.outputSensor;
    }

    @AutoLogOutput
    public boolean hasNoteAtInput() {
        return mIntakeInputs.inputSensor;
    }

    public void setForward() {
        mIntakeMotor.setPercentOutput(Constants.Intake.kIntakeSpeed);
    }

    public void setReverse() {
        mIntakeMotor.setPercentOutput(Constants.Intake.kOuttakeSpeed);
    }

    public void stop() {
        mIntakeMotor.setPercentOutput(0);
    }
//TODO: Add Reject Contingency
    public void execute() {
        
    }

}
