package com.team1701.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.lib.util.LoggedTunableNumber;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;



public class Intake extends SubsystemBase {
    private final MotorIO mIntakeMotor;
    private final MotorInputsAutoLogged mIntakeMotorInputs = new MotorInputsAutoLogged();
    private final IntakeIO mIntakeIO;
    private final IntakeInputsAutoLogged mIntakeInputs = new IntakeInputsAutoLogged();

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
        if(mIntakeInputDetectsPiece == true){
        return true;
        }

        throw new UnsupportedOperationException("Not implemented");
    }

    public void setForward() {
        mIntakeMotor.setPercentOutput(0.3);
    }

    public void setReverse() {
        mIntakeMotor.setPercentOutput(-0.3);
    }

    public void stop() {
        mIntakeMotor.setPercentOutput(0);
    }
}