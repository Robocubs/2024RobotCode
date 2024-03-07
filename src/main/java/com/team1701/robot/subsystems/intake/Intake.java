package com.team1701.robot.subsystems.intake;

import com.team1701.lib.drivers.digitalinputs.DigitalIO;
import com.team1701.lib.drivers.digitalinputs.DigitalInputsAutoLogged;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final MotorIO mIntakeMotor;
    private final MotorInputsAutoLogged mIntakeMotorInputs = new MotorInputsAutoLogged();

    private final DigitalInputsAutoLogged mIntakeEntranceSensorInputs = new DigitalInputsAutoLogged();
    private final DigitalInputsAutoLogged mIntakeExitSensorInputs = new DigitalInputsAutoLogged();

    private final DigitalIO mIntakeEntranceSensor;
    private final DigitalIO mIntakeExitSensor;

    @AutoLogOutput(key = "Intake/Motor/PercentDemand")
    private double mPercentDemand;

    public Intake(MotorIO intakeMotor, DigitalIO intakeEntranceSensor, DigitalIO intakeExitSensor) {
        mIntakeMotor = intakeMotor;
        mIntakeEntranceSensor = intakeEntranceSensor;
        mIntakeExitSensor = intakeExitSensor;
    }

    @Override
    public void periodic() {
        mIntakeMotor.updateInputs(mIntakeMotorInputs);
        mIntakeEntranceSensor.updateInputs(mIntakeEntranceSensorInputs);
        mIntakeExitSensor.updateInputs(mIntakeExitSensorInputs);

        Logger.processInputs("Intake/Motor", mIntakeMotorInputs);
        Logger.processInputs("Intake/EntranceSensor", mIntakeEntranceSensorInputs);
        Logger.processInputs("Intake/ExitSensor", mIntakeExitSensorInputs);
    }

    public boolean hasNote() {
        return hasNoteAtInput() || hasNoteAtExit();
    }

    public boolean hasNoteAtInput() {
        return mIntakeEntranceSensorInputs.blocked;
    }

    public boolean hasNoteAtExit() {
        return mIntakeExitSensorInputs.blocked;
    }

    public double getVelocityRadiansPerSecond() {
        return mIntakeMotorInputs.velocityRadiansPerSecond;
    }

    public void setForward() {
        mPercentDemand = Constants.Intake.kIntakeSpeed;
        mIntakeMotor.setPercentOutput(Constants.Intake.kIntakeSpeed);
    }

    public void setMediumForward() {
        mPercentDemand = Constants.Intake.kIntakeSpeed / 2.0;
        mIntakeMotor.setPercentOutput(Constants.Intake.kIntakeSpeed / 2.0);
    }

    public void setSlowForward() {
        mPercentDemand = Constants.Intake.kIntakeSpeed / 3.0;
        mIntakeMotor.setPercentOutput(Constants.Intake.kIntakeSpeed / 3.0);
    }

    public void setReverse() {
        mPercentDemand = Constants.Intake.kOuttakeSpeed;
        mIntakeMotor.setPercentOutput(Constants.Intake.kOuttakeSpeed);
    }

    public void stop() {
        mPercentDemand = 0;
        mIntakeMotor.setPercentOutput(0);
    }
}
