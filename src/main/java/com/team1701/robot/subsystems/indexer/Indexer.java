package com.team1701.robot.subsystems.indexer;

import com.team1701.lib.drivers.digitalinputs.DigitalIO;
import com.team1701.lib.drivers.digitalinputs.DigitalInputsAutoLogged;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.robot.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final MotorIO mIndexerMotorIO;
    private final MotorInputsAutoLogged mIndexerMotorInputsAutoLogged = new MotorInputsAutoLogged();
    private final DigitalIO mEntranceSensor;
    private final DigitalInputsAutoLogged mEntranceSensorInputs = new DigitalInputsAutoLogged();
    private final DigitalIO mExitSensor;
    private final DigitalInputsAutoLogged mExitSensorInputs = new DigitalInputsAutoLogged();

    @AutoLogOutput(key = "Indexer/Motor/DemandRadiansPerSecond")
    private double mDemandRadiansPerSecond;

    public Indexer(MotorIO motor, DigitalIO entranceSensor, DigitalIO exitSensor) {
        mEntranceSensor = entranceSensor;
        mExitSensor = exitSensor;
        mIndexerMotorIO = motor;
    }

    public static MotorIOSim createSim(DCMotor IndexerMotor) {
        return new MotorIOSim(IndexerMotor, Constants.Indexer.kIndexerReduction, 0.025, Constants.kLoopPeriodSeconds);
    }

    @Override
    public void periodic() {
        mIndexerMotorIO.updateInputs(mIndexerMotorInputsAutoLogged);
        mEntranceSensor.updateInputs(mEntranceSensorInputs);
        mExitSensor.updateInputs(mExitSensorInputs);
        Logger.processInputs("Indexer/EntranceSensor", mEntranceSensorInputs);
        Logger.processInputs("Indexer/Motor", mIndexerMotorInputsAutoLogged);
        Logger.processInputs("Indexer/ExitSensor", mExitSensorInputs);

        // TODO: update sensor values

    }

    public boolean noteIsLoaded() {
        return mExitSensorInputs.blocked;
    }

    public void setForwardLoad() {
        mIndexerMotorIO.setPercentOutput(Constants.Indexer.kIndexerLoadPercent);
    }

    public void setForwardShoot() {
        mIndexerMotorIO.setPercentOutput(Constants.Indexer.kIndexerFeedPercent);
    }

    public void setReverse() {
        mIndexerMotorIO.setPercentOutput(-Constants.Indexer.kIndexerFeedPercent);
    }

    public void stop() {
        mIndexerMotorIO.setPercentOutput(0);
    }
}
