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

    private final DigitalIO mIndexerEntranceSensor;
    private final DigitalIO mIndexerExitSensor;

    private final DigitalInputsAutoLogged mIndexerEntranceSensorInputs = new DigitalInputsAutoLogged();
    private final DigitalInputsAutoLogged mIndexerExitSensorInputs = new DigitalInputsAutoLogged();

    @AutoLogOutput(key = "Indexer/Motor/DemandRadiansPerSecond")
    private double mDemandRadiansPerSecond;

    public Indexer(MotorIO motor, DigitalIO indexerEntranceSensor, DigitalIO indexerExitSensor) {
        mIndexerEntranceSensor = indexerEntranceSensor;
        mIndexerExitSensor = indexerExitSensor;
        mIndexerMotorIO = motor;
    }

    public static MotorIOSim createSim(DCMotor IndexerMotor) {
        return new MotorIOSim(IndexerMotor, Constants.Indexer.kIndexerReduction, 0.025, Constants.kLoopPeriodSeconds);
    }

    @Override
    public void periodic() {
        mIndexerMotorIO.updateInputs(mIndexerMotorInputsAutoLogged);
        mIndexerEntranceSensor.updateInputs(mIndexerEntranceSensorInputs);
        mIndexerExitSensor.updateInputs(mIndexerExitSensorInputs);

        Logger.processInputs("Indexer/EntranceSensor", mIndexerEntranceSensorInputs);
        Logger.processInputs("Indexer/ExitSensor", mIndexerExitSensorInputs);
        Logger.processInputs("Indexer/Motor", mIndexerMotorInputsAutoLogged);
    }

    public boolean noteIsLoaded() {
        return mIndexerExitSensorInputs.blocked;
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
