package com.team1701.robot.subsystems.indexer;

import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.robot.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private MotorIO mIndexerMotorIO;
    private final MotorInputsAutoLogged mIndexerMotorInputsAutoLogged = new MotorInputsAutoLogged();
    private final IndexerInputsAutoLogged mIndexerInputsAutoLogged = new IndexerInputsAutoLogged();
    private final IndexerIO.IndexerInputs mIndexerInputs = new IndexerIO.IndexerInputs();
    private DigitalInput mEntranceSensor;
    private DigitalInput mExitSensor;
    private IndexerIO indexerIO = new IndexerIOSensors();

    @AutoLogOutput(key = "Indexer/Motor/DemandRadiansPerSecond")
    private double mDemandRadiansPerSecond;

    public Indexer(MotorIO motor) {
        mEntranceSensor = new DigitalInput(Constants.Indexer.kIndexerEntranceSensorPort);
        mExitSensor = new DigitalInput(Constants.Indexer.kIndexerExitSensorPort);
        mIndexerMotorIO = motor;
        setPID(
                Constants.Indexer.kIndexerKff.get(),
                Constants.Indexer.kIndexerKp.get(),
                0,
                Constants.Indexer.kIndexerKd.get());
        mIndexerMotorIO.setBrakeMode(false);
    
    }

    public static MotorIOSim createSim(DCMotor IndexerMotor) {
        return new MotorIOSim(IndexerMotor, Constants.Indexer.kIndexerReduction, 0.025, Constants.kLoopPeriodSeconds);
    }

    @Override
    public void periodic() {
        var hash = hashCode();
        mIndexerMotorIO.updateInputs(mIndexerMotorInputsAutoLogged);
        Logger.processInputs("Indexer/Motor", mIndexerMotorInputsAutoLogged);
        if (Constants.Indexer.kIndexerKff.hasChanged(hash)
                || Constants.Indexer.kIndexerKp.hasChanged(hash)
                || Constants.Indexer.kIndexerKd.hasChanged(hash)) {
            setPID(
                    Constants.Indexer.kIndexerKff.get(),
                    Constants.Indexer.kIndexerKp.get(),
                    0,
                    Constants.Indexer.kIndexerKd.get());
        }
       indexerIO.updateInputs(mIndexerInputsAutoLogged);
    }

    public void setPID(double ff, double p, double i, double d) {
        mIndexerMotorIO.setPID(ff, p, i, d);
    }

    public void setSpeed(double radiansPerSecond) {
        mIndexerMotorIO.setVelocityControl(radiansPerSecond);
    }

    public void loadIndexer() {
        if(mIndexerInputs.entranceSensorBlocked == true) {
            mIndexerMotorIO.setPercentOutput(Constants.Indexer.kIndexerLoadPercent);
        } else if(mIndexerInputs.exitSensorBlocked == true) {
            mIndexerMotorIO.setPercentOutput(0);
        }
    }

    public void setForward() {
        mIndexerMotorIO.setPercentOutput(Constants.Indexer.kIndexerFeedPercent);
    }

    public void setReverse() {
        mIndexerMotorIO.setPercentOutput(-Constants.Indexer.kIndexerFeedPercent);
    }

    public void stop () {
        mIndexerMotorIO.setPercentOutput(0);
    }


    public class IndexerIOSensors implements IndexerIO {

        @Override
        public void updateInputs(IndexerInputs inputs) {
            inputs.entranceSensorBlocked = mEntranceSensor.get();
            inputs.exitSensorBlocked = mExitSensor.get();
        }
    }
}