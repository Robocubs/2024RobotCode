package com.team1701.robot.subsystems.indexer;

import com.team1701.lib.drivers.digitalinputs.DigitalIO;
import com.team1701.lib.drivers.digitalinputs.DigitalInputsAutoLogged;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.robot.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final MotorIO mIndexerMotorIO;
    private final MotorInputsAutoLogged mIndexerMotorInputs = new MotorInputsAutoLogged();

    private final DigitalIO mIndexerEntranceSensor;
    private final DigitalIO mIndexerExitSensor;
    private final DigitalInputsAutoLogged mIndexerEntranceSensorInputs = new DigitalInputsAutoLogged();
    private final DigitalInputsAutoLogged mIndexerExitSensorInputs = new DigitalInputsAutoLogged();

    private TimeLockedBoolean mLockedHasNoteAtExit =
            new TimeLockedBoolean(/*.075*/ 10, Timer.getFPGATimestamp(), false, false);

    @AutoLogOutput(key = "Indexer/Motor/PercentDemand")
    private double mPercentDemand;

    public Indexer(MotorIO motor, DigitalIO indexerEntranceSensor, DigitalIO indexerExitSensor) {
        mIndexerEntranceSensor = indexerEntranceSensor;
        mIndexerExitSensor = indexerExitSensor;
        mIndexerMotorIO = motor;

        mIndexerMotorIO.setBrakeMode(true);
    }

    public static MotorIOSim createSim(DCMotor IndexerMotor) {
        return new MotorIOSim(IndexerMotor, Constants.Indexer.kIndexerReduction, 0.025, Constants.kLoopPeriodSeconds);
    }

    @Override
    public void periodic() {
        mIndexerMotorIO.updateInputs(mIndexerMotorInputs);
        mIndexerEntranceSensor.updateInputs(mIndexerEntranceSensorInputs);
        mIndexerExitSensor.updateInputs(mIndexerExitSensorInputs);

        Logger.processInputs("Indexer/EntranceSensor", mIndexerEntranceSensorInputs);
        Logger.processInputs("Indexer/ExitSensor", mIndexerExitSensorInputs);
        Logger.processInputs("Indexer/Motor", mIndexerMotorInputs);

        mLockedHasNoteAtExit.update(mIndexerExitSensorInputs.blocked, Timer.getFPGATimestamp());
    }

    @AutoLogOutput(key = "Indexer/LockedHasNoteAtExit")
    public boolean hasNoteAtExit() {
        return mLockedHasNoteAtExit.getValue();
    }

    public boolean hasNoteAtEntrance() {
        return mIndexerEntranceSensorInputs.blocked;
    }

    public boolean hasNote() {
        return mIndexerEntranceSensorInputs.blocked || mIndexerExitSensorInputs.blocked;
    }

    public double getVelocityRadiansPerSecond() {
        return mIndexerMotorInputs.velocityRadiansPerSecond;
    }

    public void setForwardLoad() {
        mPercentDemand = Constants.Indexer.kIndexerLoadPercent;
        mIndexerMotorIO.setPercentOutput(Constants.Indexer.kIndexerLoadPercent);
    }

    public void setForwardShoot() {
        mPercentDemand = Constants.Indexer.kIndexerShootPercent;
        mIndexerMotorIO.setPercentOutput(Constants.Indexer.kIndexerShootPercent);
    }

    public void setReverse() {
        mPercentDemand = -Constants.Indexer.kIndexerShootPercent;
        mIndexerMotorIO.setPercentOutput(-Constants.Indexer.kIndexerShootPercent);
    }

    public void stop() {
        mPercentDemand = 0;
        mIndexerMotorIO.setPercentOutput(0);
    }
}
