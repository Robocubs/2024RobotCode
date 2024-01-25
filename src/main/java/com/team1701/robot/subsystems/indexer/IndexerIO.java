package com.team1701.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DigitalInput;

public interface IndexerIO{
    @AutoLog
    public static class IndexerInputs {
        public boolean entranceSensorBlocked; 
        public boolean exitSensorBlocked;

    }
    boolean entranceSensorBlocked = false;
    boolean exitSensorBlocked = false;
    public default void updateInputs(IndexerInputs inputs) {
        
    }

}