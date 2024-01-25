package com.team1701.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team1701.robot.subsystems.indexer.IndexerIO;
import com.team1701.robot.subsystems.indexer.Indexer.IndexerIOSensors;
//import com.team1701.robot.subsystems.elevator.*;
//import com.team1701.robot.subsystems.intake.*;


public class IndexCommand extends Command {
    private boolean shouldLoad;
    private boolean enterSensor = IndexerIOSensors.entranceSensorBlocked;
    private boolean exitSensor = IndexerIOSensors.exitSensorBlocked;
    //private boolean elevatorReady = elevator is down
    //private boolean intakeSensor = intake has piece
    public static void loadPiece(
        boolean shouldLoad,
        boolean enterSensor,
        boolean exitSensor,
        boolean intakeSensor,
        boolean elevatorReady
    ) {
        if(intakeSensor && !exitSensor && shouldLoad) {
            return LoadPiece.Index;
        } else if (exitSensor && shouldLoad && elevatorReady) {
            return LoadPiece.Load;
        } else {
            //do nothing?
        }
    }

}