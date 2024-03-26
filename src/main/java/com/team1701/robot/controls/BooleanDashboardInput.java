package com.team1701.robot.controls;

import java.util.stream.Stream;

import com.team1701.robot.Constants;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class BooleanDashboardInput implements LoggedDashboardInput {
    private final String key;
    private final BooleanSubscriber subscriber;
    private final boolean defaultValue;
    private boolean value;

    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key, value);
        }

        public void fromLog(LogTable table) {
            value = table.get(key, defaultValue);
        }
    };

    public BooleanDashboardInput(String key) {
        this(key, false);
    }

    public BooleanDashboardInput(String key, boolean defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;
        this.subscriber = NetworkTableInstance.getDefault()
                .getBooleanTopic("/Dashboard/" + key)
                .subscribe(
                        defaultValue, PubSubOption.periodic(Constants.kLoopPeriodSeconds), PubSubOption.sendAll(true));
        periodic();
        Logger.registerDashboardInput(this);
    }

    public boolean get() {
        return value;
    }

    public void periodic() {
        if (!Logger.hasReplaySource()) {
            // If a button is pressed and released in a single loop, treat it as if it was pressed this loop
            var values = subscriber.readQueue();
            value = values.length > 0 ? Stream.of(values).anyMatch(val -> val.value) : subscriber.get();
        }

        Logger.processInputs(prefix, inputs);
    }
}
