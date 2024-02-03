package com.team1701.lib.drivers.digitalinputs;

import org.littletonrobotics.junction.AutoLog;

public interface DigitalIO {
    @AutoLog
    public static class DigitalInputs {
        public boolean blocked;
    }

    public default void updateInputs(DigitalInputs inputs) {}

    public default void getBlocked() {}
}
