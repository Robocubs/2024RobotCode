package com.team1701.lib.util;

import org.littletonrobotics.junction.Logger;

public class LoggingUtil {
    public static void logPerformance(String key, Runnable run) {
        var start = Logger.getRealTimestamp();
        run.run();
        Logger.recordOutput("Performance/" + key, (Logger.getRealTimestamp() - start) / 1000.0);
    }
}
