package com.team1701.lib.drivers.cameras.ObjectDetector;

import com.team1701.lib.drivers.cameras.config.VisionConfig;
import org.littletonrobotics.junction.AutoLog;

public interface DetectorCameraIO {
    @AutoLog
    public class DetectorCameraInputs {
        public int numberOfDetectedObjects;
        public String detectedClasses[];
        public long detectedClassIDs[];
        public double confidences[];
        public double areas[];
        public double txs[];
        public double tys[];
        public double txps[];
        public double typs[];
        public double latency;
        // Is this supposed to be here?
        public double captureTimestamp;
        public long givenPipeline;
        public boolean seesTarget;
        public boolean isConnected;

        public void constructEmptyInputArrays(int n) {
            detectedClasses = new String[n];
            detectedClassIDs = new long[n];
            confidences = new double[n];
            areas = new double[n];
            txs = new double[n];
            tys = new double[n];
            txps = new double[n];
            typs = new double[n];
        }
    }

    public default VisionConfig getVisionConfig() {
        throw new UnsupportedOperationException("Unimplemented method 'getVisionConfig'");
    }

    public default void updateInputs(DetectorCameraInputs inputs) {}
}
