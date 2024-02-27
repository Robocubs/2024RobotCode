package com.team1701.lib.drivers.cameras.neural;

import com.team1701.lib.drivers.cameras.config.VisionConfig;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DetectorCameraIO {
    public class DetectorCameraInputs implements LoggableInputs {
        public boolean isConnected;
        public DetectorPipelineResult pipelineResult = new DetectorPipelineResult();

        @Override
        public void toLog(LogTable table) {
            table.put("IsConnected", isConnected);
            table.put("PipelineResult", pipelineResult);
        }

        @Override
        public void fromLog(LogTable table) {
            this.isConnected = table.get("IsConnected", false);
            this.pipelineResult = table.get("PipelineResult", new DetectorPipelineResult());
        }
    }

    public VisionConfig getVisionConfig();

    public default void updateInputs(DetectorCameraInputs inputs) {}
}
