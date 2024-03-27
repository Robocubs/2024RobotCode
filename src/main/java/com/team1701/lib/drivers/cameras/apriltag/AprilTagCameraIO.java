package com.team1701.lib.drivers.cameras.apriltag;

import com.team1701.lib.drivers.cameras.config.VisionConfig;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public interface AprilTagCameraIO {
    public class AprilTagInputs implements LoggableInputs {
        public boolean isConnected;
        public AprilTagPipelineResult[] pipelineResults = new AprilTagPipelineResult[] {};
        public int fps;
        public int temperature;
        public int latency;

        @Override
        public void toLog(LogTable table) {
            table.put("IsConnected", isConnected);
            table.put("PipelineResults", new AprilTagPipelineResults(pipelineResults));
            table.put("FPS", fps);
            table.put("Temperature", temperature);
            table.put("Latency", latency);
        }

        @Override
        public void fromLog(LogTable table) {
            this.isConnected = table.get("IsConnected", false);
            this.pipelineResults = table.get("PipelineResults", new AprilTagPipelineResults()).results;
            this.fps = table.get("FPS", 0);
            this.temperature = table.get("Temperature", 0);
            this.latency = table.get("Latency", 0);
        }
    }

    public default void updateInputs(AprilTagInputs inputs) {}

    public VisionConfig getVisionConfig();

    public default void addToVisionSim(
            VisionSystemSim visionSim, SimCameraProperties cameraProperties, Transform3d robotToCamPose) {}
}
