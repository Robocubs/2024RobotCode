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

        @Override
        public void toLog(LogTable table) {
            table.put("IsConnected", isConnected);
            table.put("PipelineResults", new AprilTagPipelineResults(pipelineResults));
        }

        @Override
        public void fromLog(LogTable table) {
            this.isConnected = table.get("IsConnected", false);
            this.pipelineResults = table.get("PipelineResults", new AprilTagPipelineResults()).results;
        }
    }

    public default void updateInputs(AprilTagInputs inputs) {}

    public VisionConfig getVisionConfig();

    public default void addToVisionSim(
            VisionSystemSim visionSim, SimCameraProperties cameraProperties, Transform3d robotToCamPose) {}
}
