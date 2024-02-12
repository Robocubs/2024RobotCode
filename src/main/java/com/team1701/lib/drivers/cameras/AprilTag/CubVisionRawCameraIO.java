package com.team1701.lib.drivers.cameras.AprilTag;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public interface CubVisionRawCameraIO {
    public class CubVisionCameraInputs implements LoggableInputs {
        public double[] timestamps = new double[] {};
        public double[][] frames = new double[][] {};
        public double[] demoFrame = new double[] {};
        public long fps = 0;
        public boolean receivedDataFromCubVision = false;

        @Override
        public void toLog(LogTable table) {
            table.put("Timestamps", timestamps);
            table.put("FrameCount", frames.length);
            for (int i = 0; i < frames.length; i++) {
                table.put("Frame/" + Integer.toString(i), frames[i]);
            }
            table.put("DemoFrame", demoFrame);
            table.put("Fps", fps);
        }

        @Override
        public void fromLog(LogTable table) {
            timestamps = table.get("Timestamps", timestamps);
            int frameCount = (int) table.get("FrameCount", 0);
            frames = new double[frameCount][];
            for (int i = 0; i < frameCount; i++) {
                frames[i] = table.get("Frame/" + Integer.toString(i), new double[] {});
            }
            demoFrame = table.get("DemoFrame", demoFrame);
            fps = table.get("Fps", fps);
        }
    }

    public void updateInputs(CubVisionCameraInputs inputs, TimestampedDoubleArray[] queue);

    public default void addToVisionSim(
            VisionSystemSim visionSim, SimCameraProperties cameraProperties, Transform3d robotToCamPose) {}
}
