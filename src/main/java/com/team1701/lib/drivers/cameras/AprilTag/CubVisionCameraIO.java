package com.team1701.lib.drivers.cameras.AprilTag;

import com.team1701.lib.drivers.cameras.AprilTag.CubVisionRawCameraIO.CubVisionCameraInputs;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class CubVisionCameraIO implements CubVisionRawCameraIO {
    @Override
    public void updateInputs(CubVisionCameraInputs inputs, TimestampedDoubleArray[] queue) {
        inputs.timestamps = new double[queue.length];
        inputs.frames = new double[queue.length][];
        for (int i = 0; i < queue.length; i++) {
            inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
            inputs.frames[i] = queue[i].value;
        }
    }

    @Override
    public void addToVisionSim(
            VisionSystemSim visionSim, SimCameraProperties cameraProperties, Transform3d robotToCamPose) {}
}
