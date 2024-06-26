package com.team1701.lib.drivers.cameras.config;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionConfig {
    public final String cameraName;
    public final Transform3d robotToCamera;
    public final int cameraID;
    public final VisionCameraConfig remoteConfig;
    public final double weight;

    public VisionConfig(
            String cameraName,
            Transform3d robotToCamera,
            int cameraID,
            VisionCameraConfig remoteConfig,
            double weight) {
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
        this.cameraID = cameraID;
        this.remoteConfig = remoteConfig;
        this.weight = weight;
    }
}
