package com.team1701.lib.drivers.cameras.config;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionConfig {
    public final String cameraName;
    public final Transform3d robotToCamPose;
    public final int cameraID;
    public final VisionRemoteConfig remoteConfig;

    public VisionConfig(String cameraName, Transform3d robotToCamPose, int cameraID, VisionRemoteConfig remoteConfig) {
        this.cameraName = cameraName;
        this.robotToCamPose = robotToCamPose;
        this.cameraID = cameraID;
        this.remoteConfig = remoteConfig;
    }
}
