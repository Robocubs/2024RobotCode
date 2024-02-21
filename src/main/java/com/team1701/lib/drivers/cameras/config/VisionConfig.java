package com.team1701.lib.drivers.cameras.config;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionConfig {
    public final String cameraName;
    public final Transform3d robotToCamPose;
    public final int cameraID;
    public final VisionCameraConfig remoteConfig;
    public final PoseStrategy poseStrategy;
    public final PoseStrategy fallbackPoseStrategy;

    public VisionConfig(
            String cameraName,
            Transform3d robotToCamPose,
            int cameraID,
            VisionCameraConfig remoteConfig,
            PoseStrategy poseStrategy,
            PoseStrategy fallbackPoseStrategy) {
        this.cameraName = cameraName;
        this.robotToCamPose = robotToCamPose;
        this.cameraID = cameraID;
        this.remoteConfig = remoteConfig;
        this.poseStrategy = poseStrategy;
        this.fallbackPoseStrategy = fallbackPoseStrategy;
    }
}
