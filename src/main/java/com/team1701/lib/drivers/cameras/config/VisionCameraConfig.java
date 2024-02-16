package com.team1701.lib.drivers.cameras.config;

public class VisionCameraConfig {
    public final int cameraResolutionWidth;
    public final int cameraResolutionHeight;
    public final int cameraAutoExposure;
    public final int cameraExposure;
    public final int cameraGain;
    public final boolean shouldStream;

    // autoExposure=1 and a specified cameraExposure apparently tells OpenCV that the manual exposure is to be
    // used
    public static VisionCameraConfig kStandardArduCamConfig = new VisionCameraConfig(1280, 720, 1, 35, 0, false);
    public static VisionCameraConfig kLimelightConfig = new VisionCameraConfig(960, 720, 0, 0, 0, false);
    public static VisionCameraConfig kSniperCamConfig = new VisionCameraConfig(1920, 1200, 0, 120, 0, false);

    VisionCameraConfig(
            int cameraResolutionWidth,
            int cameraResolutionHeight,
            int cameraAutoExposure,
            int cameraExposure,
            int cameraGain,
            boolean shouldStream) {
        this.cameraResolutionWidth = cameraResolutionWidth;
        this.cameraResolutionHeight = cameraResolutionHeight;
        this.cameraAutoExposure = cameraAutoExposure;
        this.cameraExposure = cameraExposure;
        this.cameraGain = cameraGain;
        this.shouldStream = shouldStream;
    }
}
