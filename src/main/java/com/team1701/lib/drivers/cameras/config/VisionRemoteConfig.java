package com.team1701.lib.drivers.cameras.config;

public class VisionRemoteConfig {
    public final int cameraResolutionWidth;
    public final int cameraResolutionHeight;
    public final int cameraAutoExposure;
    public final int cameraExposure;
    public final int cameraGain;
    public final boolean shouldStream;

    // autoExposure=1 and a specified cameraExposure apparently tells OpenCV that the manual exposure is to be
    // used
    public static VisionRemoteConfig kStandardArduCamConfig = new VisionRemoteConfig(1280, 720, 1, 75, 0, false);

    VisionRemoteConfig(
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
