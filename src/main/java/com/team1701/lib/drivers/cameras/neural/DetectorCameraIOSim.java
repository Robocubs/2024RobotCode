package com.team1701.lib.drivers.cameras.neural;

import java.util.function.Supplier;

import com.team1701.lib.drivers.cameras.config.VisionConfig;
import edu.wpi.first.wpilibj.Timer;

public class DetectorCameraIOSim implements DetectorCameraIO {
    private final VisionConfig mVisionConfig;
    private final Supplier<DetectedObject[]> mDetectedObjectsSupplier;

    public DetectorCameraIOSim(VisionConfig visionConfig, Supplier<DetectedObject[]> detectedObjectsSupplier) {
        mVisionConfig = visionConfig;
        mDetectedObjectsSupplier = detectedObjectsSupplier;
    }

    @Override
    public VisionConfig getVisionConfig() {
        return mVisionConfig;
    }

    @Override
    public void updateInputs(DetectorCameraInputs inputs) {
        inputs.isConnected = true;
        inputs.pipelineResult =
                new DetectorPipelineResult(0.0, Timer.getFPGATimestamp(), mDetectedObjectsSupplier.get());
    }
}
