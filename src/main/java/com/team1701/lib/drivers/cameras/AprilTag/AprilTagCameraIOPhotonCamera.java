package com.team1701.lib.drivers.cameras.AprilTag;

import com.team1701.lib.drivers.cameras.config.VisionConfig;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagCameraIOPhotonCamera implements AprilTagCameraIO {
    private final PhotonCamera mCamera;
    private final VisionConfig mConfig;

    public AprilTagCameraIOPhotonCamera(VisionConfig config) {
        mConfig = config;
        mCamera = new PhotonCamera(config.cameraName);
    }

    @Override
    public void updateInputs(AprilTagInputs inputs) {
        inputs.isConnected = mCamera.isConnected();
        inputs.pipelineResult = mCamera.getLatestResult();
    }

    @Override
    public VisionConfig getVisionConfig() {
        return mConfig;
    }

    @Override
    public void addToVisionSim(
            VisionSystemSim visionSim, SimCameraProperties cameraProperties, Transform3d robotToCamPose) {
        var cameraSim = new PhotonCameraSim(mCamera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamPose);
    }
}
