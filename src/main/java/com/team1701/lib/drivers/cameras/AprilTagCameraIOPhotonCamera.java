package com.team1701.lib.drivers.cameras;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagCameraIOPhotonCamera implements AprilTagCameraIO {
    private final PhotonCamera mCamera;

    public AprilTagCameraIOPhotonCamera(String cameraName) {
        mCamera = new PhotonCamera(cameraName);
    }

    @Override
    public void updateInputs(PhotonCameraInputs inputs) {
        inputs.isConnected = mCamera.isConnected();
        inputs.pipelineResult = mCamera.getLatestResult();
    }

    @Override
    public void addToVisionSim(
            VisionSystemSim visionSim, SimCameraProperties cameraProperties, Transform3d robotToCamPose) {
        var cameraSim = new PhotonCameraSim(mCamera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamPose);
    }
}
