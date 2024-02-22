package com.team1701.lib.drivers.cameras.apriltag;

import java.util.Arrays;
import java.util.Optional;

import com.team1701.lib.drivers.cameras.config.VisionConfig;
import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

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
        inputs.pipelineResults = new AprilTagPipelineResult[] {toPipelineResult(mCamera.getLatestResult())};
    }

    private AprilTagPipelineResult toPipelineResult(PhotonPipelineResult result) {
        var targets = new AprilTagTarget[result.getTargets().size()];
        Arrays.setAll(targets, i -> {
            var target = result.getTargets().get(i);
            return new AprilTagTarget(
                    target.getFiducialId(),
                    target.getBestCameraToTarget(),
                    target.getAlternateCameraToTarget(),
                    target.getPoseAmbiguity());
        });

        Optional<AprilTagMultiTargetResult> multiTargetResult = Optional.empty();
        var multiTagResult = result.getMultiTagResult();
        if (multiTagResult.estimatedPose.isPresent) {
            var tagIds = new int[multiTagResult.fiducialIDsUsed.size()];
            Arrays.setAll(tagIds, i -> multiTagResult.fiducialIDsUsed.get(i));
            multiTargetResult = Optional.of(new AprilTagMultiTargetResult(
                    tagIds,
                    GeometryUtil.toPose3d(multiTagResult.estimatedPose.best),
                    multiTagResult.estimatedPose.bestReprojErr));
        }

        return new AprilTagPipelineResult(
                result.getLatencyMillis(), result.getTimestampSeconds(), targets, multiTargetResult);
    }

    @Override
    public VisionConfig getVisionConfig() {
        return mConfig;
    }

    @Override
    public void addToVisionSim(
            VisionSystemSim visionSim, SimCameraProperties cameraProperties, Transform3d robotToCamPose) {
        cameraProperties.setCalibration(
                mConfig.remoteConfig.cameraResolutionWidth,
                mConfig.remoteConfig.cameraResolutionHeight,
                cameraProperties.getDiagFOV());
        var cameraSim = new PhotonCameraSim(mCamera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamPose);
    }
}
