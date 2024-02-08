package com.team1701.lib.drivers.cameras;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;
import java.util.function.Supplier;

import com.team1701.lib.drivers.cameras.config.VisionConfig;
import com.team1701.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.TimestampedRaw;
import edu.wpi.first.wpilibj.RobotController;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.utils.PacketUtils;

public class AprilTagCameraIOCubVision implements AprilTagCameraIO {
    private static final PhotonPipelineResult kEmptyResult = new PhotonPipelineResult();
    private static final MultiTargetPNPResult kEmptyPnpResult = new MultiTargetPNPResult();
    private static VisionConfig mConfig;

    private final RawSubscriber mObservationSubscriber;
    private final IntegerSubscriber mFpsSubscriber;

    public AprilTagCameraIOCubVision(VisionConfig config) {
        mConfig = config;
        var CubVisionTable = NetworkTableInstance.getDefault().getTable("CubVision/" + config.cameraName);

        var configTable = CubVisionTable.getSubTable("config");
        configTable.getIntegerTopic("camera_id").publish().set(config.cameraID);
        configTable.getIntegerTopic("camera_resolution_width").publish().set(config.remoteConfig.cameraResolutionWidth);
        configTable
                .getIntegerTopic("camera_resolution_height")
                .publish()
                .set(config.remoteConfig.cameraResolutionHeight);
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(config.remoteConfig.cameraAutoExposure);
        configTable.getIntegerTopic("camera_exposure").publish().set(config.remoteConfig.cameraExposure);
        configTable.getIntegerTopic("camera_gain").publish().set(config.remoteConfig.cameraGain);
        configTable.getBooleanTopic("should_stream").publish().set(config.remoteConfig.shouldStream);
        configTable.getDoubleTopic("fiducial_size_m").publish().set(Constants.Vision.kAprilTagWidth);

        var outputTable = CubVisionTable.getSubTable("output");
        mObservationSubscriber = outputTable
                .getRawTopic("observations")
                .subscribe(
                        "ObservationsPacket",
                        new byte[] {},
                        PubSubOption.keepDuplicates(true),
                        PubSubOption.sendAll(true));

        mFpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);
    }

    @Override
    public void updateInputs(AprilTagInputs inputs) {
        updateInputs(inputs, Optional.empty());
    }

    @Override
    public void updateInputs(AprilTagInputs inputs, Optional<Supplier<AprilTagFieldLayout>> supplier) {
        // Use fps to determine if the camera is connected
        // FPS should always be non-zero and will update at least once every 2 + latency seconds
        var timestampedFps = mFpsSubscriber.getAtomic();
        inputs.isConnected = timestampedFps.value > 0
                && RobotController.getFPGATime() - mFpsSubscriber.getAtomic().timestamp < 2500000; // 2.5 seconds
        inputs.pipelineResult = readPhotonPipelineResult(mObservationSubscriber.getAtomic());
        ArrayList<Pose3d> tagPoses = new ArrayList<Pose3d>();
        if (supplier.isPresent()) {
            inputs.pipelineResult.targets.forEach(target -> {
                // I hate this
                Optional<Pose3d> pose = supplier.get().get().getTagPose(target.getFiducialId());
                if (pose.isPresent()) {
                    tagPoses.add(pose.get());
                }
            });
        }
        inputs.trueTrackedAprilTagPoses = tagPoses.toArray(Pose3d[]::new);
    }

    private PhotonPipelineResult readPhotonPipelineResult(TimestampedRaw timestampedPacket) {
        if (timestampedPacket.value.length == 0) {
            return kEmptyResult;
        }

        var packet = new Packet(timestampedPacket.value);
        var latency = packet.decodeDouble();

        var targets = new ArrayList<PhotonTrackedTarget>();
        var numTargets = packet.decodeByte();
        for (var i = 0; i < numTargets; i++) {
            var targetId = packet.decodeByte();
            var bestCameraToTargetError = packet.decodeDouble();
            var bestCameraToTarget = PacketUtils.unpackTransform3d(packet);
            var altCameraToTargetError = packet.decodeDouble();
            var altCameratoTarget = PacketUtils.unpackTransform3d(packet);
            var ambiguity = altCameraToTargetError > 0 ? bestCameraToTargetError / altCameraToTargetError : 1.0;

            // The packet only includes data needed for pose estimation
            targets.add(new PhotonTrackedTarget(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    (int) targetId,
                    bestCameraToTarget,
                    altCameratoTarget,
                    ambiguity,
                    Collections.emptyList(),
                    Collections.emptyList()));
        }

        var pnpResult = kEmptyPnpResult;
        var numTagsUsedForCameraPose = packet.decodeByte();

        if (numTagsUsedForCameraPose > 1) {
            var tags = new ArrayList<Integer>();
            for (var i = 0; i < numTagsUsedForCameraPose; i++) {
                tags.add((int) packet.decodeByte());
            }
            var error = packet.decodeDouble();
            var pose = PacketUtils.unpackTransform3d(packet);

            pnpResult = new MultiTargetPNPResult(new PNPResult(pose, error), tags);
        }

        var result = new PhotonPipelineResult(latency, targets, pnpResult);
        result.setTimestampSeconds(timestampedPacket.timestamp / 1000000.0 - latency / 1000.0);
        return result;
    }

    @Override
    public VisionConfig getVisionConfig() {
        return mConfig;
    }

    @Override
    public void addToVisionSim(
            VisionSystemSim visionSim, SimCameraProperties cameraProperties, Transform3d robotToCamPose) {}
}
