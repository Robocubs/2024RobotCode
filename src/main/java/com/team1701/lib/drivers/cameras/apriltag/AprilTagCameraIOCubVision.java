package com.team1701.lib.drivers.cameras.apriltag;

import java.util.Arrays;
import java.util.Optional;

import com.team1701.lib.drivers.cameras.config.VisionConfig;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Constants;
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
import org.photonvision.utils.PacketUtils;

public class AprilTagCameraIOCubVision implements AprilTagCameraIO {
    private final VisionConfig mConfig;

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
                        PubSubOption.sendAll(true),
                        PubSubOption.periodic(0.05)); // Default robot loop

        mFpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);
    }

    @Override
    public void updateInputs(AprilTagInputs inputs) {
        // Use fps to determine if the camera is connected
        // FPS should always be non-zero and will update at least once every 2 + latency seconds
        var timestampedFps = mFpsSubscriber.getAtomic();
        inputs.isConnected = timestampedFps.value > 0
                && RobotController.getFPGATime() - mFpsSubscriber.getAtomic().timestamp < 5000000; // 5 seconds

        var observations = mObservationSubscriber.readQueue();
        inputs.pipelineResults = new AprilTagPipelineResult[observations.length];
        Arrays.setAll(inputs.pipelineResults, i -> readPipelineResult(observations[i]));
    }

    private AprilTagPipelineResult readPipelineResult(TimestampedRaw timestampedPacket) {
        if (timestampedPacket.value.length == 0) {
            return AprilTagPipelineResult.kEmpty;
        }

        var packet = new Packet(timestampedPacket.value);
        var latency = packet.decodeDouble();
        var timestamp = timestampedPacket.timestamp / 1000000.0 - latency / 1000.0;

        var targets = new AprilTagTarget[packet.decodeByte()];
        for (var i = 0; i < targets.length; i++) {
            var targetId = packet.decodeByte();
            var bestCameraToTargetError = packet.decodeDouble();
            var bestCameraToTarget = PacketUtils.unpackTransform3d(packet);
            var altCameraToTargetError = packet.decodeDouble();
            var altCameraToTarget = PacketUtils.unpackTransform3d(packet);
            var ambiguity = altCameraToTargetError > 0 ? bestCameraToTargetError / altCameraToTargetError : 1.0;

            targets[i] = new AprilTagTarget(targetId, bestCameraToTarget, altCameraToTarget, ambiguity);
        }

        Optional<AprilTagMultiTargetResult> multiTargetResult = Optional.empty();
        var numTagsUsedForCameraPose = packet.decodeByte();
        if (numTagsUsedForCameraPose > 1) {
            var tags = new int[numTagsUsedForCameraPose];
            for (var i = 0; i < numTagsUsedForCameraPose; i++) {
                tags[i] = (int) packet.decodeByte();
            }

            var reprojectionError = packet.decodeDouble();
            var cameraPose = GeometryUtil.toPose3d(PacketUtils.unpackTransform3d(packet));
            multiTargetResult = Optional.of(new AprilTagMultiTargetResult(tags, cameraPose, reprojectionError));
        }

        return new AprilTagPipelineResult(latency, timestamp, targets, multiTargetResult);
    }

    @Override
    public VisionConfig getVisionConfig() {
        return mConfig;
    }

    @Override
    public void addToVisionSim(
            VisionSystemSim visionSim, SimCameraProperties cameraProperties, Transform3d robotToCamPose) {}
}
