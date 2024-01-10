package com.team1701.lib.drivers.cameras;

import java.util.ArrayList;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResults;
import org.photonvision.targeting.PNPResults;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public interface AprilTagCameraIO {
    public class PhotonCameraInputs implements LoggableInputs {
        public PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
        public boolean isConnected;

        @Override
        public void toLog(LogTable table) {
            table.put("IsConnected", isConnected);

            var targets = pipelineResult.targets;
            var targetCount = targets.size();

            table.put("Timestamp", pipelineResult.getTimestampSeconds());
            table.put("Latency", pipelineResult.getLatencyMillis());
            table.put("TargetCount", targets.size());

            for (int i = 0; i < targetCount; i++) {
                var targetNamespace = "Target/" + i + "/";
                var target = targets.get(i);

                table.put(targetNamespace + "Yaw", target.getYaw());
                table.put(targetNamespace + "Pitch", target.getPitch());
                table.put(targetNamespace + "Skew", target.getSkew());
                table.put(targetNamespace + "Area", target.getArea());
                table.put(targetNamespace + "FiducialID", target.getFiducialId());
                table.put(targetNamespace + "PoseAmbiguity", target.getPoseAmbiguity());
                table.put(targetNamespace + "Pose", target.getBestCameraToTarget());
                table.put(targetNamespace + "AltPose", target.getBestCameraToTarget());

                var minAreaRectCorners = target.getMinAreaRectCorners().stream()
                        .flatMapToDouble(c -> DoubleStream.of(c.x, c.y))
                        .toArray();

                table.put(targetNamespace + "MinAreaRectCorners", minAreaRectCorners);

                var detectedCorners = target.getDetectedCorners().stream()
                        .flatMapToDouble(c -> DoubleStream.of(c.x, c.y))
                        .toArray();

                table.put(targetNamespace + "DetectedCorners", detectedCorners);
            }

            var multiTagResult = pipelineResult.getMultiTagResult();
            table.put("MultiTag/PnpResult/IsPresent", multiTagResult.estimatedPose.isPresent);
            table.put("MultiTag/PnpResult/BestPose", multiTagResult.estimatedPose.best);
            table.put("MultiTag/PnpResult/BestPoseReprojectionError", multiTagResult.estimatedPose.bestReprojErr);
            table.put("MultiTag/PnpResult/AltPose", multiTagResult.estimatedPose.alt);
            table.put("MultiTag/PnpResult/AltPoseReprojectionError", multiTagResult.estimatedPose.altReprojErr);
            table.put("MultiTag/PnpResult/Ambiguity", multiTagResult.estimatedPose.ambiguity);
            table.put(
                    "MultiTag/TargetIdsUsed",
                    multiTagResult.fiducialIDsUsed.stream()
                            .mapToInt(Integer::intValue)
                            .toArray());
        }

        @Override
        public void fromLog(LogTable table) {
            var timestamp = table.get("Timestamp", 0.0);
            var latency = (int) table.get("Latency", 0);
            var targetCount = (int) table.get("TargetCount", 0);
            var targets = new ArrayList<PhotonTrackedTarget>(targetCount);

            for (int i = 0; i < targetCount; i++) {
                var targetNamespace = "Target/" + i + "/";

                var minAreaRectCornerCords = table.get(targetNamespace + "MinAreaRectCorners", new double[] {});
                var minAreaRectCorners = new ArrayList<TargetCorner>(4);
                for (int j = 0; j < minAreaRectCornerCords.length / 2; j++) {
                    minAreaRectCorners.add(
                            j, new TargetCorner(minAreaRectCornerCords[j * 2], minAreaRectCornerCords[j * 2 + 1]));
                }

                var detectedCornerCords = table.get(targetNamespace + "DetectedCorners", new double[] {});
                var detectedCorners = new ArrayList<TargetCorner>(detectedCornerCords.length);
                for (int j = 0; j < detectedCornerCords.length / 2; j++) {
                    detectedCorners.add(new TargetCorner(detectedCornerCords[j * 2], detectedCornerCords[j * 2 + 1]));
                }

                var legacyFiducialID = table.get(targetNamespace + "Fiducial ID", 0);

                var trackedTarget = new PhotonTrackedTarget(
                        table.get(targetNamespace + "Yaw", 0.0),
                        table.get(targetNamespace + "Pitch", 0.0),
                        table.get(targetNamespace + "Area", 0.0),
                        table.get(targetNamespace + "Skew", 0.0),
                        table.get(targetNamespace + "FiducialID", legacyFiducialID),
                        table.get(targetNamespace + "Pose", new Transform3d()),
                        table.get(targetNamespace + "AltPose", new Transform3d()),
                        table.get(targetNamespace + "PoseAmbiguity", 0.0),
                        minAreaRectCorners,
                        detectedCorners);

                targets.add(trackedTarget);
            }

            var multiTagPnpResultPresent = table.get("MultiTag/PnpResult/IsPresent", false);
            var multiTagPnpResult = multiTagPnpResultPresent
                    ? new PNPResults(
                            table.get("MultiTag/PnpResult/BestPose", new Transform3d()),
                            table.get("MultiTag/PnpResult/AltPose", new Transform3d()),
                            table.get("MultiTag/PnpResult/Ambiguity", 0.0),
                            table.get("MultiTag/PnpResult/BestPoseReprojectionError", 0.0),
                            table.get("MultiTag/PnpResult/AltPoseReprojectionError", 0.0))
                    : new PNPResults();
            var multiTagTargetIdsUsed = IntStream.of(table.get("MultiTag/TargetIdsUsed", new int[] {}))
                    .boxed()
                    .toList();
            var multiTargetResult = new MultiTargetPNPResults(multiTagPnpResult, multiTagTargetIdsUsed);

            pipelineResult = new PhotonPipelineResult(latency, targets, multiTargetResult);
            pipelineResult.setTimestampSeconds(timestamp);

            isConnected = table.get("IsConnected", false);
        }
    }

    public default void updateInputs(PhotonCameraInputs inputs) {}

    public default void addToVisionSim(
            VisionSystemSim visionSim, SimCameraProperties cameraProperties, Transform3d robotToCamPose) {}
}
