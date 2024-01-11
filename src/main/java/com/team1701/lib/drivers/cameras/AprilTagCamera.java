package com.team1701.lib.drivers.cameras;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

import com.team1701.lib.alerts.Alert;
import com.team1701.lib.drivers.cameras.AprilTagCameraIO.PhotonCameraInputs;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagCamera {
    private final AprilTagCameraIO mCameraIO;
    private final PhotonCameraInputs mCameraInputs;
    private final String mLoggingPrefix;
    private final PhotonPoseEstimator mPoseEstimator;
    private final Transform3d mRobotToCamPose;
    private final Supplier<AprilTagFieldLayout> mFieldLayoutSupplier;
    private final Supplier<Pose3d> mRobotPoseSupplier;
    private final ArrayList<Consumer<EstimatedRobotPose>> mEstimatedPoseConsumers = new ArrayList<>();
    private final ArrayList<Predicate<PhotonTrackedTarget>> mTargetFilters = new ArrayList<>();
    private final ArrayList<Predicate<Pose3d>> mPoseFilters = new ArrayList<>();
    private final Alert mDisconnectedAlert;

    public AprilTagCamera(
            String cameraName,
            AprilTagCameraIO cameraIO,
            Transform3d robotToCamPose,
            PoseStrategy poseStrategy,
            PoseStrategy fallbackPoseStrategy,
            Supplier<AprilTagFieldLayout> fieldLayoutSupplier,
            Supplier<Pose3d> robotPoseSupplier) {
        mCameraIO = cameraIO;
        mCameraInputs = new PhotonCameraInputs();
        mLoggingPrefix = "Camera/" + cameraName + "/";
        mPoseEstimator = new PhotonPoseEstimator(fieldLayoutSupplier.get(), poseStrategy, null, robotToCamPose);
        mPoseEstimator.setMultiTagFallbackStrategy(fallbackPoseStrategy);
        mRobotToCamPose = robotToCamPose;
        mFieldLayoutSupplier = fieldLayoutSupplier;
        mRobotPoseSupplier = robotPoseSupplier;
        mDisconnectedAlert = Alert.error("Camera " + cameraName + " disconnected");
    }

    public void periodic() {
        mCameraIO.updateInputs(mCameraInputs);
        Logger.processInputs(mLoggingPrefix, mCameraInputs);

        var pipelineResult = mCameraInputs.pipelineResult;
        var filteredPipelineResult = filterTargets(pipelineResult);
        var robotPose = mRobotPoseSupplier.get();

        mDisconnectedAlert.setEnabled(!mCameraInputs.isConnected);
        Logger.recordOutput(mLoggingPrefix + "TargetPoses", getFieldRelativeTargetPoses(pipelineResult, robotPose));
        Logger.recordOutput(
                mLoggingPrefix + "FilteredTargetPoses", getFieldRelativeTargetPoses(filteredPipelineResult, robotPose));

        if (!filteredPipelineResult.hasTargets()) {
            return;
        }

        mPoseEstimator.setFieldTags(mFieldLayoutSupplier.get());
        var estimatedRobotPose = mPoseEstimator.update(filteredPipelineResult);
        if (estimatedRobotPose.isEmpty()) {
            return;
        }

        Logger.recordOutput(mLoggingPrefix + "RobotPose", estimatedRobotPose.get().estimatedPose);

        var estimatedPose = estimatedRobotPose.get().estimatedPose;
        if (!mPoseFilters.stream().allMatch(filter -> filter.test(estimatedPose))) {
            return;
        }

        mEstimatedPoseConsumers.forEach(consumer -> consumer.accept(estimatedRobotPose.get()));
        Logger.recordOutput(mLoggingPrefix + "FilteredRobotPose", estimatedRobotPose.get().estimatedPose);
    }

    private PhotonPipelineResult filterTargets(PhotonPipelineResult pipelineResult) {
        var filteredTargets = pipelineResult.getTargets().stream()
                .filter(target -> mTargetFilters.stream().allMatch(filter -> filter.test(target)))
                .toList();
        var filteredPipelineResult = new PhotonPipelineResult(pipelineResult.getLatencyMillis(), filteredTargets);
        filteredPipelineResult.setTimestampSeconds(pipelineResult.getTimestampSeconds());
        return filteredPipelineResult;
    }

    private Pose2d[] getFieldRelativeTargetPoses(PhotonPipelineResult pipelineResult, Pose3d robotPose) {
        return pipelineResult.targets.stream()
                .map(target -> robotPose
                        .plus(mRobotToCamPose)
                        .plus(target.getBestCameraToTarget())
                        .toPose2d())
                .toArray(Pose2d[]::new);
    }

    public void addEstimatedPoseConsumer(Consumer<EstimatedRobotPose> consumer) {
        mEstimatedPoseConsumers.add(consumer);
    }

    public void addTargetFilter(Predicate<PhotonTrackedTarget> filter) {
        mTargetFilters.add(filter);
    }

    public void addPoseFilter(Predicate<Pose3d> filter) {
        mPoseFilters.add(filter);
    }

    public void addToVisionSim(VisionSystemSim visionSim, SimCameraProperties cameraProperties) {
        mCameraIO.addToVisionSim(visionSim, cameraProperties, mRobotToCamPose);
    }
}
