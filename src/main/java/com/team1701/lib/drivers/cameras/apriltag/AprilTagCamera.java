package com.team1701.lib.drivers.cameras.apriltag;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import com.team1701.lib.alerts.Alert;
import com.team1701.lib.drivers.cameras.apriltag.AprilTagCameraIO.AprilTagInputs;
import com.team1701.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagCamera {
    private final AprilTagCameraIO mCameraIO;
    private final AprilTagInputs mCameraInputs;
    private final String mLoggingPrefix;
    private final Transform3d mRobotToCamera;
    private final Transform3d mCameraToRobot;
    private final Supplier<AprilTagFieldLayout> mFieldLayoutSupplier;
    private final Supplier<Pose3d> mRobotPoseSupplier;
    private final Function<Double, Vector<N3>> mStdDevsFunction;
    private final List<Consumer<EstimatedRobotPose>> mEstimatedPoseConsumers = new ArrayList<>();
    private final List<Predicate<AprilTagTarget>> mTargetFilters = new ArrayList<>();
    private final List<Predicate<Pose3d>> mPoseFilters = new ArrayList<>();
    private final Alert mDisconnectedAlert;

    public static record EstimatedRobotPose(Pose3d estimatedPose, double timestamp, Vector<N3> stdDevs) {}

    public AprilTagCamera(
            AprilTagCameraIO cameraIO,
            Supplier<AprilTagFieldLayout> fieldLayoutSupplier,
            Supplier<Pose3d> robotPoseSupplier,
            Function<Double, Vector<N3>> stdDevsFunction) {
        var config = cameraIO.getVisionConfig();
        mCameraIO = cameraIO;
        mCameraInputs = new AprilTagInputs();
        mLoggingPrefix = "Camera/" + config.cameraName + "/";
        mRobotToCamera = config.robotToCamera;
        mCameraToRobot = config.robotToCamera.inverse();
        mFieldLayoutSupplier = fieldLayoutSupplier;
        mRobotPoseSupplier = robotPoseSupplier;
        mStdDevsFunction = stdDevsFunction;
        mDisconnectedAlert = Alert.error("Camera " + config.cameraName + " disconnected");
    }

    public void periodic() {
        mCameraIO.updateInputs(mCameraInputs);
        Logger.processInputs(mLoggingPrefix, mCameraInputs);

        for (var pipelineResult : mCameraInputs.pipelineResults) {
            var filteredPipelineResult = filterTargets(pipelineResult);
            var robotPose = mRobotPoseSupplier.get();

            mDisconnectedAlert.setEnabled(!mCameraInputs.isConnected);
            Logger.recordOutput(mLoggingPrefix + "TargetPoses", getFieldRelativeTargetPoses(pipelineResult, robotPose));
            Logger.recordOutput(mLoggingPrefix + "RealTargetPoses", getRealTargetPoses(pipelineResult));
            Logger.recordOutput(
                    mLoggingPrefix + "FilteredTargetPoses",
                    getFieldRelativeTargetPoses(filteredPipelineResult, robotPose));

            var estimatedRobotPose = getEstimatedPose(filteredPipelineResult);
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
    }

    private AprilTagPipelineResult filterTargets(AprilTagPipelineResult pipelineResult) {
        var filteredTargets = Stream.of(pipelineResult.targets)
                .filter(target -> mTargetFilters.stream().allMatch(filter -> filter.test(target)))
                .toArray(AprilTagTarget[]::new);
        return new AprilTagPipelineResult(
                pipelineResult.latencyMilliseconds,
                pipelineResult.timestamp,
                filteredTargets,
                pipelineResult.multiTargetResult);
    }

    private Pose3d[] getFieldRelativeTargetPoses(AprilTagPipelineResult pipelineResult, Pose3d robotPose) {
        return Stream.of(pipelineResult.targets)
                .map(target -> robotPose.plus(mRobotToCamera).plus(target.bestCameraToTarget))
                .toArray(Pose3d[]::new);
    }

    private Pose3d[] getRealTargetPoses(AprilTagPipelineResult pipelineResult) {
        var field = mFieldLayoutSupplier.get();
        return Stream.of(pipelineResult.targets)
                .map(target -> field.getTagPose(target.id))
                .filter(Optional::isPresent)
                .map(Optional::get)
                .toArray(Pose3d[]::new);
    }

    private Optional<EstimatedRobotPose> getEstimatedPose(AprilTagPipelineResult pipelineResult) {
        if (pipelineResult.multiTargetResult.isPresent()
                && pipelineResult.multiTargetResult.get().reprojectionError < 0.6) {
            var multiTargetResult = pipelineResult.multiTargetResult.get();
            var fieldLayout = mFieldLayoutSupplier.get();
            var stdDevs = getStdDevs(multiTargetResult.targetIds, multiTargetResult.cameraPose);

            Logger.recordOutput(mLoggingPrefix + "StdDev", stdDevs.getData());

            return Optional.of(new EstimatedRobotPose(
                    multiTargetResult
                            .cameraPose
                            .relativeTo(fieldLayout.getOrigin())
                            .plus(mCameraToRobot),
                    pipelineResult.timestamp,
                    stdDevs));
        }

        if (pipelineResult.targets.length == 0) {
            return Optional.empty();
        }

        var lowestAmbiguityTarget = pipelineResult.targets[0];
        for (var i = 1; i < pipelineResult.targets.length; i++) {
            if (pipelineResult.targets[i].ambiguity < lowestAmbiguityTarget.ambiguity) {
                lowestAmbiguityTarget = pipelineResult.targets[i];
            }
        }

        var targetPose = mFieldLayoutSupplier.get().getTagPose(lowestAmbiguityTarget.id);
        if (targetPose.isEmpty()) {
            return Optional.empty();
        }

        var stdDevs = mStdDevsFunction.apply(
                lowestAmbiguityTarget.bestCameraToTarget.getTranslation().getNorm());

        // Don't trust single-target rotations at all
        // Trust XY in general much less
        stdDevs.times(Constants.Vision.kSingleTargetStdDevScalar);
        stdDevs.set(2, 0, Double.POSITIVE_INFINITY);

        Logger.recordOutput(mLoggingPrefix + "StdDev", stdDevs.getData());

        return Optional.of(new EstimatedRobotPose(
                targetPose
                        .get()
                        .transformBy(lowestAmbiguityTarget.bestCameraToTarget.inverse())
                        .transformBy(mCameraToRobot),
                pipelineResult.timestamp,
                stdDevs));
    }

    private Vector<N3> getStdDevs(int[] targetIds, Pose3d cameraPose) {
        var field = mFieldLayoutSupplier.get();
        var distance = IntStream.of(targetIds)
                .boxed()
                .map(field::getTagPose)
                .filter(Optional::isPresent)
                .mapToDouble(targetPose -> targetPose.get().getTranslation().getDistance(cameraPose.getTranslation()))
                .average()
                .orElse(10);
        return mStdDevsFunction.apply(distance);
    }

    public void addEstimatedPoseConsumer(Consumer<EstimatedRobotPose> consumer) {
        mEstimatedPoseConsumers.add(consumer);
    }

    public void addTargetFilter(Predicate<AprilTagTarget> filter) {
        mTargetFilters.add(filter);
    }

    public void addPoseFilter(Predicate<Pose3d> filter) {
        mPoseFilters.add(filter);
    }

    public void addToVisionSim(VisionSystemSim visionSim, SimCameraProperties cameraProperties) {
        mCameraIO.addToVisionSim(visionSim, cameraProperties, mRobotToCamera);
    }
}
