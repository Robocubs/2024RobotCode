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
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
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
    private final String mCameraName;
    private final String mLoggingPrefix;
    private final Transform3d mRobotToCamera;
    private final Transform3d mCameraToRobot;
    private final Supplier<AprilTagFieldLayout> mFieldLayoutSupplier;
    private final Function<Double, Pose3d> mRobotPoseFunction;
    private final Function<StdDevArguments, Vector<N3>> mStdDevsFunction;
    private final double mStdDevsScalar;
    private final SingleTargetMode mSingleTargetMode;
    private final List<Consumer<EstimatedRobotPose>> mEstimatedPoseConsumers = new ArrayList<>();
    private final List<Predicate<AprilTagTarget>> mTargetFilters = new ArrayList<>();
    private final List<Predicate<Pose3d>> mPoseFilters = new ArrayList<>();
    private final Alert mDisconnectedAlert;

    public static record EstimatedRobotPose(Pose3d estimatedPose, double timestamp, Vector<N3> stdDevs) {}

    public static record StdDevArguments(int numTargets, double avgDistance, double scalar) {}

    public AprilTagCamera(
            AprilTagCameraIO cameraIO,
            Supplier<AprilTagFieldLayout> fieldLayoutSupplier,
            Function<Double, Pose3d> robotPoseFunction,
            Function<StdDevArguments, Vector<N3>> stdDevsFunction,
            SingleTargetMode singleTargetMode) {
        var config = cameraIO.getVisionConfig();
        mCameraIO = cameraIO;
        mCameraInputs = new AprilTagInputs();
        mCameraName = config.cameraName;
        mLoggingPrefix = "Camera/" + config.cameraName + "/";
        mRobotToCamera = config.robotToCamera;
        mCameraToRobot = config.robotToCamera.inverse();
        mFieldLayoutSupplier = fieldLayoutSupplier;
        mRobotPoseFunction = robotPoseFunction;
        mStdDevsFunction = stdDevsFunction;
        mStdDevsScalar = 1 / config.weight;
        mSingleTargetMode = singleTargetMode;
        mDisconnectedAlert = Alert.error("Camera " + config.cameraName + " disconnected");
    }

    public void periodic() {
        mCameraIO.updateInputs(mCameraInputs);
        LoggingUtil.logPerformance(
                "Process" + mCameraName + "Inputs", () -> Logger.processInputs(mLoggingPrefix, mCameraInputs));

        for (var pipelineResult : mCameraInputs.pipelineResults) {
            var filteredPipelineResult = filterTargets(pipelineResult);
            var robotPose = mRobotPoseFunction.apply(pipelineResult.timestamp);

            mDisconnectedAlert.setEnabled(!mCameraInputs.isConnected);
            Logger.recordOutput(mLoggingPrefix + "TargetPoses", getFieldRelativeTargetPoses(pipelineResult, robotPose));
            Logger.recordOutput(mLoggingPrefix + "RealTargetPoses", getRealTargetPoses(pipelineResult));
            Logger.recordOutput(
                    mLoggingPrefix + "FilteredTargetPoses",
                    getFieldRelativeTargetPoses(filteredPipelineResult, robotPose));

            var estimatedRobotPose = getMultiTargetPose(filteredPipelineResult)
                    .filter(this::filterPose)
                    .or(() -> getSingleTargetPose(filteredPipelineResult, robotPose)
                            .filter(this::filterPose));

            if (estimatedRobotPose.isPresent()) {
                mEstimatedPoseConsumers.forEach(consumer -> consumer.accept(estimatedRobotPose.get()));
                Logger.recordOutput(mLoggingPrefix + "RobotPose", estimatedRobotPose.get().estimatedPose);
            } else {
                Logger.recordOutput(mLoggingPrefix + "RobotPose", GeometryUtil.kPose3dIdentity);
            }
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

    private boolean filterPose(EstimatedRobotPose pose) {
        return mPoseFilters.stream().allMatch(filter -> filter.test(pose.estimatedPose));
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

    private Optional<EstimatedRobotPose> getMultiTargetPose(AprilTagPipelineResult pipelineResult) {
        return pipelineResult.multiTargetResult.map(multiTargetResult -> {
            var fieldLayout = mFieldLayoutSupplier.get();
            var stdDevs = getStdDevs(multiTargetResult.targetIds, multiTargetResult.cameraPose);

            Logger.recordOutput(mLoggingPrefix + "StdDev", stdDevs.getData());

            return new EstimatedRobotPose(
                    multiTargetResult
                            .cameraPose
                            .relativeTo(fieldLayout.getOrigin())
                            .plus(mCameraToRobot),
                    pipelineResult.timestamp,
                    stdDevs);
        });
    }

    private Optional<EstimatedRobotPose> getSingleTargetPose(AprilTagPipelineResult pipelineResult, Pose3d robotPose) {
        if (pipelineResult.targets.length == 0) {
            return Optional.empty();
        }

        var fieldLayout = mFieldLayoutSupplier.get();
        Optional<Pose3d> targetPose = Optional.empty();
        var cameraToTarget = GeometryUtil.kTransform3dIdentity;

        switch (mSingleTargetMode) {
            case LOWEST_AMBIGUITY -> {
                var lowestAmbiguityTarget = pipelineResult.targets[0];
                for (var i = 1; i < pipelineResult.targets.length; i++) {
                    if (pipelineResult.targets[i].ambiguity < lowestAmbiguityTarget.ambiguity) {
                        lowestAmbiguityTarget = pipelineResult.targets[i];
                    }
                }

                targetPose = fieldLayout.getTagPose(lowestAmbiguityTarget.id);
                cameraToTarget = lowestAmbiguityTarget.bestCameraToTarget;
            }
            case CLOSEST_TO_HEADING -> {
                var cameraHeadingRadians = robotPose.getRotation().getZ()
                        + mRobotToCamera.getRotation().getZ();
                var lowestError = Double.POSITIVE_INFINITY;
                for (var target : pipelineResult.targets) {
                    var currentTargetPose = fieldLayout.getTagPose(target.id);
                    if (currentTargetPose.isEmpty()) {
                        continue;
                    }

                    var targetHeadingRadians =
                            currentTargetPose.get().getRotation().getZ();

                    var bestError = Math.abs(MathUtil.angleModulus(cameraHeadingRadians
                            + target.bestCameraToTarget.getRotation().getZ()
                            - targetHeadingRadians));
                    if (bestError < lowestError) {
                        lowestError = bestError;
                        cameraToTarget = target.bestCameraToTarget;
                        targetPose = currentTargetPose;
                    }

                    var altError = Math.abs(MathUtil.angleModulus(cameraHeadingRadians
                            + target.altCameraToTarget.getRotation().getZ()
                            - targetHeadingRadians));
                    if (altError < lowestError) {
                        lowestError = altError;
                        cameraToTarget = target.altCameraToTarget;
                        targetPose = currentTargetPose;
                    }
                }
            }
        }

        if (targetPose.isEmpty()) {
            return Optional.empty();
        }

        var stdDevs = mStdDevsFunction.apply(
                new StdDevArguments(1, cameraToTarget.getTranslation().getNorm(), mStdDevsScalar));

        Logger.recordOutput(mLoggingPrefix + "StdDev", stdDevs.getData());

        return Optional.of(new EstimatedRobotPose(
                targetPose.get().transformBy(cameraToTarget.inverse()).transformBy(mCameraToRobot),
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
        return mStdDevsFunction.apply(new StdDevArguments(targetIds.length, distance, mStdDevsScalar));
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

    public static enum SingleTargetMode {
        LOWEST_AMBIGUITY,
        CLOSEST_TO_HEADING,
    }
}
