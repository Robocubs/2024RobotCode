package com.team1701.lib.drivers.cameras;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.team1701.lib.alerts.Alert;
import com.team1701.lib.drivers.cameras.CubVisionRawCameraIO.CubVisionCameraInputs;
import com.team1701.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CubVisionCamera {
    private static final double kAmbiguityThreshold = 0.15;

    private final CubVisionCameraIO mCameraIO;
    private final CubVisionCameraInputs mCubVisionCamInputs;

    private final String mLoggingPrefix;
    private final Transform3d mCamToRobotPose;
    private final Supplier<AprilTagFieldLayout> mFieldLayoutSupplier;

    private final Alert mDisconnectedAlert;
    private final DoubleArraySubscriber observationSubscriber;
    private final DoubleArraySubscriber demoObservationSubscriber;
    private final IntegerSubscriber fpsSubscriber;

    private final ArrayList<Consumer<EstimatedRobotPose>> mEstimatedPoseConsumers = new ArrayList<>();

    private final Timer mDisconnectedTimer = new Timer();
    private static final double disconnectedTimeout = 0.5;

    public CubVisionCamera(
            String cameraName,
            int cameraID,
            CubVisionRawCameraIO cameraIO,
            Transform3d robotToCamPose,
            Supplier<AprilTagFieldLayout> fieldLayoutSupplier) {
        mCameraIO = (CubVisionCameraIO) cameraIO;
        mLoggingPrefix = "Camera/" + cameraName + "/";

        mCubVisionCamInputs = new CubVisionCameraInputs();

        mCamToRobotPose = robotToCamPose.inverse();
        mFieldLayoutSupplier = fieldLayoutSupplier;

        var northstarTable = NetworkTableInstance.getDefault().getTable("CubVision/" + cameraName);
        var configTable = northstarTable.getSubTable("config");
        configTable.getIntegerTopic("camera_id").publish().set(cameraID);
        configTable.getIntegerTopic("camera_resolution_width").publish().set(Constants.Vision.cameraResolutionWidth);
        configTable.getIntegerTopic("camera_resolution_height").publish().set(Constants.Vision.cameraResolutionHeight);
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(Constants.Vision.cameraAutoExposure);
        configTable.getIntegerTopic("camera_exposure").publish().set(Constants.Vision.cameraExposure);
        configTable.getIntegerTopic("camera_gain").publish().set(Constants.Vision.cameraGain);
        configTable.getDoubleTopic("fiducial_size_m").publish().set(Constants.Vision.kAprilTagWidth);

        try {
            configTable
                    .getStringTopic("tag_layout")
                    .publish()
                    .set(new ObjectMapper().writeValueAsString(fieldLayoutSupplier.get()));
        } catch (JsonProcessingException e) {
            throw new RuntimeException("Failed to serialize AprilTag layout JSON for Northstar");
        }

        var outputTable = northstarTable.getSubTable("output");
        observationSubscriber = outputTable
                .getDoubleArrayTopic("observations")
                .subscribe(new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        // This is sort-of an artifact of testing CubVision, not sure if I should keep it
        demoObservationSubscriber = outputTable
                .getDoubleArrayTopic("demo_observations")
                .subscribe(new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);

        mDisconnectedAlert = Alert.error("No data from \"" + cameraName + "\"");
        mDisconnectedTimer.start();
    }

    public void periodic() {
        Logger.processInputs(mLoggingPrefix, mCubVisionCamInputs);
        var queue = observationSubscriber.readQueue();

        mCameraIO.updateInputs(mCubVisionCamInputs, queue);
        mCubVisionCamInputs.fps = fpsSubscriber.get();

        // Update disconnected alert
        if (queue.length > 0) {
            mDisconnectedTimer.reset();
        }

        var hasElapsedDisconnectedTimeout = mDisconnectedTimer.hasElapsed(disconnectedTimeout);
        mDisconnectedAlert.setEnabled(hasElapsedDisconnectedTimeout);
        mCubVisionCamInputs.receivedDataFromCubVision = !hasElapsedDisconnectedTimeout;

        var estimatedPoses = constructCameraPose(mCubVisionCamInputs);
        estimatedPoses.forEach(pose -> mEstimatedPoseConsumers.forEach(consumer -> consumer.accept(pose)));

        if (estimatedPoses.size() > 0) {
            Logger.recordOutput(mLoggingPrefix + "/EstimatedRobotPose3D", estimatedPoses.get(0).estimatedPose);
            Logger.recordOutput(
                    mLoggingPrefix + "/EstimatedRobotPose2D",
                    estimatedPoses.get(0).estimatedPose.toPose2d());
        }
    }

    private List<EstimatedRobotPose> constructCameraPose(CubVisionCameraInputs inputs) {
        var estimatedRobotPoses = new ArrayList<EstimatedRobotPose>();
        var frames = inputs.frames;
        var timestamps = mCubVisionCamInputs.timestamps;
        for (int frameIndex = 0; frameIndex < timestamps.length; frameIndex++) {
            var values = frames[frameIndex];

            // Exit if blank frame
            if (values.length == 0 || values[0] == 0) {
                continue;
            }

            var robotTimestamp = Timer.getFPGATimestamp();
            var timestamp = timestamps[frameIndex] - (values[1] / 1000.0);

            // Switch based on number of poses
            switch ((int) values[0]) {
                case 1:
                    // One pose (multi-tag), use directly
                    var cameraPose = new Pose3d(
                            values[3],
                            values[4],
                            values[5],
                            new Rotation3d(new Quaternion(values[6], values[7], values[8], values[9])));
                    estimatedRobotPoses.add(new EstimatedRobotPose(
                            cameraPose.transformBy(mCamToRobotPose),
                            timestamp,
                            Collections.emptyList(),
                            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
                    break;

                case 2:
                    // Two poses (one tag), disambiguate
                    double error0 = values[2];
                    double error1 = values[10];
                    Pose3d cameraPose0 = new Pose3d(
                            values[3],
                            values[4],
                            values[5],
                            new Rotation3d(new Quaternion(values[6], values[7], values[8], values[9])));
                    Pose3d cameraPose1 = new Pose3d(
                            values[11],
                            values[12],
                            values[13],
                            new Rotation3d(new Quaternion(values[14], values[15], values[16], values[17])));

                    // Select pose using projection errors
                    if (error0 < error1 * kAmbiguityThreshold) {
                        estimatedRobotPoses.add(new EstimatedRobotPose(
                                cameraPose0.transformBy(mCamToRobotPose),
                                timestamp,
                                Collections.emptyList(),
                                PoseStrategy.LOWEST_AMBIGUITY));
                    } else if (error1 < error0 * kAmbiguityThreshold) {
                        estimatedRobotPoses.add(new EstimatedRobotPose(
                                cameraPose1.transformBy(mCamToRobotPose),
                                timestamp,
                                Collections.emptyList(),
                                PoseStrategy.LOWEST_AMBIGUITY));
                    }
                    break;
            }

            // TODO
            // Exit if robot pose is off the field
            // if (robotPose3d.getX() < -fieldBorderMargin
            //         || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
            //         || robotPose3d.getY() < -fieldBorderMargin
            //         || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
            //         || robotPose3d.getZ() < -zMargin
            //         || robotPose3d.getZ() > zMargin) {
            //     continue;
            // }

        }

        estimatedRobotPoses.sort((a, b) -> Double.compare(b.timestampSeconds, a.timestampSeconds));

        return estimatedRobotPoses;
    }

    public void addEstimatedPoseConsumer(Consumer<EstimatedRobotPose> consumer) {
        mEstimatedPoseConsumers.add(consumer);
    }
}
