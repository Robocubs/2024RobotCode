package com.team1701.lib.drivers.cameras.neural;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

import com.team1701.lib.alerts.Alert;
import com.team1701.lib.drivers.cameras.config.VisionConfig;
import com.team1701.lib.drivers.cameras.neural.DetectorCameraIO.DetectorCameraInputs;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.Logger;

public class DetectorCamera {
    private final DetectorCameraIO mDetectorCameraIO;
    private final DetectorCameraInputs mDetectorCameraInputs;
    private final String mLoggingPrefix;
    private final Function<Double, Pose3d> mRobotPoseSupplier;
    private final List<Predicate<Pose3d>> mPoseFilters = new ArrayList<>();
    private final VisionConfig mConfig;
    private final Alert mDisconnectedAlert;
    private final List<Consumer<List<DetectedObjectState>>> mNoteStateConsumers = new ArrayList<>();

    public static record DetectedObjectState(double timestamp, Pose3d pose) {
        public boolean isSame(DetectedObjectState other, double distanceThreshold) {
            return pose.getTranslation().getDistance(other.pose.getTranslation()) < distanceThreshold;
        }
    }

    public DetectorCamera(DetectorCameraIO cameraIO, Function<Double, Pose3d> robotPoseSupplier) {
        mDetectorCameraIO = cameraIO;
        mConfig = mDetectorCameraIO.getVisionConfig();
        mRobotPoseSupplier = robotPoseSupplier;
        mDetectorCameraInputs = new DetectorCameraInputs();
        mLoggingPrefix = "DetectorCamera/" + mConfig.cameraName + "/";
        mDisconnectedAlert = Alert.error("DetectorCamera " + mConfig.cameraName + " disconnected");
    }

    private Pose3d getDetectedObjectPosition(Pose2d robotPose, Pose2d cameraPose, DetectedObject detectedObject) {
        var robotToCamPose = mConfig.robotToCamera;

        // (TargetHeight - CameraHeight) / tan(CameraPitch + TargetPitch)
        var x = (FieldConstants.kNoteHeight - robotToCamPose.getZ())
                / Math.tan(robotToCamPose.getRotation().getY() + detectedObject.pitch.getRadians());
        var y = x * Math.tan(detectedObject.yaw.getRadians());

        var cameraToObject = new Transform2d(x, -y, GeometryUtil.kRotationIdentity);
        var objectPoseNoRotation = cameraPose.transformBy(cameraToObject);
        var objectLineupRotation = objectPoseNoRotation
                .getTranslation()
                .minus(robotPose.getTranslation())
                .getAngle();

        return new Pose3d(
                objectPoseNoRotation.getX(),
                objectPoseNoRotation.getY(),
                FieldConstants.kHalfNoteHeight,
                GeometryUtil.toRotation3d(objectLineupRotation));
    }

    public void periodic() {
        mDetectorCameraIO.updateInputs(mDetectorCameraInputs);
        Logger.processInputs(mLoggingPrefix, mDetectorCameraInputs);

        mDisconnectedAlert.setEnabled(!mDetectorCameraInputs.isConnected);
        if (!mDetectorCameraInputs.isConnected || mDetectorCameraInputs.pipelineResult.detectedObjects.length == 0) {
            return;
        }

        var pipelineResult = mDetectorCameraInputs.pipelineResult;
        var robotPose3d = mRobotPoseSupplier.apply(pipelineResult.timestamp);
        var robotPose2d = robotPose3d.toPose2d();
        var cameraPose = robotPose3d.transformBy(mConfig.robotToCamera).toPose2d();

        List<Pose3d> detectedObjects = new ArrayList<>(pipelineResult.detectedObjects.length);
        List<DetectedObjectState> detectedObjectStates = new ArrayList<>(pipelineResult.detectedObjects.length);
        for (var detectedObject : pipelineResult.detectedObjects) {
            detectedObjects.add(getDetectedObjectPosition(robotPose2d, cameraPose, detectedObject));
        }

        Logger.recordOutput(mLoggingPrefix + "ObjectPosesNoFilter", detectedObjects.toArray(Pose3d[]::new));

        detectedObjects.forEach(
                pose -> detectedObjectStates.add(new DetectedObjectState(pipelineResult.timestamp, pose)));
        mNoteStateConsumers.forEach(consumer -> consumer.accept(detectedObjectStates));
    }

    public void addDetectedObjectConsumer(Consumer<List<DetectedObjectState>> consumer) {
        mNoteStateConsumers.add(consumer);
    }

    public void addPoseFilter(Predicate<Pose3d> filter) {
        mPoseFilters.add(filter);
    }
}
