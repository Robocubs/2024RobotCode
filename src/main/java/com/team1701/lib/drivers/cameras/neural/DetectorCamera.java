package com.team1701.lib.drivers.cameras.neural;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

import com.team1701.lib.alerts.Alert;
import com.team1701.lib.drivers.cameras.config.VisionConfig;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.states.NoteState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class DetectorCamera {
    private final DetectorCameraIO mDetectorCameraIO;
    private final DetectorCameraInputsAutoLogged mDetectorCameraInputs;
    private final String mLoggingPrefix;
    private final Supplier<Pose3d> mRobotPoseSupplier;
    private final ArrayList<Predicate<Pose3d>> mPoseFilters = new ArrayList<>();
    private final VisionConfig mConfig;
    private final Alert mDisconnectedAlert;
    private ArrayList<Consumer<List<NoteState>>> mNoteStateConsumers = new ArrayList<>();

    /*
     * Big TODO:
     *
     * For the future, we may want to create filters and consumers based on
     * custom detected object classes. This could be for cameras that detect multiple
     * objects, and we may include confidence, class, pose, etc.
     * In the interest of time, however, this code only supports filtering
     * poses.
     */

    public DetectorCamera(DetectorCameraIO cameraIO, Supplier<Pose3d> robotPoseSupplier) {
        mDetectorCameraIO = cameraIO;
        mConfig = mDetectorCameraIO.getVisionConfig();
        mRobotPoseSupplier = robotPoseSupplier;
        mDetectorCameraInputs = new DetectorCameraInputsAutoLogged();
        mLoggingPrefix = "DetectorCamera/" + mConfig.cameraName + "/";
        mDisconnectedAlert = Alert.error("DetectorCamera " + mConfig.cameraName + " disconnected");
    }

    private Pose3d getDetectedObjectPosition(Pose2d robotPose, double tx, double ty, double ta) {
        var robotToCamPose = mConfig.robotToCamPose;
        double x = PhotonUtils.calculateDistanceToTargetMeters(
                robotToCamPose.getZ(),
                FieldConstants.kNoteHeight,
                robotToCamPose.getRotation().getY(),
                Math.toRadians(ty));
        double y = x * Math.tan(Math.toRadians(tx));

        var translationToObject = new Translation2d(x, -y);
        var robotToCamTransform = new Transform2d(
                new Translation2d(robotToCamPose.getX(), robotToCamPose.getY()),
                new Rotation2d(robotToCamPose.getRotation().getZ()));
        var objectPoseNoRotation = robotPose
                .transformBy(robotToCamTransform)
                .transformBy(new Transform2d(translationToObject, new Rotation2d()));
        var objectLineupRotation = objectPoseNoRotation
                .getTranslation()
                .minus(robotPose.getTranslation())
                .getAngle();
        var objectPose2d = new Pose2d(objectPoseNoRotation.getTranslation(), objectLineupRotation);

        return new Pose3d(
                objectPose2d.getX(),
                objectPose2d.getY(),
                FieldConstants.kNoteHeight / 2,
                new Rotation3d(0.0, 0.0, objectPose2d.getRotation().getRadians()));
    }

    public void periodic() {
        mDetectorCameraIO.updateInputs(mDetectorCameraInputs);
        Logger.processInputs(mLoggingPrefix, mDetectorCameraInputs);

        mDisconnectedAlert.setEnabled(!mDetectorCameraInputs.isConnected);
        var currentRobotPose = mRobotPoseSupplier.get();
        List<Pose3d> detectedObjects = new ArrayList<Pose3d>();
        List<NoteState> noteStates = new ArrayList<>();

        for (int i = 0; i < mDetectorCameraInputs.numberOfDetectedObjects; i++) {
            detectedObjects.add(getDetectedObjectPosition(
                    currentRobotPose.toPose2d(),
                    mDetectorCameraInputs.txs[i],
                    mDetectorCameraInputs.tys[i],
                    mDetectorCameraInputs.areas[i]));
        }

        Logger.recordOutput(mLoggingPrefix + "ObjectPosesNoFilter", detectedObjects.toArray(Pose3d[]::new));
        for (int i = 0; i < mDetectorCameraInputs.numberOfDetectedObjects; i++) {
            Logger.recordOutput(
                    mLoggingPrefix + "ObjectDistancesNoFilter/" + i,
                    detectedObjects.get(i).getTranslation().getDistance(currentRobotPose.getTranslation()));
        }

        // Filter the objects. Just remove it if it doesn't pass a filter
        detectedObjects.forEach(pose -> {
            if (!mPoseFilters.stream().allMatch(filter -> filter.test(pose))) {
                detectedObjects.remove(pose);
            } else {
                noteStates.add(new NoteState(mDetectorCameraInputs.captureTimestamp, pose));
            }
        });

        // Convert to an array and pass to all consumers
        mNoteStateConsumers.forEach(consumer -> consumer.accept(noteStates));
    }

    public void addNoteStateConsumer(Consumer<List<NoteState>> consumer) {
        mNoteStateConsumers.add(consumer);
    }

    public void addPoseFilter(Predicate<Pose3d> filter) {
        mPoseFilters.add(filter);
    }
}
