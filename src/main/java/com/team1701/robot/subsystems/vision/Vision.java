package com.team1701.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.team1701.lib.alerts.Alert;
import com.team1701.lib.drivers.cameras.apriltag.AprilTagCamera;
import com.team1701.lib.drivers.cameras.apriltag.AprilTagCamera.EstimatedRobotPose;
import com.team1701.lib.drivers.cameras.apriltag.AprilTagCamera.StdDevArguments;
import com.team1701.lib.drivers.cameras.apriltag.AprilTagCameraIO;
import com.team1701.lib.drivers.cameras.neural.DetectorCamera;
import com.team1701.lib.drivers.cameras.neural.DetectorCameraIO;
import com.team1701.lib.estimation.PoseEstimator.VisionMeasurement;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.Robot;
import com.team1701.robot.states.RobotState;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class Vision extends SubsystemBase {
    private final RobotState mRobotState;
    private final AprilTagFieldLayout mAprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    private final List<AprilTagCamera> mAprilTagCameras = new ArrayList<>();
    private final List<DetectorCamera> mDetectorCameras = new ArrayList<>();
    private final List<EstimatedRobotPose> mEstimatedRobotPoses = new ArrayList<>();
    private Optional<VisionSystemSim> mVisionSim = Optional.empty();

    public Vision(RobotState robotState, AprilTagCameraIO[] cameraIOs, DetectorCameraIO[] detectorCameraIOs) {

        mRobotState = robotState;

        Supplier<AprilTagFieldLayout> fieldLayoutSupplier = () -> mAprilTagFieldLayout;

        var CubVisionConfigTable = NetworkTableInstance.getDefault().getTable("CubVision/config");

        try {
            CubVisionConfigTable.getStringTopic("tag_layout")
                    .publish()
                    .set(new ObjectMapper().writeValueAsString(mAprilTagFieldLayout));
        } catch (JsonProcessingException e) {
            Alert.error("Failed to load AprilTag layout for Vision").enable();
        }

        CubVisionConfigTable.getIntegerArrayTopic("valid_ids").publish().set(Constants.Vision.kValidOnboardIds);
        CubVisionConfigTable.getStringArrayTopic("bus_keys").publish().set(Constants.Vision.kBusKeys);

        for (AprilTagCameraIO cameraIO : cameraIOs) {
            mAprilTagCameras.add(new AprilTagCamera(
                    cameraIO,
                    fieldLayoutSupplier,
                    mRobotState::getPose3d,
                    this::getStdDevForDistance,
                    Constants.Vision.kSingleTargetMode));
        }

        if (Robot.isSimulation()) {
            var visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(mAprilTagFieldLayout);

            var cameraProperties = new SimCameraProperties();
            cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(70));
            cameraProperties.setCalibError(0.035, 0.010);
            cameraProperties.setFPS(70);
            cameraProperties.setAvgLatencyMs(10);
            cameraProperties.setLatencyStdDevMs(5);

            mAprilTagCameras.forEach(camera -> camera.addToVisionSim(visionSim, cameraProperties));

            mVisionSim = Optional.of(visionSim);
        }

        mAprilTagCameras.forEach(camera -> {
            camera.addEstimatedPoseConsumer(mEstimatedRobotPoses::add);
            camera.addTargetFilter(target -> target.ambiguity < Constants.Vision.kAmbiguityThreshold);
            camera.addPoseFilter(pose -> poseIsInField(pose));
        });

        for (DetectorCameraIO cameraIO : detectorCameraIOs) {
            var cam = new DetectorCamera(cameraIO, mRobotState::getPose3d);
            cam.addDetectedObjectConsumer(mRobotState::addDetectedNotes);
            mDetectorCameras.add(cam);
        }
    }

    // This doesn't account for the Source's weird shape and the bumpers, but that's fine for now.
    private boolean poseIsInField(Pose3d pose) {
        var y = pose.getY();
        var x = pose.getX();
        var z = pose.getZ();

        return y >= 0
                && y <= FieldConstants.kFieldShortLengthMeters
                && x >= 0
                && x <= FieldConstants.kFieldLongLengthMeters
                && z >= -1.0
                && z < 1.0;
    }

    private Vector<N3> getStdDevForDistance(StdDevArguments args) {
        var distance = args.avgDistance();
        var numTargets = args.numTargets();
        var scalar = args.scalar();
        if (!Constants.Vision.kUseInterpolatedVisionStdDevValues) {

            var xy = Constants.Vision.kXYStdDevCoefficient * distance * distance * scalar / numTargets;
            var rotation = numTargets > 1
                    ? Constants.Vision.kRotationStdDevCoefficient * distance * distance * scalar / numTargets
                    : Double.POSITIVE_INFINITY;
            return VecBuilder.fill(xy, xy, rotation);
        }

        if (numTargets > 1) {
            return VecBuilder.fill(
                    Constants.Vision.kVisionXStdDevInterpolater.get(distance) * scalar,
                    Constants.Vision.kVisionYStdDevInterpolater.get(distance) * scalar,
                    Constants.Vision.kVisionThetaStdDevInterpolater.get(distance) * scalar);
        }

        return VecBuilder.fill(
                Constants.Vision.kVisionXStdDevInterpolater.get(distance)
                        * scalar
                        * Constants.Vision.kSingleTargetStdDevScalar,
                Constants.Vision.kVisionYStdDevInterpolater.get(distance)
                        * scalar
                        * Constants.Vision.kSingleTargetStdDevScalar,
                Double.POSITIVE_INFINITY);
    }

    @Override
    public void periodic() {
        mAprilTagCameras.forEach(AprilTagCamera::periodic);
        mDetectorCameras.forEach(DetectorCamera::periodic);

        mRobotState.addVisionMeasurements(mEstimatedRobotPoses.stream()
                .map(estimation -> new VisionMeasurement(
                        estimation.timestamp(), estimation.estimatedPose().toPose2d(), estimation.stdDevs()))
                .toArray(VisionMeasurement[]::new));

        mEstimatedRobotPoses.clear();
    }

    @Override
    public void simulationPeriodic() {
        if (mVisionSim.isEmpty()) {
            return;
        }

        mVisionSim.get().update(mRobotState.getPose2d());
        Logger.recordOutput("Vision/SimPose", mVisionSim.get().getDebugField().getRobotPose());
    }

    public Optional<Boolean> detectorCameraIsConnected(int index) {
        if (mDetectorCameras.size() >= index + 1) {
            return Optional.of(mDetectorCameras.get(index).isConnected());
        }
        return Optional.empty();
    }
}
