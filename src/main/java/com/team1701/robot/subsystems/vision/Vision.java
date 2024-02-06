package com.team1701.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.team1701.lib.alerts.Alert;
import com.team1701.lib.drivers.cameras.AprilTagCamera;
import com.team1701.lib.drivers.cameras.AprilTagCameraIO;
import com.team1701.lib.estimation.PoseEstimator.VisionMeasurement;
import com.team1701.robot.Constants;
import com.team1701.robot.Robot;
import com.team1701.robot.states.RobotState;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    private static final Vector<N3> kDefaultStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);

    private final RobotState mRobotState;
    private AprilTagFieldLayout mAprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    private final List<AprilTagCamera> mCameras = new ArrayList<>();
    private final List<EstimatedRobotPose> mEstimatedRobotPoses = new ArrayList<>();
    private Optional<VisionSystemSim> mVisionSim = Optional.empty();

    public Vision(
            RobotState robotState,
            AprilTagCameraIO cameraIOFrontLeft,
            AprilTagCameraIO cameraIOFrontRight,
            AprilTagCameraIO cameraIOBackLeft,
            AprilTagCameraIO cameraIOBackRight) {

        mRobotState = robotState;

        Supplier<AprilTagFieldLayout> fieldLayoutSupplier = () -> mAprilTagFieldLayout;

        var CubVisionConfigTable = NetworkTableInstance.getDefault().getTable("CubVision/config");

        try {
            CubVisionConfigTable.getStringTopic("tag_layout")
                    .publish()
                    .set(new ObjectMapper().writeValueAsString(fieldLayoutSupplier.get()));
        } catch (JsonProcessingException e) {
            Alert.error("Failed to load AprilTag layout for Vision").enable();
        }

        mCameras.add(new AprilTagCamera(
                Constants.Vision.kFrontLeftCameraName,
                cameraIOFrontLeft,
                Constants.Vision.kRobotToFrontLeftCamPose,
                Constants.Vision.kPoseStrategy,
                Constants.Vision.kFallbackPoseStrategy,
                fieldLayoutSupplier,
                mRobotState::getPose3d));
        mCameras.add(new AprilTagCamera(
                Constants.Vision.kFrontRightCameraName,
                cameraIOFrontRight,
                Constants.Vision.kRobotToFrontRightCamPose,
                Constants.Vision.kPoseStrategy,
                Constants.Vision.kFallbackPoseStrategy,
                fieldLayoutSupplier,
                mRobotState::getPose3d));
        mCameras.add(new AprilTagCamera(
                Constants.Vision.kBackLeftCameraName,
                cameraIOBackLeft,
                Constants.Vision.kRobotToBackLeftCamPose,
                Constants.Vision.kPoseStrategy,
                Constants.Vision.kFallbackPoseStrategy,
                fieldLayoutSupplier,
                mRobotState::getPose3d));
        mCameras.add(new AprilTagCamera(
                Constants.Vision.kBackRightCameraName,
                cameraIOBackRight,
                Constants.Vision.kRobotToBackRightCamPose,
                Constants.Vision.kPoseStrategy,
                Constants.Vision.kFallbackPoseStrategy,
                fieldLayoutSupplier,
                mRobotState::getPose3d));

        if (Robot.isSimulation()) {
            var visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(mAprilTagFieldLayout);

            var cameraProperties = new SimCameraProperties();
            cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProperties.setCalibError(0.035, 0.010);
            cameraProperties.setFPS(70);
            cameraProperties.setAvgLatencyMs(10);
            cameraProperties.setLatencyStdDevMs(5);

            mCameras.forEach(camera -> camera.addToVisionSim(visionSim, cameraProperties));

            mVisionSim = Optional.of(visionSim);
        }

        mCameras.forEach(camera -> {
            camera.addEstimatedPoseConsumer(mEstimatedRobotPoses::add);
            camera.addTargetFilter(target -> target.getPoseAmbiguity() < Constants.Vision.kAmbiguityThreshold);
        });
    }

    @Override
    public void periodic() {
        mCameras.forEach(AprilTagCamera::periodic);

        mRobotState.addVisionMeasurements(mEstimatedRobotPoses.stream()
                .map(estimation -> {
                    if (!Constants.Vision.kUseInterpolatedVisionStdDevValues) {
                        return new VisionMeasurement(
                                estimation.timestampSeconds, estimation.estimatedPose.toPose2d(), kDefaultStdDevs);
                    }

                    double avgDistanceToTarget = 0.0;

                    for (PhotonTrackedTarget target : estimation.targetsUsed) {
                        // TODO: can I always use the bestCamToTarget here?
                        avgDistanceToTarget +=
                                target.getBestCameraToTarget().getTranslation().getNorm();
                    }
                    avgDistanceToTarget /= estimation.targetsUsed.size();

                    double interpolatedXYStdDeviation =
                            Constants.Vision.kVisionXYStdDevInterpolater.get(avgDistanceToTarget);
                    double interpolatedThetaStdDeviation =
                            Constants.Vision.kVisionThetaStdDevInterpolater.get(avgDistanceToTarget);

                    return new VisionMeasurement(
                            estimation.timestampSeconds,
                            estimation.estimatedPose.toPose2d(),
                            VecBuilder.fill(
                                    interpolatedXYStdDeviation,
                                    interpolatedXYStdDeviation,
                                    interpolatedThetaStdDeviation));
                })
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
}