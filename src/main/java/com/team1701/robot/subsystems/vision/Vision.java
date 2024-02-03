package com.team1701.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.team1701.lib.alerts.Alert;
import com.team1701.lib.drivers.cameras.AprilTagCamera;
import com.team1701.lib.drivers.cameras.AprilTagCameraIO;
import com.team1701.robot.Constants;
import com.team1701.robot.Robot;
import com.team1701.robot.states.RobotState;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    private final RobotState mRobotState;
    private AprilTagFieldLayout mAprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    private final ArrayList<AprilTagCamera> mCameras = new ArrayList<AprilTagCamera>();
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
            cameraProperties.setCalibError(0.35, 0.10);
            cameraProperties.setFPS(15);
            cameraProperties.setAvgLatencyMs(50);
            cameraProperties.setLatencyStdDevMs(15);

            mCameras.forEach(camera -> camera.addToVisionSim(visionSim, cameraProperties));

            mVisionSim = Optional.of(visionSim);
        }

        Consumer<? super AprilTagCamera> addMainEstimatedPoseConsumer;
        if (Constants.Vision.kUseInterpolatedVisionStdDevValues) {
            addMainEstimatedPoseConsumer = camera -> {
                camera.addEstimatedPoseConsumer(estimation -> {
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

                    mRobotState.addVisionMeasurement(
                            estimation.estimatedPose.toPose2d(),
                            estimation.timestampSeconds,
                            VecBuilder.fill(
                                    interpolatedXYStdDeviation,
                                    interpolatedXYStdDeviation,
                                    interpolatedThetaStdDeviation));
                });
            };
        } else {
            addMainEstimatedPoseConsumer = camera -> {
                camera.addEstimatedPoseConsumer(estimation -> mRobotState.addVisionMeasurement(
                        estimation.estimatedPose.toPose2d(), estimation.timestampSeconds));
            };
        }

        mCameras.forEach(addMainEstimatedPoseConsumer);
        mCameras.forEach(camera ->
                camera.addTargetFilter(target -> target.getPoseAmbiguity() < Constants.Vision.kAmbiguityThreshold));
    }

    @Override
    public void periodic() {
        mCameras.forEach(AprilTagCamera::periodic);
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
