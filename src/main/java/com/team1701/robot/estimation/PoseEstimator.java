package com.team1701.robot.estimation;

import java.util.stream.Stream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Constants;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.AutoLogOutput;

public class PoseEstimator {
    private static PoseEstimator mInstance = null;

    private Rotation2d mGyroAngle = GeometryUtil.kRotationIdentity;
    private SwerveModulePosition[] mModulePositions = Stream.generate(SwerveModulePosition::new)
            .limit(Constants.Drive.kNumModules)
            .toArray(SwerveModulePosition[]::new);
    private SwerveDrivePoseEstimator mPoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Drive.kKinematics, mGyroAngle, mModulePositions, GeometryUtil.kPoseIdentity);

    public static PoseEstimator getInstance() {
        if (mInstance == null) {
            synchronized (PoseEstimator.class) {
                if (mInstance == null) {
                    mInstance = new PoseEstimator();
                }
            }
        }

        return mInstance;
    }

    private PoseEstimator() {}

    @AutoLogOutput()
    public Pose2d getPose2d() {
        return mPoseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput()
    public Pose3d getPose3d() {
        return new Pose3d(mPoseEstimator.getEstimatedPosition());
    }

    public void update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        mGyroAngle = gyroAngle;
        mModulePositions = modulePositions;
        mPoseEstimator.update(gyroAngle, modulePositions);
    }

    public void updateWithTime(double timeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        mGyroAngle = gyroAngle;
        mModulePositions = modulePositions;
        mPoseEstimator.updateWithTime(timeSeconds, gyroAngle, modulePositions);
    }

    public void setPose(Pose2d pose) {
        resetPosition(mGyroAngle, mModulePositions, pose);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        mPoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        mGyroAngle = gyroAngle;
        mModulePositions = modulePositions;
        mPoseEstimator.resetPosition(gyroAngle, modulePositions, pose);
    }
}
