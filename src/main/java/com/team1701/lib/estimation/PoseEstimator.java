package com.team1701.lib.estimation;

import java.util.Comparator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface PoseEstimator {
    public static record DriveMeasurement(
            double timestampSeconds, Rotation2d gyroAngle, SwerveDriveWheelPositions wheelPositions) {
        public DriveMeasurement(double timestampSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
            this(timestampSeconds, gyroAngle, new SwerveDriveWheelPositions(modulePositions));
        }

        public static final Comparator<DriveMeasurement> compareTimestamp =
                (DriveMeasurement a, DriveMeasurement b) -> Double.compare(a.timestampSeconds, b.timestampSeconds);
    }

    public static record VisionMeasurement(double timestampSeconds, Pose2d pose, Matrix<N3, N1> stdDevs) {
        public static final Comparator<VisionMeasurement> compareStdDevDesc =
                (VisionMeasurement a, VisionMeasurement b) -> -Double.compare(
                        a.stdDevs().get(0, 0) + a.stdDevs().get(1, 0),
                        b.stdDevs().get(0, 0) + b.stdDevs().get(1, 0));
    }

    public void resetPose(Pose2d pose);

    public Pose2d getEstimatedPose();

    public Pose2d getEstimatedPose(double timestampSeconds);

    public void addDriveMeasurements(DriveMeasurement... measurements);

    public void addVisionMeasurements(VisionMeasurement... visionMeasurements);
}
