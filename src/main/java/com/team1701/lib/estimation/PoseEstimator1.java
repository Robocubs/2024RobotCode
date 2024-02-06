package com.team1701.lib.estimation;

import java.util.Arrays;
import java.util.Objects;
import java.util.stream.Stream;

import com.team1701.lib.swerve.ExtendedSwerveDriveKinematics;
import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimator1 {
    private final ExtendedSwerveDriveKinematics mKinematics;
    private final Odometry<SwerveDriveWheelPositions> mOdometry;
    private final Matrix<N3, N1> mQ = new Matrix<>(Nat.N3(), Nat.N1());

    private static final double kBufferDuration = 0.5;
    private final TimeInterpolatableBuffer<InterpolationRecord> mPoseBuffer =
            TimeInterpolatableBuffer.createBuffer(kBufferDuration);

    private DriveMeasurement mLastDriveMeasurement;

    public static record DriveMeasurement(
            double timestampSeconds, Rotation2d gyroAngle, SwerveDriveWheelPositions modulePositions) {
        public DriveMeasurement(double timestampSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
            this(timestampSeconds, gyroAngle, new SwerveDriveWheelPositions(modulePositions));
        }
    }

    public static record VisionMeasurement(double timestampSeconds, Pose2d pose, Matrix<N3, N1> stdDevs) {}

    public PoseEstimator1(ExtendedSwerveDriveKinematics kinematics, Matrix<N3, N1> stateStdDevs) {
        mKinematics = kinematics;

        var positions = new SwerveModulePosition[kinematics.getNumModules()];
        Arrays.fill(positions, new SwerveModulePosition());
        mOdometry = new Odometry<>(
                mKinematics,
                GeometryUtil.kRotationIdentity,
                new SwerveDriveWheelPositions(positions),
                GeometryUtil.kPoseIdentity);

        for (int i = 0; i < 3; ++i) {
            mQ.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
    }

    public void resetPose(Pose2d pose) {
        mOdometry.resetPosition(mLastDriveMeasurement.gyroAngle, mLastDriveMeasurement.modulePositions, pose);
    }

    public Pose2d getEstimatedPose() {
        return mOdometry.getPoseMeters();
    }

    public void addDriveMeasurement(DriveMeasurement measurement) {
        var modulePositions = measurement.modulePositions.copy();
        mOdometry.update(measurement.gyroAngle, modulePositions);
        mPoseBuffer.addSample(
                measurement.timestampSeconds,
                new InterpolationRecord(mOdometry.getPoseMeters(), measurement.gyroAngle, modulePositions));
        mLastDriveMeasurement =
                new DriveMeasurement(measurement.timestampSeconds, measurement.gyroAngle, modulePositions);
    }

    public void addVisionMeasurements(VisionMeasurement[] visionMeasurements) {
        var bufferTimespanThreshold = mPoseBuffer.getInternalBuffer().lastKey() - kBufferDuration;

        Stream.of(visionMeasurements)
                .filter(measurement -> measurement.timestampSeconds() > bufferTimespanThreshold)
                .sorted((a, b) -> Double.compare(a.timestampSeconds(), b.timestampSeconds()))
                .toArray(VisionMeasurement[]::new);

        // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)

        for (var i = 0; i < visionMeasurements.length; i++) {
            var measurement = visionMeasurements[i];

            // Step 1: Get the pose odometry measured at the moment the vision measurement was made.
            var sample = mPoseBuffer.getSample(measurement.timestampSeconds);
            if (sample.isEmpty()) {
                return;
            }

            // Step 2: Measure the twist between the odometry pose and the vision pose.
            var twist = sample.get().poseMeters.log(measurement.pose);

            // Step 3: We should not trust the twist entirely, so instead we scale this twist by a Kalman
            // gain matrix representing how much we trust vision measurements compared to our current pose.
            var kTimesTwist =
                    calculateKalmanGain(measurement.stdDevs).times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

            // Step 4: Convert back to Twist2d.
            var scaledTwist = new Twist2d(kTimesTwist.get(0, 0), kTimesTwist.get(1, 0), kTimesTwist.get(2, 0));

            // Step 5: Reset Odometry to state at sample with vision adjustment.
            mOdometry.resetPosition(
                    sample.get().gyroAngle,
                    sample.get().wheelPositions,
                    sample.get().poseMeters.exp(scaledTwist));

            // Step 6: Record the current pose to allow multiple measurements from the same timestamp
            mPoseBuffer.addSample(
                    measurement.timestampSeconds,
                    new InterpolationRecord(getEstimatedPose(), sample.get().gyroAngle, sample.get().wheelPositions));

            // Step 7: Replay odometry inputs to update the pose buffer and correct odometry. If there is a next
            // measurement, only calculate what is needed for it.
            var entries = mPoseBuffer
                    .getInternalBuffer()
                    .tailMap(measurement.timestampSeconds)
                    .entrySet();
            var nextVisionTimestamp =
                    i + 1 < visionMeasurements.length ? visionMeasurements[i + 1].timestampSeconds() : Double.MAX_VALUE;
            for (var entry : entries) {
                addDriveMeasurement(new DriveMeasurement(
                        entry.getKey(), entry.getValue().gyroAngle, entry.getValue().wheelPositions));

                // Need to update one entry past next vision measurement to allow for interpolation
                if (entry.getKey() > nextVisionTimestamp) {
                    break;
                }
            }
        }
    }

    private Matrix<N3, N3> calculateKalmanGain(Matrix<N3, N1> visionMeasurementStdDevs) {
        // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
        }

        var visionK = new Matrix<>(Nat.N3(), Nat.N3());

        for (int row = 0; row < 3; ++row) {
            if (mQ.get(row, 0) == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, mQ.get(row, 0) / (mQ.get(row, 0) + Math.sqrt(mQ.get(row, 0) * r[row])));
            }
        }

        return visionK;
    }

    // TODO: Consider using Twist2d instead of gyro/wheels
    private class InterpolationRecord implements Interpolatable<InterpolationRecord> {
        private final Pose2d poseMeters;
        private final Rotation2d gyroAngle;
        private final SwerveDriveWheelPositions wheelPositions;

        private InterpolationRecord(Pose2d poseMeters, Rotation2d gyro, SwerveDriveWheelPositions wheelPositions) {
            this.poseMeters = poseMeters;
            this.gyroAngle = gyro;
            this.wheelPositions = wheelPositions;
        }

        @Override
        public InterpolationRecord interpolate(InterpolationRecord endValue, double t) {
            if (t < 0) {
                return this;
            } else if (t >= 1) {
                return endValue;
            } else {
                var wheelLerp = wheelPositions.interpolate(endValue.wheelPositions, t);
                var gyroLerp = gyroAngle.interpolate(endValue.gyroAngle, t);

                // Create a twist to represent the change based on the interpolated sensor inputs.
                Twist2d twist = mKinematics.toTwist2d(wheelPositions, wheelLerp);
                twist.dtheta = gyroLerp.minus(gyroAngle).getRadians();

                return new InterpolationRecord(poseMeters.exp(twist), gyroLerp, wheelLerp);
            }
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof PoseEstimator1.InterpolationRecord)) {
                return false;
            }
            var record = (InterpolationRecord) obj;
            return Objects.equals(gyroAngle, record.gyroAngle)
                    && Objects.equals(wheelPositions, record.wheelPositions)
                    && Objects.equals(poseMeters, record.poseMeters);
        }

        @Override
        public int hashCode() {
            return Objects.hash(gyroAngle, wheelPositions, poseMeters);
        }
    }
}
