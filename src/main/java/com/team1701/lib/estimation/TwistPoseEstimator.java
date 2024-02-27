package com.team1701.lib.estimation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.NavigableMap;
import java.util.TreeMap;

import com.team1701.lib.alerts.Alert;
import com.team1701.lib.swerve.ExtendedSwerveDriveKinematics;
import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

public class TwistPoseEstimator implements PoseEstimator {
    private static final double kHistorySeconds = 0.5;

    private final Alert mVisionAlert = Alert.error("Vision measurements added before drive measurements.");
    private final Matrix<N3, N1> mQ = new Matrix<>(Nat.N3(), Nat.N1());
    private final NavigableMap<Double, PoseUpdate> mUpdates = new TreeMap<>();
    private final ExtendedSwerveDriveKinematics mKinematics;

    private Pose2d mPose = GeometryUtil.kPoseIdentity;
    private DriveMeasurement mLastDriveMeasurement;

    public TwistPoseEstimator(ExtendedSwerveDriveKinematics kinematics, Matrix<N3, N1> stateStdDevs) {
        mKinematics = kinematics;

        var modulePositions = new SwerveModulePosition[kinematics.getNumModules()];
        Arrays.fill(modulePositions, new SwerveModulePosition());
        mLastDriveMeasurement = new DriveMeasurement(0.0, Rotation2d.fromDegrees(0.0), modulePositions);

        for (int i = 0; i < 3; ++i) {
            mQ.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
    }

    @Override
    public void resetPose(Pose2d pose) {
        mPose = pose;
        mUpdates.clear();
    }

    @Override
    public Pose2d getEstimatedPose() {
        return mPose;
    }

    @Override
    public Pose2d getEstimatedPose(double timestampSeconds) {
        var nextUpdate = mUpdates.ceilingEntry(timestampSeconds);
        if (nextUpdate == null) {
            return mPose;
        }

        var previousUpdate = mUpdates.floorEntry(timestampSeconds);
        if (previousUpdate == null) {
            return nextUpdate.getValue().basePose.exp(nextUpdate.getValue().twist);
        }

        var ratio = (timestampSeconds - previousUpdate.getKey()) / (nextUpdate.getKey() - previousUpdate.getKey());
        var twist = GeometryUtil.multiply(nextUpdate.getValue().twist, ratio);

        return nextUpdate.getValue().basePose.exp(twist);
    }

    @Override
    public void addDriveMeasurements(DriveMeasurement... measurements) {
        if (measurements.length == 0) {
            return;
        }

        Arrays.sort(measurements, DriveMeasurement.compareTimestamp);

        // Remove any updates that are newer than the new data
        // This should never happen, but it's good to be safe
        var firstMeasurementTimestamp = measurements[0].timestampSeconds();
        while (!mUpdates.isEmpty() && mUpdates.lastKey() > firstMeasurementTimestamp) {
            mUpdates.pollLastEntry();
        }

        // Add new data
        var previousUpdate = mUpdates.floorEntry(measurements[0].timestampSeconds());
        mPose = previousUpdate == null
                ? mPose
                : previousUpdate.getValue().basePose.exp(previousUpdate.getValue().twist);
        for (var measurement : measurements) {
            var twist = mKinematics.toTwist2d(mLastDriveMeasurement.wheelPositions(), measurement.wheelPositions());
            twist.dtheta = measurement
                    .gyroAngle()
                    .minus(mLastDriveMeasurement.gyroAngle())
                    .getRadians();

            mUpdates.put(measurement.timestampSeconds(), new PoseUpdate(mPose, twist, new ArrayList<>()));
            mPose = mPose.exp(twist);

            mLastDriveMeasurement = measurement;
        }

        // Clear old data
        while (mUpdates.size() > 1 && mUpdates.firstKey() < Timer.getFPGATimestamp() - kHistorySeconds) {
            mUpdates.remove(mUpdates.firstKey());
        }
    }

    @Override
    public void addVisionMeasurements(VisionMeasurement... visionMeasurements) {
        var minimumTimestamp = Timer.getFPGATimestamp() - kHistorySeconds;
        var firstMeasurementTimestamp = Double.MAX_VALUE;

        for (var measurement : visionMeasurements) {
            var timestamp = measurement.timestampSeconds();
            if (timestamp < minimumTimestamp) {
                continue;
            }

            if (mUpdates.containsKey(timestamp)) {
                // There was already an update at this timestamp, add to it
                var measurements = mUpdates.get(timestamp).visionMeasurements();
                measurements.add(measurement);
                measurements.sort(VisionMeasurement.compareStdDevDesc);

                firstMeasurementTimestamp = Math.min(firstMeasurementTimestamp, timestamp);
            } else {
                // Insert a new update
                var previousUpdate = mUpdates.floorEntry(timestamp);
                var nextUpdate = mUpdates.ceilingEntry(timestamp);

                if (previousUpdate == null || nextUpdate == null) {
                    // Outside the range of existing data
                    mVisionAlert.enable();
                    continue;
                }

                // Create partial twists (prev -> vision, vision -> next)
                var ratio = (timestamp - previousUpdate.getKey()) / (nextUpdate.getKey() - previousUpdate.getKey());
                var twist0 = GeometryUtil.multiply(nextUpdate.getValue().twist, ratio);
                var twist1 = GeometryUtil.multiply(nextUpdate.getValue().twist, 1.0 - ratio);

                // Add new pose updates
                var measurements = new ArrayList<VisionMeasurement>();
                measurements.add(measurement);
                measurements.sort(VisionMeasurement.compareStdDevDesc);
                mUpdates.put(timestamp, new PoseUpdate(previousUpdate.getValue().basePose, twist0, measurements));
                mUpdates.put(
                        nextUpdate.getKey(),
                        new PoseUpdate(
                                GeometryUtil.kPoseIdentity, // Will be overwritten when we update
                                twist1,
                                nextUpdate.getValue().visionMeasurements));

                firstMeasurementTimestamp = Math.min(firstMeasurementTimestamp, previousUpdate.getKey());
            }
        }

        // Recalculate starting at the first new measurement
        var tailMap = mUpdates.tailMap(firstMeasurementTimestamp, true);
        if (tailMap.isEmpty()) {
            return;
        }

        var mPose = tailMap.firstEntry().getValue().basePose;
        for (var update : tailMap.entrySet()) {
            var newEntry = new PoseUpdate(mPose, update.getValue().twist, update.getValue().visionMeasurements);
            tailMap.put(update.getKey(), newEntry);
            mPose = newEntry.apply(mPose, mQ);
        }
    }

    private static record PoseUpdate(Pose2d basePose, Twist2d twist, ArrayList<VisionMeasurement> visionMeasurements) {
        public Pose2d apply(Pose2d lastPose, Matrix<N3, N1> q) {
            // Apply drive twist
            var pose = lastPose.exp(twist);

            // Apply vision updates
            for (var visionMeasurement : visionMeasurements) {
                // Calculate Kalman gains based on std devs
                // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
                Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
                var r = new double[3];
                for (int i = 0; i < 3; ++i) {
                    r[i] = visionMeasurement.stdDevs().get(i, 0)
                            * visionMeasurement.stdDevs().get(i, 0);
                }
                for (int row = 0; row < 3; ++row) {
                    if (q.get(row, 0) == 0.0) {
                        visionK.set(row, row, 0.0);
                    } else {
                        visionK.set(row, row, q.get(row, 0) / (q.get(row, 0) + Math.sqrt(q.get(row, 0) * r[row])));
                    }
                }

                // Calculate twist between current and vision pose
                var visionTwist = pose.log(visionMeasurement.pose());

                // Multiply by Kalman gain matrix
                var twistMatrix = visionK.times(VecBuilder.fill(visionTwist.dx, visionTwist.dy, visionTwist.dtheta));

                // Apply twist
                pose = pose.exp(new Twist2d(twistMatrix.get(0, 0), twistMatrix.get(1, 0), twistMatrix.get(2, 0)));
            }

            return pose;
        }
    }
}
