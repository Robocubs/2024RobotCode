package com.team1701.lib.estimation;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.NavigableMap;
import java.util.TreeMap;

import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

public class PoseEstimator2 {
    private static final double kHistorySeconds = 0.1;

    private final Matrix<N3, N1> mQ = new Matrix<>(Nat.N3(), Nat.N1());
    private final NavigableMap<Double, PoseUpdate> mUpdates = new TreeMap<>();

    private Pose2d mBasePose = GeometryUtil.kPoseIdentity;
    private Pose2d mPose = GeometryUtil.kPoseIdentity;

    public static record DriveMeasurement(double timestampSeconds, Twist2d twist2d) {}

    public static record VisionMeasurement(double timestampSeconds, Pose2d pose, Matrix<N3, N1> stdDevs) {
        private static final Comparator<VisionMeasurement> compareDescStdDev =
                (VisionMeasurement a, VisionMeasurement b) -> {
                    return -Double.compare(
                            a.stdDevs().get(0, 0) + a.stdDevs().get(1, 0),
                            b.stdDevs().get(0, 0) + b.stdDevs().get(1, 0));
                };
    }

    public PoseEstimator2(Matrix<N3, N1> stateStdDevs) {
        for (int i = 0; i < 3; ++i) {
            mQ.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
    }

    public void resetPose(Pose2d pose) {
        mBasePose = pose;
        mPose = pose;
        mUpdates.clear();
    }

    public Pose2d getEstimatedPose() {
        return mPose;
    }

    // TODO: Handle exisitng vision measurements
    public void addDriveMeasurements(DriveMeasurement... driveMeasurements) {
        for (var measurement : driveMeasurements) {
            mUpdates.put(measurement.timestampSeconds, new PoseUpdate(measurement.twist2d, new ArrayList<>()));
        }

        // Recalculate latest pose once
        update();
    }

    // TODO: Add alarm to check if vision measurements happen before drive measurements
    public void addVisionMeasurements(VisionMeasurement... visionMeasurements) {
        for (var measurement : visionMeasurements) {
            var timestamp = measurement.timestampSeconds;

            if (mUpdates.containsKey(timestamp)) {
                // There was already an update at this timestamp, add to it
                var measurements = mUpdates.get(timestamp).visionMeasurements();
                measurements.add(measurement);
                measurements.sort(VisionMeasurement.compareDescStdDev);

            } else {
                // Insert a new update
                var previousUpdate = mUpdates.floorEntry(timestamp);
                var nextUpdate = mUpdates.ceilingEntry(timestamp);

                // TODO: Handle scenario
                if (previousUpdate == null || nextUpdate == null) {
                    // Outside the range of existing data
                    return;
                }

                // Create partial twists (prev -> vision, vision -> next)
                var ratio = (timestamp - previousUpdate.getKey()) / (nextUpdate.getKey() - previousUpdate.getKey());
                var twist0 = GeometryUtil.multiply(nextUpdate.getValue().twist(), ratio);
                var twist1 = GeometryUtil.multiply(nextUpdate.getValue().twist(), 1.0 - ratio);

                // Add new pose updates
                var measurements = new ArrayList<VisionMeasurement>();
                measurements.add(measurement);
                measurements.sort(VisionMeasurement.compareDescStdDev);
                mUpdates.put(timestamp, new PoseUpdate(twist0, measurements));
                mUpdates.put(nextUpdate.getKey(), new PoseUpdate(twist1, nextUpdate.getValue().visionMeasurements));
            }
        }

        // Recalculate latest pose once
        update();
    }

    public void update() {
        // Clear old data and update base pose
        while (mUpdates.size() > 1 && mUpdates.firstKey() < Timer.getFPGATimestamp() - kHistorySeconds) {
            var update = mUpdates.pollFirstEntry();
            mBasePose = update.getValue().apply(mBasePose, mQ);
        }

        // Update latest pose
        mPose = mBasePose;
        for (var updateEntry : mUpdates.entrySet()) {
            mPose = updateEntry.getValue().apply(mPose, mQ);
        }
    }

    private static record PoseUpdate(Twist2d twist, ArrayList<VisionMeasurement> visionMeasurements) {
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
