package com.team1701.robot.states;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import com.team1701.lib.estimation.PoseEstimator1;
import com.team1701.lib.estimation.PoseEstimator1.DriveMeasurement;
import com.team1701.lib.estimation.PoseEstimator2;
import com.team1701.lib.estimation.PoseEstimator3;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
    private static final double kDetectedNoteTimeout = 1.0;

    private Rotation2d mLastGyroAngle = GeometryUtil.kRotationIdentity;
    private SwerveDriveWheelPositions mLastSwerveModulePositions =
            new SwerveDriveWheelPositions(Stream.generate(SwerveModulePosition::new)
                    .limit(Constants.Drive.kNumModules)
                    .toArray(SwerveModulePosition[]::new));

    private final PoseEstimator1 mPoseEstimator1 =
            new PoseEstimator1(Constants.Drive.kKinematics, VecBuilder.fill(0.005, 0.005, 0.0005));
    private final PoseEstimator2 mPoseEstimator2 = new PoseEstimator2(VecBuilder.fill(0.005, 0.005, 0.0005));
    private final PoseEstimator3 mPoseEstimator3 = new PoseEstimator3(VecBuilder.fill(0.005, 0.005, 0.0005));
    private final SwerveDrivePoseEstimator mPoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Drive.kKinematics,
            mLastGyroAngle,
            mLastSwerveModulePositions.positions,
            GeometryUtil.kPoseIdentity,
            // TODO: Collect values for standard state deviation
            // Standard state deviation defaults to 6328's
            // Vision is the default we've been using anyways.
            VecBuilder.fill(0.005, 0.005, 0.0005),
            VecBuilder.fill(0.9, 0.9, 0.9));

    private final List<NoteState> mDetectedNotes = new ArrayList<>();

    public void periodic() {
        var timeout = Timer.getFPGATimestamp() - kDetectedNoteTimeout;
        mDetectedNotes.removeIf(note -> note.lastDetectedTimestamp < timeout);
    }

    @AutoLogOutput
    public double getMatchTime() {
        return DriverStation.getMatchTime();
    }

    @AutoLogOutput
    public Pose2d getPose2d() {
        return mPoseEstimator1.getEstimatedPose();
    }

    @AutoLogOutput
    public Pose3d getPose3d() {
        return new Pose3d(mPoseEstimator1.getEstimatedPose());
    }

    public Rotation2d getHeading() {
        return getPose2d().getRotation();
    }

    public void addDriveMeasurement(DriveMeasurement... driveMeasurements) {
        var driveMeasurements2 = new PoseEstimator2.DriveMeasurement[driveMeasurements.length];
        var driveMeasurements3 = new PoseEstimator3.DriveMeasurement[driveMeasurements.length];
        for (int i = 0; i < driveMeasurements.length; i++) {
            var measurement = driveMeasurements[i];
            var twist =
                    Constants.Drive.kKinematics.toTwist2d(mLastSwerveModulePositions, measurement.modulePositions());
            twist = new Twist2d(
                    twist.dx,
                    twist.dy,
                    measurement.gyroAngle().minus(mLastGyroAngle).getRadians());

            driveMeasurements2[i] = new PoseEstimator2.DriveMeasurement(measurement.timestampSeconds(), twist);
            driveMeasurements3[i] = new PoseEstimator3.DriveMeasurement(measurement.timestampSeconds(), twist);

            mLastGyroAngle = measurement.gyroAngle();
            mLastSwerveModulePositions = measurement.modulePositions();
        }

        var start = Logger.getRealTimestamp();
        for (var driveMeasurement : driveMeasurements) {
            mPoseEstimator.updateWithTime(
                    driveMeasurement.timestampSeconds(),
                    driveMeasurement.gyroAngle(),
                    driveMeasurement.modulePositions().positions);
        }
        var stop = Logger.getRealTimestamp();

        Logger.recordOutput("PoseEstimatorTuning/0/Time", stop - start);
        Logger.recordOutput("PoseEstimatorTuning/0/Pose", mPoseEstimator.getEstimatedPosition());

        start = Logger.getRealTimestamp();
        for (var driveMeasurement : driveMeasurements) {
            mPoseEstimator1.addDriveMeasurement(driveMeasurement);
        }
        stop = Logger.getRealTimestamp();

        Logger.recordOutput("PoseEstimatorTuning/1/Time", stop - start);
        Logger.recordOutput("PoseEstimatorTuning/1/Pose", mPoseEstimator1.getEstimatedPose());

        start = Logger.getRealTimestamp();
        mPoseEstimator2.addDriveMeasurements(driveMeasurements2);
        stop = Logger.getRealTimestamp();

        Logger.recordOutput("PoseEstimatorTuning/2/Time", stop - start);
        Logger.recordOutput("PoseEstimatorTuning/2/Pose", mPoseEstimator2.getEstimatedPose());

        start = Logger.getRealTimestamp();
        mPoseEstimator3.addDriveMeasurements(driveMeasurements3);
        stop = Logger.getRealTimestamp();

        Logger.recordOutput("PoseEstimatorTuning/3/Time", stop - start);
        Logger.recordOutput("PoseEstimatorTuning/3/Pose", mPoseEstimator3.getEstimatedPose());
    }

    public void addVisionMeasurements(PoseEstimator1.VisionMeasurement... visionMeasurements) {
        mPoseEstimator1.addVisionMeasurements(visionMeasurements);

        var visionMeasurements2 = new PoseEstimator2.VisionMeasurement[visionMeasurements.length];
        var visionMeasurements3 = new PoseEstimator3.VisionMeasurement[visionMeasurements.length];
        for (int i = 0; i < visionMeasurements.length; i++) {
            var measurement = visionMeasurements[i];
            visionMeasurements2[i] = new PoseEstimator2.VisionMeasurement(
                    measurement.timestampSeconds(), measurement.pose(), measurement.stdDevs());
            visionMeasurements3[i] = new PoseEstimator3.VisionMeasurement(
                    measurement.timestampSeconds(), measurement.pose(), measurement.stdDevs());
        }

        var start = Logger.getRealTimestamp();
        for (var visionMeasurement : visionMeasurements) {
            mPoseEstimator.addVisionMeasurement(visionMeasurement.pose(), visionMeasurement.timestampSeconds());
        }
        var stop = Logger.getRealTimestamp();

        Logger.recordOutput("PoseEstimatorTuning/0/VisionTime", stop - start);

        start = Logger.getRealTimestamp();
        mPoseEstimator1.addVisionMeasurements(visionMeasurements);
        stop = Logger.getRealTimestamp();

        Logger.recordOutput("PoseEstimatorTuning/1/VisionTime", stop - start);

        start = Logger.getRealTimestamp();
        mPoseEstimator2.addVisionMeasurements(visionMeasurements2);
        stop = Logger.getRealTimestamp();

        Logger.recordOutput("PoseEstimatorTuning/2/VisionTime", stop - start);

        start = Logger.getRealTimestamp();
        mPoseEstimator3.addVisionMeasurements(visionMeasurements3);
        stop = Logger.getRealTimestamp();

        Logger.recordOutput("PoseEstimatorTuning/3/VisionTime", stop - start);
    }

    public void resetPose(Pose2d pose) {
        mPoseEstimator1.resetPose(pose);
        mPoseEstimator2.resetPose(pose);
        mPoseEstimator3.resetPose(pose);
    }

    @AutoLogOutput
    public boolean inWing() {
        var poseX = getPose2d().getX();
        return Configuration.isBlueAlliance()
                ? poseX < FieldConstants.kWingLength
                : poseX > FieldConstants.kFieldLongLengthMeters - FieldConstants.kWingLength;
    }

    @AutoLogOutput
    public boolean inOpponentWing() {
        var poseX = getPose2d().getX();
        return Configuration.isBlueAlliance()
                ? poseX > FieldConstants.kFieldLongLengthMeters - FieldConstants.kWingLength
                : poseX < FieldConstants.kWingLength;
    }

    public Translation3d getSpeakerPose() {
        return Configuration.isBlueAlliance()
                ? FieldConstants.kBlueSpeakerOpeningCenter
                : FieldConstants.kRedSpeakerOpeningCenter;
    }

    @AutoLogOutput
    public Rotation2d getSpeakerHeading() {
        return getSpeakerPose()
                .toTranslation2d()
                .minus(getPose2d().getTranslation())
                .getAngle();
    }

    @AutoLogOutput
    public Translation2d[] getDetectedNotePoses2d() {
        return mDetectedNotes.stream().map(note -> note.pose).toArray(Translation2d[]::new);
    }

    @AutoLogOutput
    public Translation3d[] getDetectedNotePoses3d() {
        return mDetectedNotes.stream()
                .map(note -> new Translation3d(note.pose.getX(), note.pose.getY(), 0.0))
                .toArray(Translation3d[]::new);
    }

    public void addDetectedNotes(List<NoteState> notes) {
        for (var newNote : notes) {
            mDetectedNotes.removeIf(existingNote -> existingNote.isSame(newNote));
        }

        mDetectedNotes.addAll(notes);
    }

    @AutoLogOutput
    public boolean hasNote() {
        // TODO: implement
        return false;
    }
}
