package com.team1701.robot.states;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
    private static final double kDetectedNoteTimeout = 1.0;

    private Rotation2d mGyroAngle = GeometryUtil.kRotationIdentity;
    private SwerveModulePosition[] mModulePositions = Stream.generate(SwerveModulePosition::new)
            .limit(Constants.Drive.kNumModules)
            .toArray(SwerveModulePosition[]::new);
    private SwerveDrivePoseEstimator mPoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Drive.kKinematics, mGyroAngle, mModulePositions, GeometryUtil.kPoseIdentity);
    private List<NoteState> mDetectedNotes = new ArrayList<>();

    public void periodic() {
        var timeout = Timer.getFPGATimestamp() - kDetectedNoteTimeout;
        mDetectedNotes.removeIf(note -> note.lastDetectedTimestamp < timeout);
    }

    @AutoLogOutput
    public Pose2d getPose2d() {
        return mPoseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput
    public Pose3d getPose3d() {
        return new Pose3d(mPoseEstimator.getEstimatedPosition());
    }

    public Rotation2d getHeading() {
        return getPose2d().getRotation();
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

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        mPoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public void resetPose(Pose2d pose) {
        resetPose(mGyroAngle, mModulePositions, pose);
    }

    public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        mGyroAngle = gyroAngle;
        mModulePositions = modulePositions;
        mPoseEstimator.resetPosition(gyroAngle, modulePositions, pose);
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
