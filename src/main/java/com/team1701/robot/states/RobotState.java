package com.team1701.robot.states;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.team1701.lib.estimation.PoseEstimator;
import com.team1701.lib.estimation.PoseEstimator.DriveMeasurement;
import com.team1701.lib.estimation.PoseEstimator.VisionMeasurement;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
    private static final double kDetectedNoteTimeout = 1.0;

    private final TimeLockedBoolean mHasNote = new TimeLockedBoolean(0.1, Timer.getFPGATimestamp(), true, false);

    private Optional<Indexer> mIndexer = Optional.empty();
    private Optional<Intake> mIntake = Optional.empty();
    private Optional<Shooter> mShooter = Optional.empty();

    private final PoseEstimator mPoseEstimator =
            new PoseEstimator(Constants.Drive.kKinematics, VecBuilder.fill(0.005, 0.005, 0.0005));
    private final List<NoteState> mDetectedNotes = new ArrayList<>();

    public void addSubsystems(Shooter shooter, Indexer indexer, Intake intake) {
        mIndexer = Optional.of(indexer);
        mIntake = Optional.of(intake);
        mShooter = Optional.of(shooter);
    }

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
        return mPoseEstimator.getEstimatedPose();
    }

    @AutoLogOutput
    public Pose3d getPose3d() {
        return new Pose3d(mPoseEstimator.getEstimatedPose());
    }

    @AutoLogOutput
    public double getDistanceToSpeaker() {
        return getPose3d()
                .getTranslation()
                .getDistance(new Translation3d(0.47560569643974304, 5.553, FieldConstants.kSpeakerHeight));
    }

    public Rotation2d getHeading() {
        return getPose2d().getRotation();
    }

    public void addDriveMeasurements(DriveMeasurement... driveMeasurements) {
        mPoseEstimator.addDriveMeasurements(driveMeasurements);
    }

    public void addVisionMeasurements(VisionMeasurement... visionMeasurements) {
        mPoseEstimator.addVisionMeasurements(visionMeasurements);
    }

    public void resetPose(Pose2d pose) {
        mPoseEstimator.resetPose(pose);
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
    public Pose2d[] getDetectedNotePoses2d() {
        return mDetectedNotes.stream()
                .map(note -> note.pose.getTranslation().toTranslation2d())
                .toArray(Pose2d[]::new);
    }

    @AutoLogOutput
    public Pose3d[] getDetectedNotePoses3d() {
        return mDetectedNotes.stream().map(note -> note.pose).toArray(Pose3d[]::new);
    }

    public void addDetectedNotes(List<NoteState> notes) {
        for (var newNote : notes) {
            mDetectedNotes.removeIf(existingNote -> existingNote.isSame(newNote));
        }

        mDetectedNotes.addAll(notes);
    }

    @AutoLogOutput
    public boolean hasNote() {
        if (mIntake.isEmpty() || mIndexer.isEmpty()) {
            return false;
        }
        var hasNote = mIntake.get().hasNote() || mIndexer.get().hasNote();
        return mHasNote.update(hasNote, Timer.getFPGATimestamp());
    }

    @AutoLogOutput
    public Pose3d getShooterExitPose() {
        var shooterHingePose = getPose3d().transformBy(Constants.Robot.kRobotToShooterHinge);
        return new Pose3d(
                shooterHingePose.getTranslation(),
                new Rotation3d(
                        shooterHingePose.getRotation().getX(),
                        shooterHingePose.getRotation().getY()
                                - mShooter.get().getAngle().getRadians(),
                        shooterHingePose.getRotation().getZ()));
    }

    @AutoLogOutput
    public Rotation2d calculateShooterAngleTowardsSpeaker() {
        var translationToSpeaker = getSpeakerPose()
                .minus(getPose3d()
                        .transformBy(Constants.Robot.kRobotToShooterHinge)
                        .getTranslation());

        return new Rotation2d(translationToSpeaker.toTranslation2d().getNorm(), translationToSpeaker.getZ());
    }
}
