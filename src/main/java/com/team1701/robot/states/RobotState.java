package com.team1701.robot.states;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.team1701.lib.drivers.cameras.neural.DetectorCamera.DetectedObjectState;
import com.team1701.lib.estimation.PoseEstimator;
import com.team1701.lib.estimation.PoseEstimator.DriveMeasurement;
import com.team1701.lib.estimation.PoseEstimator.VisionMeasurement;
import com.team1701.lib.estimation.TwistPoseEstimator;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableBoolean;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.Robot;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
    private static final double kDetectedNoteTimeout = 3.0;
    private static final double kDuplicateNoteDistanceThreshold = Units.inchesToMeters(10.0);

    private static final LoggedTunableBoolean mEnableCameraPoseEstimation =
            new LoggedTunableBoolean("EnableCameraPoseEstimation", true);

    private final TimeLockedBoolean mHasNote = new TimeLockedBoolean(0.1, Timer.getFPGATimestamp(), true, false);
    private final TimeLockedBoolean mOutOfAmpRange = new TimeLockedBoolean(0.25, Timer.getFPGATimestamp(), true, true);
    private final Field2d mField;

    private Optional<Indexer> mIndexer = Optional.empty();
    private Optional<Intake> mIntake = Optional.empty();
    private Optional<Shooter> mShooter = Optional.empty();

    private ShootingState mShootingState = new ShootingState();

    public RobotState() {
        mField = new Field2d();
        SmartDashboard.putData("Field", mField);
        SmartDashboard.putBoolean("IsSimulation", Robot.isSimulation());
    }

    @AutoLogOutput
    private ScoringMode mScoringMode = ScoringMode.SPEAKER;

    private final PoseEstimator mPoseEstimator =
            new TwistPoseEstimator(Constants.Drive.kKinematics, VecBuilder.fill(0.005, 0.005, 0.0005));
    private final List<DetectedObjectState> mDetectedNotes = new ArrayList<>();

    public void addSubsystems(Shooter shooter, Indexer indexer, Intake intake) {
        mIndexer = Optional.of(indexer);
        mIntake = Optional.of(intake);
        mShooter = Optional.of(shooter);
    }

    public void periodic() {
        var timeout = Timer.getFPGATimestamp() - kDetectedNoteTimeout;
        mDetectedNotes.removeIf(note -> note.timestamp() < timeout);
        mOutOfAmpRange.update(getDistanceToAmp() > 1, Timer.getFPGATimestamp());
        mField.setRobotPose(getPose2d());
    }

    @AutoLogOutput
    public double getMatchTime() {
        return DriverStation.getMatchTime();
    }

    @AutoLogOutput
    public Pose2d getPose2d() {
        return mPoseEstimator.getEstimatedPose();
    }

    public Pose2d getPose2d(double timestampSeconds) {
        return mPoseEstimator.getEstimatedPose(timestampSeconds);
    }

    @AutoLogOutput
    public Pose3d getPose3d() {
        return new Pose3d(mPoseEstimator.getEstimatedPose());
    }

    public Pose3d getPose3d(double timestampSeconds) {
        return new Pose3d(mPoseEstimator.getEstimatedPose(timestampSeconds));
    }

    @AutoLogOutput
    public double getDistanceToSpeaker() {
        return getPose3d()
                .getTranslation()
                .getDistance(
                        Configuration.isBlueAlliance()
                                ? FieldConstants.kBlueSpeakerOpeningCenter
                                : FieldConstants.kRedSpeakerOpeningCenter);
    }

    public Rotation2d getHeading() {
        return getPose2d().getRotation();
    }

    public void addDriveMeasurements(DriveMeasurement... driveMeasurements) {
        mPoseEstimator.addDriveMeasurements(driveMeasurements);
    }

    public void addVisionMeasurements(VisionMeasurement... visionMeasurements) {
        if (mEnableCameraPoseEstimation.get()) {
            Logger.recordOutput("RobotState/LastVisionMeasurement", Timer.getFPGATimestamp());
            mPoseEstimator.addVisionMeasurements(visionMeasurements);
        }
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

    @AutoLogOutput
    public boolean inNearHalf() {
        var poseX = getPose2d().getX();
        return Configuration.isBlueAlliance()
                ? poseX < FieldConstants.kFieldLongLengthMeters / 2.0
                : poseX > FieldConstants.kFieldLongLengthMeters / 2.0;
    }

    @AutoLogOutput
    public double getDistanceToAmp() {
        return getPose2d()
                .getTranslation()
                .getDistance(
                        Configuration.isBlueAlliance()
                                ? FieldConstants.kBlueAmpPosition.toTranslation2d()
                                : FieldConstants.kRedAmpPosition.toTranslation2d());
    }

    @AutoLogOutput
    public boolean outOfAmpRange() {
        return mOutOfAmpRange.getValue();
    }

    public Translation3d getSpeakerPose() {
        return Configuration.isBlueAlliance()
                ? FieldConstants.kBlueSpeakerOpeningCenter
                : FieldConstants.kRedSpeakerOpeningCenter;
    }

    public Translation3d getAmpPose() {
        return Configuration.isBlueAlliance() ? FieldConstants.kBlueAmpPosition : FieldConstants.kRedAmpPosition;
    }

    public Rotation2d getSpeakerHeading() {
        return getSpeakerPose()
                .toTranslation2d()
                .minus(getPose2d().getTranslation())
                .getAngle();
    }

    public Rotation2d getMovingSpeakerHeading(Drive drive) {
        var projectedTranslation = getPose2d()
                .getTranslation()
                .plus(new Translation2d(
                        drive.getFieldRelativeVelocity().vxMetersPerSecond * Constants.kLoopPeriodSeconds,
                        drive.getFieldRelativeVelocity().vyMetersPerSecond * Constants.kLoopPeriodSeconds));
        return getSpeakerPose().toTranslation2d().minus(projectedTranslation).getAngle();
    }

    public Rotation2d getAmpHeading() {
        return Configuration.isBlueAlliance() ? GeometryUtil.kRotationHalfPi : GeometryUtil.kRotationMinusHalfPi;
    }

    public Pose2d[] getDetectedNotePoses2d() {
        return mDetectedNotes.stream().map(note -> note.pose().toPose2d()).toArray(Pose2d[]::new);
    }

    @AutoLogOutput
    public Pose3d[] getDetectedNotePoses3d() {
        return mDetectedNotes.stream().map(note -> note.pose()).toArray(Pose3d[]::new);
    }

    public void addDetectedNotes(List<DetectedObjectState> notes) {
        for (var newNote : notes) {
            mDetectedNotes.removeIf(existingNote -> existingNote.isSame(newNote, kDuplicateNoteDistanceThreshold));
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
        var rotatedShooterHingePose = new Pose3d(
                shooterHingePose.getTranslation(),
                new Rotation3d(
                        shooterHingePose.getRotation().getX(),
                        shooterHingePose.getRotation().getY()
                                - mShooter.get().getAngle().getRadians(),
                        shooterHingePose.getRotation().getZ()));
        return rotatedShooterHingePose.transformBy(Constants.Robot.kShooterHingeToShooterExit);
    }

    @AutoLogOutput
    public Rotation2d calculateShooterAngleTowardsSpeaker() {
        return Rotation2d.fromRadians(Constants.Shooter.kShooterAngleInterpolator.get(getDistanceToSpeaker()));
    }

    public void setShootingState(ShootingState shootingState) {
        mShootingState = shootingState;
    }

    public ShootingState getShootingState() {
        return mShootingState;
    }

    public void setScoringMode(ScoringMode scoringMode) {
        this.mScoringMode = scoringMode;
    }

    public void setPath(Pose2d... poses) {
        mField.getObject("Path").setPoses(poses);
    }

    @AutoLogOutput
    public ScoringMode getScoringMode() {
        return mScoringMode;
    }

    public boolean isSpeakerMode() {
        return mScoringMode == ScoringMode.SPEAKER;
    }

    public boolean isAmpMode() {
        return mScoringMode == ScoringMode.AMP;
    }

    public boolean isClimbMode() {
        return mScoringMode == ScoringMode.CLIMB;
    }

    public enum ScoringMode {
        SPEAKER,
        AMP,
        CLIMB
    }
}
