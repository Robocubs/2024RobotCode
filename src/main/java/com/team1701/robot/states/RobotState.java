package com.team1701.robot.states;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.team1701.lib.drivers.digitalinputs.DigitalIO;
import com.team1701.lib.drivers.digitalinputs.DigitalIOSensor;
import com.team1701.lib.drivers.digitalinputs.DigitalIOSim;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.estimation.PoseEstimator;
import com.team1701.lib.estimation.PoseEstimator.DriveMeasurement;
import com.team1701.lib.estimation.PoseEstimator.VisionMeasurement;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.robot.Configuration;
import com.team1701.robot.Configuration.Mode;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import com.team1701.robot.util.SparkMotorFactory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
    private static final double kDetectedNoteTimeout = 1.0;

    private final Indexer mIndexer;
    private final Intake mIntake;

    private final TimeLockedBoolean mHasNote = new TimeLockedBoolean(0.1, Timer.getFPGATimestamp(), true, false);

    private final PoseEstimator mPoseEstimator =
            new PoseEstimator(Constants.Drive.kKinematics, VecBuilder.fill(0.005, 0.005, 0.0005));
    private final List<NoteState> mDetectedNotes = new ArrayList<>();

    public RobotState() {

        Optional<Indexer> indexer = Optional.empty();
        Optional<Intake> intake = Optional.empty();

        if (Configuration.getMode() != Mode.REPLAY) {
            switch (Configuration.getRobot()) {
                case COMPETITION_BOT:
                    indexer = Optional.of(new Indexer(
                            SparkMotorFactory.createIndexerMotorIOSparkFlex(Constants.Indexer.kIndexerMotorId),
                            new DigitalIOSensor(Constants.Indexer.kIndexerEntranceSensorId),
                            new DigitalIOSensor(Constants.Indexer.kIndexerExitSensorId)));
                    intake = Optional.of(new Intake(
                            SparkMotorFactory.createIntakeMotorIOSparkFlex(Constants.Intake.kIntakeMotorId),
                            new DigitalIOSensor(Constants.Intake.kIntakeEntranceSensorId),
                            new DigitalIOSensor(Constants.Intake.kIntakeExitSensorId)));

                    break;
                case SIMULATION_BOT:
                    indexer = Optional.of(new Indexer(
                            new MotorIOSim(DCMotor.getNeoVortex(1), 1, 0.001, Constants.kLoopPeriodSeconds),
                            new DigitalIOSim(() -> false),
                            new DigitalIOSim(() -> false)));

                    intake = Optional.of(new Intake(
                            new MotorIOSim(DCMotor.getNeoVortex(1), 1, 0.001, Constants.kLoopPeriodSeconds),
                            new DigitalIOSim(() -> false),
                            new DigitalIOSim(() -> false)));

                    break;
                default:
                    break;
            }
        }

        this.mIndexer = indexer.orElseGet(() -> new Indexer(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));

        this.mIntake = intake.orElseGet(() -> new Intake(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {}));
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
        var hasNote = mIntake.hasNote() || mIndexer.hasNote();
        return mHasNote.update(hasNote, Timer.getFPGATimestamp());
    }
}
