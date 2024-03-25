package com.team1701.robot.simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import com.team1701.lib.drivers.cameras.config.VisionConfig;
import com.team1701.lib.drivers.cameras.neural.DetectedObject;
import com.team1701.lib.drivers.digitalinputs.DigitalIO;
import com.team1701.lib.drivers.digitalinputs.DigitalIOSim;
import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.autonomous.AutoNote;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class NoteSimulator extends SubsystemBase {
    private static final double kIntakePathLength = Units.inchesToMeters(20.0);
    private static final double kIndexerPathLength = Units.inchesToMeters(13.0);
    private static final double kShooterPathLength = Units.inchesToMeters(13.0);
    private static final double kNoteDiameter = Units.inchesToMeters(14.0);
    private static final double kNoteRadius = kNoteDiameter / 2;
    private static final double kNoteThickness = Units.inchesToMeters(2.0);
    private static final double kIntakeRollerRadius = Units.inchesToMeters(1.0);
    private static final double kIndexerRollerRadius = Units.inchesToMeters(1.0);
    private static final double kShooterRollerRadius = Units.inchesToMeters(1.0);
    private static final Pose3d kBlueNoteSpawn =
            new Pose3d(1.0, 1.0, kNoteThickness / 2, GeometryUtil.kRotation3dIdentity);
    private static final Pose3d kRedNoteSpawn = new Pose3d(
            FieldConstants.kFieldLongLengthMeters - 1.0, 1.0, kNoteThickness / 2, GeometryUtil.kRotation3dIdentity);
    private static final Pose3d[] kStartingNotePoses = Stream.of(
                    AutoNote.A.bluePose(),
                    AutoNote.B.bluePose(),
                    AutoNote.C.bluePose(),
                    AutoNote.A.redPose(),
                    AutoNote.B.redPose(),
                    AutoNote.C.redPose(),
                    AutoNote.M1.pose(),
                    AutoNote.M2.pose(),
                    AutoNote.M3.pose(),
                    AutoNote.M4.pose(),
                    AutoNote.M5.pose())
            .map(pose -> new Pose3d(pose.getX(), pose.getY(), kNoteThickness / 2, GeometryUtil.kRotation3dIdentity))
            .toArray(Pose3d[]::new);

    private final RobotState mRobotState;
    private final VisionConfig mDetectorVisionConfig;
    private Intake mIntake = new Intake(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {});
    private Indexer mIndexer = new Indexer(new MotorIO() {}, new DigitalIO() {}, new DigitalIO() {});
    private Shooter mShooter = new Shooter(new MotorIO() {}, new MotorIO() {}, new MotorIO() {}, new EncoderIO() {});

    private final List<NoteOnField> mNotesOnField = new ArrayList<>();
    private final List<NoteInRobot> mNotesInRobot = new ArrayList<>();

    public record NoteSimulatorSensors(
            DigitalIO intakeEntranceSensor,
            DigitalIO intakeExitSensor,
            DigitalIO indexerEntranceSensor,
            DigitalIO indexerExitSensor) {}

    public NoteSimulator(RobotState robotState, VisionConfig detectorVisionConfig) {
        mRobotState = robotState;
        mDetectorVisionConfig = detectorVisionConfig;

        mNotesInRobot.add(new NoteInRobot(kIndexerPathLength / 2, NoteLocation.INTAKE));
        new Trigger(DriverStation::isAutonomous).onTrue(Commands.runOnce(this::placeAutonNotes));
    }

    public NoteSimulatorSensors getSensors() {
        return new NoteSimulatorSensors(
                new DigitalIOSim(() -> mNotesInRobot.stream()
                        .anyMatch(note -> note.location == NoteLocation.INTAKE && note.position < kNoteDiameter)),
                new DigitalIOSim(() -> mNotesInRobot.stream()
                        .anyMatch(note ->
                                note.location == NoteLocation.INTAKE && note.position > kIntakePathLength - kNoteRadius
                                        || note.location == NoteLocation.INDEXER && note.position < kNoteRadius)),
                new DigitalIOSim(() -> mNotesInRobot.stream().anyMatch(note -> note.location == NoteLocation.INDEXER)),
                new DigitalIOSim(() -> mNotesInRobot.stream()
                        .anyMatch(note -> note.location == NoteLocation.INDEXER
                                        && note.position > kIndexerPathLength - kNoteDiameter * 0.75
                                || note.location == NoteLocation.SHOOTER)));
    }

    public void bindSubsystems(Intake intake, Indexer indexer, Shooter shooter) {
        mIntake = intake;
        mIndexer = indexer;
        mShooter = shooter;
    }

    @Override
    public void simulationPeriodic() {
        // Process notes on field
        var notesToRemoveFromField = new ArrayList<NoteOnField>();
        for (var note : mNotesOnField) {
            // Apply note movement
            if (note.velocity.getNorm() > 0) {
                var movementThisTick = note.velocity.times(Constants.kLoopPeriodSeconds);
                note.pose = new Pose3d(note.pose.getTranslation().plus(movementThisTick), note.pose.getRotation());
            }

            // Check if note is in position to intake
            if (mIntake.getVelocityRadiansPerSecond() > 0.1 && note.isTouchingIntake()) {
                mNotesInRobot.add(new NoteInRobot());
                notesToRemoveFromField.add(note);
            }
        }

        mNotesOnField.removeAll(notesToRemoveFromField);

        mNotesOnField.removeIf(note -> !Util.inRange(note.pose.getX(), 0, FieldConstants.kFieldLongLengthMeters)
                || !Util.inRange(note.pose.getY(), 0, FieldConstants.kFieldShortLengthMeters));

        // Process notes in robot
        var notesToRemoveFromRobot = new ArrayList<NoteInRobot>();
        for (var note : mNotesInRobot) {
            switch (note.location) {
                case INTAKE:
                    if (MathUtil.isNear(0, mIndexer.getVelocityRadiansPerSecond(), 0.1)) {
                        continue;
                    }

                    note.position +=
                            mIntake.getVelocityRadiansPerSecond() * kIntakeRollerRadius * Constants.kLoopPeriodSeconds;

                    if (mIntake.getVelocityRadiansPerSecond() > 0 && note.position > kIntakePathLength) {
                        note.location = NoteLocation.INDEXER;
                        note.position = 0.0;
                    } else if (mIntake.getVelocityRadiansPerSecond() < 0 && note.position < 0) {
                        var noteToRobot = new Transform2d(
                                -Constants.Robot.kRobotBackToCenter - kNoteRadius, 0, GeometryUtil.kRotationIdentity);
                        mNotesOnField.add(new NoteOnField(
                                new Pose3d(mRobotState.getPose2d().transformBy(noteToRobot))));
                        notesToRemoveFromRobot.add(note);
                    }

                    break;
                case INDEXER:
                    if (MathUtil.isNear(0, mIndexer.getVelocityRadiansPerSecond(), 0.1)) {
                        continue;
                    }

                    note.position += mIndexer.getVelocityRadiansPerSecond()
                            * kIndexerRollerRadius
                            * Constants.kLoopPeriodSeconds;

                    if (mIndexer.getVelocityRadiansPerSecond() > 0 && note.position > kIndexerPathLength) {
                        note.location = NoteLocation.SHOOTER;
                        note.position = 0.0;
                    } else if (mIndexer.getVelocityRadiansPerSecond() < 0 && note.position < 0) {
                        note.location = NoteLocation.INTAKE;
                        note.position = kIntakePathLength;
                    }

                    break;
                case SHOOTER:
                    var averageRollerSpeed = DoubleStream.of(
                                    mShooter.getRollerSpeedsRadiansPerSecond().toArray())
                            .average()
                            .orElse(0);
                    if (MathUtil.isNear(0, averageRollerSpeed, 0.1)) {
                        continue;
                    }

                    note.position += averageRollerSpeed * kShooterRollerRadius * Constants.kLoopPeriodSeconds;

                    if (averageRollerSpeed > 0 && note.position > kShooterPathLength) {
                        var shooterExitPose = mRobotState.getShooterExitPose();
                        var notePose = shooterExitPose.transformBy(
                                new Transform3d(kNoteRadius, 0, 0, GeometryUtil.kRotation3dIdentity));
                        var noteVelocity = new Translation3d(averageRollerSpeed * kShooterRollerRadius, 0, 0)
                                .rotateBy(notePose.getRotation());
                        mNotesOnField.add(new NoteOnField(notePose, noteVelocity));
                        notesToRemoveFromRobot.add(note);
                    } else if (averageRollerSpeed < 0 && note.position < 0) {
                        note.location = NoteLocation.INDEXER;
                        note.position = kIndexerPathLength;
                    }

                    break;
            }
        }

        mNotesInRobot.removeAll(notesToRemoveFromRobot);

        // Place new notes
        if (mNotesInRobot.isEmpty() && DriverStation.isTeleop()) {
            if (mNotesOnField.stream().noneMatch(note -> note.isNear(kBlueNoteSpawn))) {
                mNotesOnField.add(new NoteOnField(kBlueNoteSpawn));
            }

            if (mNotesOnField.stream().noneMatch(note -> note.isNear(kRedNoteSpawn))) {
                mNotesOnField.add(new NoteOnField(kRedNoteSpawn));
            }
        }

        Logger.recordOutput("NoteSimulator/NotePoses", getNotePoses());
        Logger.recordOutput(
                "NoteSimulator/NotesInIntake",
                mNotesInRobot.stream()
                        .filter(note -> note.location == NoteLocation.INTAKE)
                        .count());
        Logger.recordOutput(
                "NoteSimulator/NotesInIndexer",
                mNotesInRobot.stream()
                        .filter(note -> note.location == NoteLocation.INDEXER)
                        .count());
        Logger.recordOutput(
                "NoteSimulator/NotesInShooter",
                mNotesInRobot.stream()
                        .filter(note -> note.location == NoteLocation.SHOOTER)
                        .count());
        Logger.recordOutput(
                "NoteSimulator/NoteInRobotPositions",
                mNotesInRobot.stream()
                        .mapToDouble(NoteInRobot::getTotalPosition)
                        .toArray());
    }

    public void placeAutonNotes() {
        mNotesOnField.clear();
        for (var note : kStartingNotePoses) {
            mNotesOnField.add(new NoteOnField(note));
        }

        mNotesInRobot.clear();
        mNotesInRobot.add(new NoteInRobot(kIntakePathLength - Units.inchesToMeters(1), NoteLocation.INDEXER));
    }

    public Pose3d[] getNotePosesOnField() {
        return mNotesOnField.stream().map(note -> note.pose).toArray(Pose3d[]::new);
    }

    public DetectedObject[] getDetectedObjects() {
        var cameraPose = mRobotState.getPose3d().transformBy(mDetectorVisionConfig.robotToCamera);
        return mNotesOnField.stream()
                .filter(note -> note.pose.getZ() < kNoteThickness)
                .map(note -> note.pose.relativeTo(cameraPose).getTranslation())
                .filter(translation -> translation.getX() > 0)
                .map(translation -> {
                    var pitch = new Rotation2d(translation.getX(), translation.getZ());
                    var yaw = new Rotation2d(translation.getX(), -translation.getY());
                    return new DetectedObject("Note", 0, 1, 1, pitch, yaw, new Translation2d());
                })
                .filter(detectedObject -> Util.inRange(detectedObject.yaw.getDegrees(), 31.15)
                        && Util.inRange(detectedObject.pitch.getDegrees(), 24.85))
                .toArray(DetectedObject[]::new);
    }

    public Pose3d[] getNotePoses() {
        return Stream.concat(
                        mNotesOnField.stream().map(note -> note.pose),
                        mNotesInRobot.stream()
                                .map(note -> mRobotState.getPose3d().transformBy(note.getRobotToNote())))
                .toArray(Pose3d[]::new);
    }

    private class NoteOnField {
        private Pose3d pose;
        private Translation3d velocity;

        public NoteOnField(Pose3d pose) {
            this.pose = pose;
            this.velocity = GeometryUtil.kTranslation3dIdentity;
        }

        public NoteOnField(Pose3d pose, Translation3d velocity) {
            this.pose = pose;
            this.velocity = velocity;
        }

        public boolean isNear(Pose3d pose) {
            return pose.getTranslation().getDistance(this.pose.getTranslation()) < kNoteDiameter;
        }

        public boolean isTouchingIntake() {
            var noteRelativeToRobot = pose.toPose2d().relativeTo(mRobotState.getPose2d());
            return pose.getZ() < kNoteThickness
                    && Util.inRange(
                            noteRelativeToRobot.getX(),
                            -Constants.Robot.kRobotBackToCenter - kNoteRadius,
                            -Constants.Robot.kRobotBackToCenter)
                    && Util.inRange(
                            noteRelativeToRobot.getY(),
                            -Constants.Robot.kRobotSideToCenter,
                            Constants.Robot.kRobotSideToCenter);
        }
    }

    private class NoteInRobot {
        private double position;
        private NoteLocation location;

        public NoteInRobot() {
            this.position = 0.0;
            this.location = NoteLocation.INTAKE;
        }

        public NoteInRobot(double position, NoteLocation location) {
            this.position = position;
            this.location = location;
        }

        public Transform3d getRobotToNote() {
            return new Transform3d(0, 0, 0.5, GeometryUtil.kRotation3dIdentity);
        }

        public double getTotalPosition() {
            var position = this.position;
            if (location == NoteLocation.INTAKE) {
                return position;
            }

            position += kIntakePathLength;
            if (location == NoteLocation.INDEXER) {
                return position;
            }

            position += kIndexerPathLength;
            return position;
        }
    }

    private enum NoteLocation {
        INTAKE,
        INDEXER,
        SHOOTER,
    }
}
