package com.team1701.robot.simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.DoubleStream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.states.RobotState;
import com.team1701.robot.subsystems.indexer.Indexer;
import com.team1701.robot.subsystems.intake.Intake;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLogOutput;

public class NoteSimulator {
    private static double kIntakePathLength = Units.inchesToMeters(20.0);
    private static double kIndexerPathLength = Units.inchesToMeters(14.0);
    private static double kShooterPathLength = Units.inchesToMeters(14.0);
    private static double kNoteDiameter = Units.inchesToMeters(14.0);
    private static double kNoteRadius = kNoteDiameter / 2;
    private static double kNoteThickness = Units.inchesToMeters(2.0);
    private static double kIntakeRollerRadius = Units.inchesToMeters(1.0);
    private static double kIndexerRollerRadius = Units.inchesToMeters(1.0);
    private static double kShooterRollerRadius = Units.inchesToMeters(1.0);
    private static Pose3d kBlueNoteSpawn = new Pose3d();
    private static Pose3d kRedNoteSpawn = new Pose3d();

    private final RobotState mRobotState;
    private final Intake mIntake;
    private final Indexer mIndexer;
    private final Shooter mShooter;

    private final List<NoteOnField> mNotesOnField = new ArrayList<>();
    private final List<NoteInRobot> mNotesInRobot = new ArrayList<>();

    // TODO: Implement boolean suppliers for sensors
    public NoteSimulator(RobotState robotState, Intake intake, Indexer indexer, Shooter shooter) {
        mRobotState = robotState;
        mIntake = intake;
        mIndexer = indexer;
        mShooter = shooter;

        // TODO: Place starting notes on the field

        mNotesInRobot.add(new NoteInRobot(kIndexerPathLength / 2, NoteLocation.INDEXER));
    }

    public void periodic() {
        // Process notes on field
        var notesToRemoveFromField = new ArrayList<NoteOnField>();
        for (var note : mNotesOnField) {
            // Apply note movement
            if (note.velocity.getNorm() > 0) {
                // TODO: Apply physics
                var movementThisTick = note.velocity.times(Constants.kLoopPeriodSeconds);
                note.pose = new Pose3d(note.pose.getTranslation().plus(movementThisTick), note.pose.getRotation());
            }

            // Check if note is in position to intake
            if (mIntake.getVelocityRadiansPerSecond() > 0.1 && note.isTouchingIntake()) {
                mNotesInRobot.add(new NoteInRobot());
                notesToRemoveFromField.remove(note);
            }
        }

        mNotesOnField.removeAll(notesToRemoveFromField);

        mNotesOnField.removeIf(note -> !Util.inRange(note.pose.getX(), 0, FieldConstants.kFieldLongLengthMeters)
                && !Util.inRange(note.pose.getY(), 0, FieldConstants.kFieldShortLengthMeters));

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
                    var averageRollerSpeed = DoubleStream.of(mShooter.getRollerSpeedsRadiansPerSecond())
                            .average()
                            .orElse(0);
                    if (MathUtil.isNear(0, averageRollerSpeed, 0.1)) {
                        continue;
                    }

                    note.position += averageRollerSpeed * kShooterRollerRadius * Constants.kLoopPeriodSeconds;

                    if (averageRollerSpeed > 0 && note.position > kShooterPathLength) {
                        var shooterExitPose = mRobotState.getShooterExitPose();
                        var notePose = shooterExitPose.transformBy(
                                new Transform3d(kNoteRadius, 0, 0, shooterExitPose.getRotation()));
                        var noteVelocity = new Translation3d(averageRollerSpeed * kShooterRollerRadius, 0, 0);
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

        // TODO: Randomly place additional notes in valid places on the field
        // Place new notes
        if (mNotesInRobot.isEmpty()) {
            if (mNotesOnField.stream().noneMatch(note -> note.isNear(kBlueNoteSpawn))) {
                mNotesOnField.add(new NoteOnField(kBlueNoteSpawn));
            }

            if (mNotesOnField.stream().noneMatch(note -> note.isNear(kRedNoteSpawn))) {
                mNotesOnField.add(new NoteOnField(kRedNoteSpawn));
            }
        }
    }

    @AutoLogOutput
    public Pose3d[] getNotePoses() {
        // TODO: Add notes in robot
        return mNotesOnField.stream().map(note -> note.pose).toArray(Pose3d[]::new);
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
                            -Constants.Robot.kRobotBackToCenter,
                            -Constants.Robot.kRobotBackToCenter - kNoteRadius)
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
    }

    private enum NoteLocation {
        INTAKE,
        INDEXER,
        SHOOTER,
    }
}
