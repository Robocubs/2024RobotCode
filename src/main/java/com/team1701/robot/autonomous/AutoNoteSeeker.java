package com.team1701.robot.autonomous;

import java.util.Optional;
import java.util.stream.Stream;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration;
import com.team1701.robot.Constants;
import com.team1701.robot.FieldConstants;
import com.team1701.robot.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;

public class AutoNoteSeeker {
    private static final double kSeekNoteTolerance = FieldConstants.kSpaceBetweenWingNotes * 0.67;

    private final RobotState mRobotState;

    private Optional<AutoNote> mNote = Optional.empty();
    private Optional<Pose2d> mLastDetectedNote = Optional.empty();

    public AutoNoteSeeker(RobotState robotState) {
        mRobotState = robotState;
    }

    public void clear() {
        mNote = Optional.empty();
        mLastDetectedNote = Optional.empty();
        Logger.recordOutput("Autonomous/NoteSeeker/NotePose", GeometryUtil.kPoseIdentity);
        Logger.recordOutput("Autonomous/NoteSeeker/LastDetectedNotePose", GeometryUtil.kPoseIdentity);
    }

    public void setNote(AutoNote note) {
        mNote = Optional.of(note);
        mLastDetectedNote = Optional.empty();
        Logger.recordOutput("Autonomous/NoteSeeker/NotePose", note.pose());
    }

    public Optional<Pose2d> getDetectedNoteToSeek() {
        // Find the detected note that is closest to the note to seek and within tolerance
        var newDetectedNote = mNote.flatMap(note -> Stream.of(mRobotState.getDetectedNotePoses2d())
                // Filter out notes across the center
                .filter(detectedNote -> (Configuration.isBlueAlliance()
                                && detectedNote.getX()
                                        < FieldConstants.kCenterLine + Constants.Robot.kRobotFrontToCenter)
                        || (Configuration.isRedAlliance()
                                && detectedNote.getX()
                                        > FieldConstants.kCenterLine - Constants.Robot.kRobotFrontToCenter))
                // Filter out notes that are not near the note to seek
                .filter(detectedPose -> GeometryUtil.isNear(
                        note.pose().getTranslation(), detectedPose.getTranslation(), kSeekNoteTolerance))
                // Prioritize note closest to last detected note, then closest to the intended note.
                .sorted((detectedPose1, detectedPose2) -> {
                    var noteTranslation = mLastDetectedNote.orElse(note.pose()).getTranslation();
                    return Double.compare(
                            noteTranslation.getDistance(detectedPose1.getTranslation()),
                            noteTranslation.getDistance(detectedPose2.getTranslation()));
                })
                .findFirst());

        if (newDetectedNote.isEmpty()) {
            return mLastDetectedNote;
        }

        mLastDetectedNote = newDetectedNote;
        Logger.recordOutput("Autonomous/NoteSeeker/LastDetectedNotePose", newDetectedNote.get());

        return newDetectedNote;
    }
}
