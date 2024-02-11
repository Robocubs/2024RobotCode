package com.team1701.robot.states;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;

public class NoteState {
    private static final double kNoteDeduplicationThreshold = Units.inchesToMeters(10.0);

    public final double lastDetectedTimestamp;
    public final Pose3d pose;

    public NoteState(double lastDetectedTimestamp, Pose3d pose) {
        this.lastDetectedTimestamp = lastDetectedTimestamp;
        this.pose = pose;
    }

    public boolean isSame(NoteState other) {
        return pose.getTranslation().getDistance(other.pose.getTranslation()) < kNoteDeduplicationThreshold;
    }
}
