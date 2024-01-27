package com.team1701.robot.states;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class NoteState {
    private static final double kNoteDeduplicationThreshold = Units.inchesToMeters(10.0);

    public final double lastDetectedTimestamp;
    public final Translation2d pose;

    public NoteState(int detectorId, double lastDetectedTimestamp, Translation2d pose) {
        this.lastDetectedTimestamp = lastDetectedTimestamp;
        this.pose = pose;
    }

    public boolean isSame(NoteState other) {
        return pose.getDistance(other.pose) < kNoteDeduplicationThreshold;
    }
}
