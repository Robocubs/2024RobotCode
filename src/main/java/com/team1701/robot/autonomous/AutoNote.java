package com.team1701.robot.autonomous;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration;
import com.team1701.robot.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;

public class AutoNote {
    public static final AutoNote A = new AutoNote(
            FieldConstants.kNoteStartLineBlueWing,
            FieldConstants.kShortLengthMidLine + FieldConstants.kSpaceBetweenWingNotes * 2);
    public static final AutoNote B = new AutoNote(
            FieldConstants.kNoteStartLineBlueWing,
            FieldConstants.kShortLengthMidLine + FieldConstants.kSpaceBetweenWingNotes);
    public static final AutoNote C =
            new AutoNote(FieldConstants.kNoteStartLineBlueWing, FieldConstants.kShortLengthMidLine);

    public static final AutoNote M1 = new AutoNote(
            FieldConstants.kCenterLine,
            FieldConstants.kShortLengthMidLine + FieldConstants.kSpaceBetweenCenterNotes * 2);
    public static final AutoNote M2 = new AutoNote(
            FieldConstants.kCenterLine, FieldConstants.kShortLengthMidLine + FieldConstants.kSpaceBetweenCenterNotes);
    public static final AutoNote M3 = new AutoNote(FieldConstants.kCenterLine, FieldConstants.kShortLengthMidLine);
    public static final AutoNote M4 = new AutoNote(
            FieldConstants.kCenterLine, FieldConstants.kShortLengthMidLine - FieldConstants.kSpaceBetweenCenterNotes);
    public static final AutoNote M5 = new AutoNote(
            FieldConstants.kCenterLine,
            FieldConstants.kShortLengthMidLine - FieldConstants.kSpaceBetweenCenterNotes * 2);

    private final Pose2d mBluePose;
    private final Pose2d mRedPose;

    private AutoNote(double x, double y) {
        mBluePose = new Pose2d(x, y, GeometryUtil.kRotationIdentity);
        mRedPose = GeometryUtil.flipX(mBluePose, FieldConstants.kFieldLongLengthMeters);
    }

    public Pose2d pose() {
        return Configuration.isRedAlliance() ? mRedPose : mBluePose;
    }

    public Pose2d bluePose() {
        return mBluePose;
    }

    public Pose2d redPose() {
        return mRedPose;
    }
}
