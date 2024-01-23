package com.team1701.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    // blue locations use blue field origin, red locations use red origin
    // all units are in meters and radians
    public static double kFieldLongLengthMeters = (Units.inchesToMeters((54 * 12) + 3.25));
    public static double kFieldShortLengthMeters = (Units.inchesToMeters((26 * 12) + 11.25));

    public static Translation3d kBlueSpeakerOpeningCenter = new Translation3d(0, Units.inchesToMeters(218.42), 204.5);
    public static Translation3d kRedSpeakerOpeningCenter = new Translation3d(0, Units.inchesToMeters(651.23), 204.5);
}
