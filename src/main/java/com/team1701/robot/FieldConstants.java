package com.team1701.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    public static double kFieldLongLengthMeters = (Units.inchesToMeters((54 * 12) + 3.25));
    public static double kFieldShortLengthMeters = (Units.inchesToMeters((26 * 12) + 11.25));
    public static double kCenterLine = kFieldLongLengthMeters / 2.0;
    public static double kWingLength = Units.inchesToMeters(231.2);

    public static Translation3d kBlueSpeakerOpeningCenter =
            new Translation3d(Units.inchesToMeters(0.9), Units.inchesToMeters(218.42), 204.5);
    public static Translation3d kRedSpeakerOpeningCenter = new Translation3d(
            kFieldLongLengthMeters - kBlueSpeakerOpeningCenter.getX(),
            kBlueSpeakerOpeningCenter.getY(),
            kBlueSpeakerOpeningCenter.getZ());
}
