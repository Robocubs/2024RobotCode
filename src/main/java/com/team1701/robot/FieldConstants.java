package com.team1701.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    public static double kFieldLongLengthMeters = (Units.inchesToMeters((54 * 12) + 3.25));
    public static double kFieldShortLengthMeters = (Units.inchesToMeters((26 * 12) + 11.25));
    public static double kCenterLine = kFieldLongLengthMeters / 2.0;
    public static double kWingLength = Units.inchesToMeters(231.2);
    public static double kNoteHeight = Units.inchesToMeters(2);

    public static double kSpeakerHeight = Units.inchesToMeters(80.4375);
    public static Translation3d kBlueSpeakerOpeningCenter =
            new Translation3d(Units.inchesToMeters(9), Units.inchesToMeters(218.42), kSpeakerHeight);
    public static Translation3d kRedSpeakerOpeningCenter = new Translation3d(
            kFieldLongLengthMeters - kBlueSpeakerOpeningCenter.getX(),
            kBlueSpeakerOpeningCenter.getY(),
            kSpeakerHeight);

    public static Translation3d kBlueSource =
            new Translation3d(Units.inchesToMeters(30), Units.inchesToMeters(30), 0.0);
    public static Translation3d kRedSource =
            new Translation3d(kFieldLongLengthMeters - kBlueSource.getX(), kBlueSource.getY(), kBlueSource.getZ());

    public static Translation3d kBlueAmpPosition =
            new Translation3d(Units.inchesToMeters(72.5), Units.inchesToMeters(323), Units.inchesToMeters(53.38));
    public static Translation3d kRedAmpPosition =
            new Translation3d(Units.inchesToMeters(578.77), Units.inchesToMeters(323), Units.inchesToMeters(53.38));
}
