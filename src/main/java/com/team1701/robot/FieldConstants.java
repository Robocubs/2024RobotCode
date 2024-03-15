package com.team1701.robot;

import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    public static final double kFieldLongLengthMeters = (Units.inchesToMeters((54 * 12) + 3.25));
    public static final double kFieldShortLengthMeters = (Units.inchesToMeters((26 * 12) + 11.25));
    public static final double kCenterLine = kFieldLongLengthMeters / 2.0;
    public static final double kShortLengthMidLine = kFieldShortLengthMeters / 2.0;
    public static final double kWingLength = Units.inchesToMeters(231.2);
    public static final double kNoteHeight = Units.inchesToMeters(2);
    public static final double kHalfNoteHeight = kNoteHeight / 2;

    public static final double kSpeakerHeight = Units.inchesToMeters(80.4375);
    public static final Translation3d kBlueSpeakerOpeningCenter =
            new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(218.42), kSpeakerHeight);
    public static final Translation3d kRedSpeakerOpeningCenter = new Translation3d(
            kFieldLongLengthMeters - kBlueSpeakerOpeningCenter.getX(),
            kBlueSpeakerOpeningCenter.getY(),
            kSpeakerHeight);

    public static final Translation3d kBlueSpeakerToleranceTranslation =
            new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(230.42), kSpeakerHeight);
    public static final Translation3d kRedSpeakerToleranceTranslation = new Translation3d(
            kFieldLongLengthMeters - kBlueSpeakerToleranceTranslation.getX(),
            kBlueSpeakerToleranceTranslation.getY(),
            kSpeakerHeight);

    public static final Translation3d kBlueSource =
            new Translation3d(Units.inchesToMeters(30), Units.inchesToMeters(30), 0.0);
    public static Translation3d kRedSource =
            new Translation3d(kFieldLongLengthMeters - kBlueSource.getX(), kBlueSource.getY(), kBlueSource.getZ());

    public static final Translation3d kBlueAmpPosition = new Translation3d(1.8, 7.89, Units.inchesToMeters(53.38));
    public static Translation3d kRedAmpPosition =
            new Translation3d(Units.inchesToMeters(578.77), 7.89, Units.inchesToMeters(53.38));

    public static final Pose2d kBlueAmpDrivePose = new Pose2d(
            Units.inchesToMeters(72.5),
            Units.inchesToMeters(323.0) - Constants.Robot.kRobotFrontToCenterWithBumpers,
            Rotation2d.fromDegrees(90));

    public static final Pose2d kRedAmpDrivePose = new Pose2d(
            Units.inchesToMeters(578.77),
            Units.inchesToMeters(323.0) - Constants.Robot.kRobotFrontToCenterWithBumpers,
            Rotation2d.fromDegrees(90));

    public static final Pose2d kBluePassingTarget =
            new Pose2d(new Translation2d(1.5, 6.5), GeometryUtil.kRotationIdentity);
    public static final Pose2d kRedPassingTarget =
            new Pose2d(new Translation2d(kFieldLongLengthMeters - 1.5, 6.5), GeometryUtil.kRotationIdentity);
}
