package com.team1701.robot.util;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration;
import com.team1701.robot.FieldConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldUtil {
    public static double getDistanceToSpeaker(Translation3d translation) {
        return translation.getDistance(
                Configuration.isBlueAlliance()
                        ? FieldConstants.kBlueSpeakerOpeningCenter
                        : FieldConstants.kRedSpeakerOpeningCenter);
    }

    public static double getDistanceToSpeaker(Translation2d translation) {
        return getDistanceToSpeaker(GeometryUtil.toTranslation3d(translation));
    }

    public static double getDistanceToSpeaker(Pose3d pose) {
        return getDistanceToSpeaker(pose.getTranslation());
    }

    public static double getDistanceToSpeaker(Pose2d pose) {
        return getDistanceToSpeaker(pose.getTranslation());
    }

    public static double getDistanceToLongPassTarget(Translation2d translation) {
        return translation.getDistance(
                Configuration.isBlueAlliance()
                        ? FieldConstants.kBlueLongPassTarget.getTranslation()
                        : FieldConstants.kRedLongPassTarget.getTranslation());
    }

    public static double getDistanceToMidPassTarget(Translation2d translation) {
        return translation.getDistance(FieldConstants.kMidPassTarget.getTranslation()) - 1;
    }

    public static Rotation2d getHeadingToLongPassTarget(Translation2d translation) {
        var target = Configuration.isBlueAlliance()
                ? FieldConstants.kBlueLongPassTarget.getTranslation()
                : FieldConstants.kRedLongPassTarget.getTranslation();
        return target.minus(translation).getAngle();
    }

    public static Rotation2d getHeadingToMidPassTarget(Translation2d translation) {
        return FieldConstants.kMidPassTarget.getTranslation().minus(translation).getAngle();
    }

    public static Rotation2d getHeadingToSpeaker(Translation2d translation) {
        var speakerTranslation = Configuration.isBlueAlliance()
                ? FieldConstants.kBlueSpeakerOpeningCenter
                : FieldConstants.kRedSpeakerOpeningCenter;
        return speakerTranslation.toTranslation2d().minus(translation).getAngle();
    }

    public static Rotation2d getHeadingToSpeaker(Pose2d pose) {
        return getHeadingToSpeaker(pose.getTranslation());
    }

    public static Rotation2d getSpeakerHeadingTolerance(Translation2d translation) {
        var tolerancePose = Configuration.isBlueAlliance()
                ? new Pose3d(FieldConstants.kBlueSpeakerToleranceTranslation, GeometryUtil.kRotation3dIdentity)
                : new Pose3d(FieldConstants.kRedSpeakerToleranceTranslation, GeometryUtil.kRotation3dIdentity);
        var heading = tolerancePose
                .getTranslation()
                .toTranslation2d()
                .minus(translation)
                .getAngle();
        var toleranceRadians = Math.abs(MathUtil.angleModulus(
                heading.getRadians() - getHeadingToSpeaker(translation).getRadians()));
        return Rotation2d.fromRadians(Math.max(0.017, toleranceRadians / 2));
    }
}
