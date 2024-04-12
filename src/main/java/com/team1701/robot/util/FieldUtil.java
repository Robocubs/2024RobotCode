package com.team1701.robot.util;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration;
import com.team1701.robot.FieldConstants;
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

    public static double getDistanceToPassTarget(Translation2d translation) {
        return translation.getDistance(
                Configuration.isBlueAlliance()
                        ? FieldConstants.kBluePassingTarget.getTranslation()
                        : FieldConstants.kRedPassingTarget.getTranslation());
    }

    public static Rotation2d getHeadingToPassTarget(Translation2d translation) {
        var target = Configuration.isBlueAlliance()
                ? FieldConstants.kBluePassingTarget.getTranslation()
                : FieldConstants.kRedPassingTarget.getTranslation();
        return target.minus(translation).getAngle();
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
}
