package com.team1701.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;

public final class GeometryUtil {
    public static final Pose2d kPoseIdentity = new Pose2d();
    public static final Translation2d kTranslationIdentity = new Translation2d();
    public static final Rotation2d kRotationIdentity = new Rotation2d();
    public static final Rotation2d kRotationPi = new Rotation2d(Math.PI);
    public static final Rotation2d kRotationMinusPi = new Rotation2d(-Math.PI);
    public static final Rotation2d kRotationHalfPi = new Rotation2d(Math.PI / 2.0);
    public static final Rotation2d kRotationMinusHalfPi = new Rotation2d(-Math.PI / 2.0);
    public static final Rotation2d kRotationThreeHalfPi = new Rotation2d(3.0 * Math.PI / 2.0);
    public static final Twist2d kTwistIdentity = new Twist2d();
    public static final Transform3d kTransform3dIdentity = new Transform3d();
    public static final Translation3d kTranslation3dIdentity = new Translation3d();
    public static final Rotation3d kRotation3dIdentity = new Rotation3d();

    public static Rotation2d flipX(Rotation2d rotation) {
        return kRotationPi.minus(rotation);
    }

    public static Pose2d flipX(Pose2d pose, double fieldLength) {
        return new Pose2d(fieldLength - pose.getX(), pose.getY(), flipX(pose.getRotation()));
    }

    public static Pose2d[] flipX(Pose2d[] poses, double fieldLength) {
        var flippedPoses = new Pose2d[poses.length];
        for (var i = 0; i < poses.length; i++) {
            flippedPoses[i] = flipX(poses[i], fieldLength);
        }
        return flippedPoses;
    }

    public static Rotation2d clampRotation(
            Rotation2d rotation, Rotation2d minimumRotation, Rotation2d maximumRotation) {
        return Rotation2d.fromRadians(
                MathUtil.clamp(rotation.getRadians(), minimumRotation.getRadians(), maximumRotation.getRadians()));
    }

    public static Rotation2d angleModulus(Rotation2d rotation) {
        return Rotation2d.fromRadians(MathUtil.angleModulus(rotation.getRadians()));
    }

    public static Rotation2d angleModulus(Rotation2d rotation, Rotation2d minimumInput, Rotation2d maximumInput) {
        return Rotation2d.fromRadians(
                MathUtil.inputModulus(rotation.getRadians(), minimumInput.getRadians(), maximumInput.getRadians()));
    }

    public static boolean isNear(Rotation2d expected, Rotation2d actual, Rotation2d tolerance) {
        var difference = MathUtil.angleModulus(expected.minus(actual).getRadians());
        return MathUtil.isNear(difference, 0.0, tolerance.getRadians());
    }

    public static boolean isNear(Translation2d expected, Translation2d actual, double tolerance) {
        var difference = expected.minus(actual).getNorm();
        return MathUtil.isNear(difference, 0.0, tolerance);
    }

    public static boolean isNear(
            Pose2d expected, Pose2d actual, double translationTolerance, Rotation2d rotationTolerance) {
        return isNear(expected.getTranslation(), actual.getTranslation(), translationTolerance)
                && isNear(expected.getRotation(), actual.getRotation(), rotationTolerance);
    }

    public static Twist2d multiply(Twist2d twist, double scalar) {
        return new Twist2d(twist.dx * scalar, twist.dy * scalar, twist.dtheta * scalar);
    }

    public static Translation2d getTranslation2d(Pose3d pose) {
        return new Translation2d(pose.getX(), pose.getY());
    }

    public static Pose3d toPose3d(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    public static Rotation3d toRotation3d(Rotation2d rotation) {
        return new Rotation3d(0, 0, rotation.getRadians());
    }

    public static Translation3d toTranslation3d(Translation2d translation) {
        return new Translation3d(translation.getX(), translation.getY(), 0);
    }
}
