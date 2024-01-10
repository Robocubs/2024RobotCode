package com.team1701.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;

public final class GeometryUtil {
    public static final Pose2d kPoseIdentity = new Pose2d();
    public static final Rotation2d kRotationIdentity = new Rotation2d();
    public static final Rotation2d kRotationPi = new Rotation2d(Math.PI);
    public static final Rotation2d kRotationMinusPi = new Rotation2d(-Math.PI);
    public static final Rotation2d kRotationHalfPi = new Rotation2d(Math.PI / 2.0);
    public static final Rotation2d kRotationMinusHalfPi = new Rotation2d(-Math.PI / 2.0);
    public static final Twist2d kTwistIdentity = new Twist2d();
    public static final Transform3d kTransform3dIdentity = new Transform3d();
    public static final Translation3d kTranslation3dIdentity = new Translation3d();
    public static final Rotation3d kRotation3dIdentity = new Rotation3d();

    public static Rotation2d flipX(Rotation2d rotation) {
        return rotation.getRadians() < 0 ? kRotationMinusPi.minus(rotation) : kRotationPi.minus(rotation);
    }

    public static Rotation2d angleModulus(Rotation2d rotation) {
        return Rotation2d.fromRadians(MathUtil.angleModulus(rotation.getRadians()));
    }

    public static boolean isNear(Rotation2d expected, Rotation2d actual, Rotation2d tolerance) {
        return MathUtil.isNear(
                MathUtil.angleModulus(expected.getRadians()),
                MathUtil.angleModulus(actual.getRadians()),
                tolerance.getRadians());
    }
}
