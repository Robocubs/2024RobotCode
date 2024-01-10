package com.team1701.lib.util;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class KinematicsUtil {
    public static Twist2d toTwist2d(ChassisSpeeds s) {
        return new Twist2d(s.vxMetersPerSecond, s.vyMetersPerSecond, s.omegaRadiansPerSecond);
    }
}
