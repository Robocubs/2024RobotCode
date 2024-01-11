package com.team1701.lib.util;

public final class Util {
    public static final double kEpsilon = 1e-12;

    public static boolean epsilonEquals(double a, double b) {
        return (a - kEpsilon <= b) && (a + kEpsilon >= b);
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static boolean inRangeInclusive(double v, double maxMagnitude) {
        return inRangeInclusive(v, -maxMagnitude, maxMagnitude);
    }

    public static boolean inRangeInclusive(double v, double min, double max) {
        return v >= min && v <= max;
    }
}
