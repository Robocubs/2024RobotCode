package com.team1701.lib.util;

import edu.wpi.first.math.MathUtil;

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

    public static boolean sequentiallyMatch(double[] expected, double[] actual, double tolerance) {
        boolean matches = false;
        int shortestLength = expected.length < actual.length ? expected.length : actual.length;
        for (int i = 0; i < shortestLength; i++) {
            matches = MathUtil.isNear(expected[i], actual[i], tolerance);
        }
        return matches;
    }

    public static double[] clampAll(double[] values, double min, double max) {
        for (int i = 0; i < values.length; i++) {
            values[i] = MathUtil.clamp(values[i], min, max);
        }
        return values;
    }
}
