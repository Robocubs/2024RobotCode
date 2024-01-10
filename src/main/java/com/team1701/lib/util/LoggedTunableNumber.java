package com.team1701.lib.util;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or value not in
 * dashboard.
 */
public class LoggedTunableNumber {
    private static final String kTableKey = "TunableNumbers";
    private static boolean kTuningEnabled = true;

    private final String mKey;
    private boolean mHasDefault = false;
    private double mDefaultValue;
    private LoggedDashboardNumber mDashboardNumber;
    private Map<Integer, Double> mLastHasChangedValues = new HashMap<>();

    public static void enableTuning(boolean tuningEnabled) {
        kTuningEnabled = tuningEnabled;
    }

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public LoggedTunableNumber(String dashboardKey) {
        mKey = kTableKey + "/" + dashboardKey;
    }

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public LoggedTunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(double defaultValue) {
        if (!mHasDefault) {
            mHasDefault = true;
            this.mDefaultValue = defaultValue;
            if (kTuningEnabled) {
                mDashboardNumber = new LoggedDashboardNumber(mKey, defaultValue);
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public double get() {
        if (!mHasDefault) {
            return 0.0;
        } else {
            return kTuningEnabled ? mDashboardNumber.get() : mDefaultValue;
        }
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple objects. Recommended
     * approach is to pass the result of "hashCode()".
     * @return True if the number has changed since the last time this method was called, false otherwise.
     */
    public boolean hasChanged(int id) {
        var currentValue = get();
        var lastValue = mLastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            mLastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }
}
