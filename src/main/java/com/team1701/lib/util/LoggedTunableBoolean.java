package com.team1701.lib.util;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or value not in
 * dashboard.
 */
public class LoggedTunableBoolean {
    private static final String kTableKey = "TunableBooleans";
    private static boolean kTuningEnabled = true;

    private final String mKey;
    private boolean mHasDefault = false;
    private boolean mDefaultValue;
    private LoggedDashboardBoolean mDashboardBoolean;
    private Map<Integer, Boolean> mLastHasChangedValues = new HashMap<>();

    public static void enableTuning(boolean tuningEnabled) {
        kTuningEnabled = tuningEnabled;
    }

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public LoggedTunableBoolean(String dashboardKey) {
        mKey = kTableKey + "/" + dashboardKey;
    }

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public LoggedTunableBoolean(String dashboardKey, boolean defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(boolean defaultValue) {
        if (!mHasDefault) {
            mHasDefault = true;
            this.mDefaultValue = defaultValue;
            if (kTuningEnabled) {
                mDashboardBoolean = new LoggedDashboardBoolean(mKey, defaultValue);
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public boolean get() {
        if (mHasDefault && kTuningEnabled) {
            return mDashboardBoolean.get();
        }

        return mDefaultValue;
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
