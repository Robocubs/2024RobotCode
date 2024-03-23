package com.team1701.lib.util.tuning;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

/**
 * Class for a tunable boolean. Gets value from dashboard in tuning mode, returns default if not or value not in
 * dashboard.
 */
public class LoggedTunableBoolean implements LoggedTunableValue {
    private static final String kTableKey = "TunableBooleans";

    private final String mKey;
    private boolean mHasDefault = false;
    private boolean mDefaultValue;
    private LoggedDashboardBoolean mDashboardBoolean;
    private Map<Integer, Boolean> mLastHasChangedValues = new HashMap<>();

    /**
     * Create a new LoggedTunableBoolean
     *
     * @param dashboardKey Key on dashboard
     */
    public LoggedTunableBoolean(String dashboardKey) {
        mKey = kTableKey + "/" + dashboardKey;
    }

    /**
     * Create a new LoggedTunableBoolean with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public LoggedTunableBoolean(String dashboardKey, boolean defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the boolean. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(boolean defaultValue) {
        if (!mHasDefault) {
            mHasDefault = true;
            this.mDefaultValue = defaultValue;
            if (LoggedTunableValue.kTuningEnabled) {
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
        if (mHasDefault && LoggedTunableValue.kTuningEnabled) {
            return mDashboardBoolean.get();
        }

        return mDefaultValue;
    }

    /**
     * Checks whether the boolean has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple objects. Recommended
     * approach is to pass the result of "hashCode()".
     * @return True if the boolean has changed since the last time this method was called, false otherwise.
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
