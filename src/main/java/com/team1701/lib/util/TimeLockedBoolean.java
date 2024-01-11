package com.team1701.lib.util;

public class TimeLockedBoolean {
    private final double mMinimumTime;
    private final boolean mLockedValue;
    private double mLastUnlockedValueTimestamp;
    private boolean mValue;

    public TimeLockedBoolean(double minimumTime, double timestamp) {
        this(minimumTime, timestamp, true);
    }

    public TimeLockedBoolean(double minimumTime, double timestamp, boolean lockedValue) {
        this(minimumTime, timestamp, lockedValue, !lockedValue);
    }

    public TimeLockedBoolean(double minimumTime, double timestamp, boolean lockedValue, boolean initialValue) {
        mMinimumTime = minimumTime;
        mLastUnlockedValueTimestamp = timestamp;
        mLockedValue = lockedValue;
        mValue = initialValue;
    }

    public boolean update(boolean newValue, double timestamp) {
        if (newValue != mLockedValue) {
            mLastUnlockedValueTimestamp = timestamp;
            mValue = newValue;
            return mValue;
        }

        mValue = timestamp - mLastUnlockedValueTimestamp > mMinimumTime ? newValue : !newValue;
        return mValue;
    }

    public boolean getValue() {
        return mValue;
    }
}
