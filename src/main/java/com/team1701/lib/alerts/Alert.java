package com.team1701.lib.alerts;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Alert {
    private static SendableAlerts mSendableAlerts = null;

    private AlertType mType;
    private String mMessage;
    private boolean mEnabled;
    private long mLastEnabledTimestamp;
    private boolean mLogResolution;

    public static Alert info(String message) {
        return new Alert(AlertType.INFO, message);
    }

    public static Alert warning(String message) {
        return new Alert(AlertType.WARNING, message);
    }

    public static Alert error(String message) {
        return new Alert(AlertType.ERROR, message);
    }

    Alert(AlertType type, String message) {
        mType = type;
        mMessage = message;
        mLogResolution = type != AlertType.INFO;

        synchronized (Alert.class) {
            if (mSendableAlerts == null) {
                mSendableAlerts = new SendableAlerts();
                SmartDashboard.putData("Alerts", mSendableAlerts);
            }
        }

        mSendableAlerts.add(this);
    }

    public Alert withResolutionLogging(boolean shouldLogResolution) {
        mLogResolution = shouldLogResolution;
        return this;
    }

    public void setMessage(String message) {
        if (message.equals(mMessage)) {
            return;
        }

        mMessage = message;

        if (mEnabled) {
            logToConsole();
            mSendableAlerts.invalidateCache();
        }
    }

    public void setEnabled(boolean enabled) {
        if (enabled == mEnabled) {
            return;
        }

        mEnabled = enabled;
        mSendableAlerts.invalidateCache();

        if (enabled) {
            logToConsole();
        } else if (mLogResolution) {
            System.out.println("INFO: [RESOLVED] " + mMessage);
        }
    }

    public void enable() {
        setEnabled(true);
    }

    public void disable() {
        setEnabled(false);
    }

    private void logToConsole() {
        switch (mType) {
            case INFO:
                System.out.println("INFO: " + mMessage);
                break;
            case WARNING:
                DriverStation.reportWarning(mMessage, false);
                break;
            case ERROR:
                DriverStation.reportError(mMessage, false);
                break;
        }
    }

    private static class SendableAlerts implements Sendable {
        private final List<Alert> mAlerts = new ArrayList<>();
        private String[] mInfoMessages = new String[] {};
        private String[] mWarningMessages = new String[] {};
        private String[] mErrorMessages = new String[] {};
        private boolean mCacheValid = true;

        private void add(Alert alert) {
            mAlerts.add(alert);
        }

        private void invalidateCache() {
            mCacheValid = false;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Alerts");
            builder.addStringArrayProperty("infos", () -> getCachedMessages(AlertType.INFO), null);
            builder.addStringArrayProperty("warnings", () -> getCachedMessages(AlertType.WARNING), null);
            builder.addStringArrayProperty("errors", () -> getCachedMessages(AlertType.ERROR), null);
        }

        private String[] getCachedMessages(AlertType alertType) {
            updateCache();
            switch (alertType) {
                case INFO:
                    return mInfoMessages;
                case WARNING:
                    return mWarningMessages;
                case ERROR:
                    return mErrorMessages;
                default:
                    return new String[] {};
            }
        }

        private void updateCache() {
            if (mCacheValid) {
                return;
            }

            mInfoMessages = getMessages(AlertType.INFO);
            mWarningMessages = getMessages(AlertType.WARNING);
            mErrorMessages = getMessages(AlertType.ERROR);

            mCacheValid = true;
        }

        private String[] getMessages(AlertType type) {
            return mAlerts.stream()
                    .filter(alert -> alert.mType == type && alert.mEnabled)
                    .sorted((alert1, alert2) -> (int) (alert1.mLastEnabledTimestamp - alert2.mLastEnabledTimestamp))
                    .map(alert -> alert.mMessage)
                    .toArray(String[]::new);
        }
    }

    public static enum AlertType {
        INFO,
        WARNING,
        ERROR
    }
}
