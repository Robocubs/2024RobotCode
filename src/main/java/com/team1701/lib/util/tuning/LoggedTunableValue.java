package com.team1701.lib.util.tuning;

public interface LoggedTunableValue {
    public static boolean kTuningEnabled = true;

    /**
     * Checks whether the value has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple objects. Recommended
     * approach is to pass the result of "hashCode()".
     * @return True if the value has changed since the last time this method was called, false otherwise.
     */
    public boolean hasChanged(int id);

    /**
     * Runs action if any of the tunableValues have changed
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple objects.
     * Recommended approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable values have changed
     * @param tunableValues All tunable values to check
     */
    public static void ifChanged(int id, Runnable action, LoggedTunableValue... tunableValues) {
        if (!kTuningEnabled) {
            return;
        }

        for (var value : tunableValues) {
            if (value.hasChanged(id)) {
                action.run();
                break;
            }
        }
    }
}
