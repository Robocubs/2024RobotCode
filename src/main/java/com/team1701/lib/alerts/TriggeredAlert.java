package com.team1701.lib.alerts;

import java.util.function.BooleanSupplier;

import com.team1701.lib.alerts.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggeredAlert {
    private final Alert mAlert;

    public static TriggeredAlert info(String message, BooleanSupplier trigger) {
        return info(message, new Trigger(trigger));
    }

    public static TriggeredAlert info(String message, Trigger trigger) {
        return new TriggeredAlert(AlertType.INFO, message, trigger);
    }

    public static TriggeredAlert warning(String message, BooleanSupplier trigger) {
        return warning(message, new Trigger(trigger));
    }

    public static TriggeredAlert warning(String message, Trigger trigger) {
        return new TriggeredAlert(AlertType.WARNING, message, trigger);
    }

    public static TriggeredAlert error(String message, BooleanSupplier trigger) {
        return error(message, new Trigger(trigger));
    }

    public static TriggeredAlert error(String message, Trigger trigger) {
        return new TriggeredAlert(AlertType.ERROR, message, trigger);
    }

    private TriggeredAlert(AlertType type, String message, Trigger trigger) {
        mAlert = new Alert(type, message);
        trigger.onTrue(Commands.runOnce(mAlert::enable).ignoringDisable(true).withName("EnableAlert"));
        trigger.onFalse(Commands.runOnce(mAlert::disable).ignoringDisable(true).withName("DisableAlert"));
        mAlert.setEnabled(trigger.getAsBoolean());
    }
}
