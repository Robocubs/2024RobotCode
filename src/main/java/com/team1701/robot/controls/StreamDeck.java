package com.team1701.robot.controls;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.team1701.lib.alerts.Alert;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class StreamDeck {
    private final Map<StreamDeckButton, Button> buttons = new HashMap<>();

    private static record Button(
            LoggedDashboardBoolean pressed, BooleanSupplier selected, BooleanPublisher activePub) {}

    public StreamDeck() {
        CommandScheduler.getInstance()
                .schedule(Commands.run(() -> buttons.values()
                                .forEach(button -> button.activePub.set(button.selected.getAsBoolean())))
                        .ignoringDisable(true)
                        .withName("StreamDeckPeriodic"));
    }

    public StreamDeck configureButton(Consumer<ButtonConfiguration> config) {
        var configuration = new ButtonConfiguration();
        config.accept(configuration);

        var nt = NetworkTableInstance.getDefault();
        var deckTable = nt.getTable("StreamDeck");
        configuration.buttonConfigurations.forEach((button, selected) -> {
            var table = deckTable.getSubTable("Button/" + button.index);
            table.getStringTopic("Key").publish().set("/SmartDashboard/" + button.key);
            table.getStringTopic("Icon").publish().set(button.icon);
            table.getStringTopic("Label").publish().set(button.label);

            var dashboardBoolean = new LoggedDashboardBoolean(button.key, false);
            buttons.put(
                    button,
                    new Button(
                            dashboardBoolean,
                            selected.orElse(dashboardBoolean::get),
                            table.getBooleanTopic("Selected").publish()));
        });

        deckTable.getIntegerTopic("LastModified").publish().set(Logger.getTimestamp());

        return this;
    }

    public Trigger button(StreamDeckButton button) {
        if (!buttons.containsKey(button)) {
            Alert.warning("Stream Deck button trigger added for invalid button " + button.index)
                    .enable();
            return new Trigger(() -> false);
        }

        return new Trigger(buttons.get(button).pressed::get);
    }

    public ButtonGroup buttonGroup() {
        return new ButtonGroup();
    }

    public static enum StreamDeckButton {
        // kButton(0, 0, "Controls/TestButton", "ArrowUpward", "Button"),
        kStopIntakeButton(1, 0, "Controls/StopIntakingButton", "StopIntakingIcon", "Stop Intake"),
        kRejectButton(2, 0, "Controls/RejectButton", "RejectIntakeIcon", "Reject Intake"),
        kForwardButton(0, 0, "Controls/ForwardButton", "ForceIntakeIcon", "Forward Intake"),
        kArmUpButton(0, 1, "Controls/ArmUpButton", "RaiseArmIcon", "Arm Up"),
        kArmDownButton(2, 1, "Controls/ArmDownButton", "LowerArmIcon", "Arm Down"),
        kArmHomeButton(1, 1, "Controls/ArmHomeButton", "Circle", "Arm Home"),
        kShooterUpButton(0, 3, "Controls/ShooterUpButton", "RaiseShooterIcon", "Shooter Up"),
        kShooterDownButton(2, 3, "Controls/ShooterDownButton", "LowerShooterIcon", "Shooter Down"),
        kShootButton(1, 3, "Controls/ShootButton", "ShootIcon", "Shoot"),
        kSpeakerModeButton(0, 4, "Controls/SpeakerModeButton", "SpeakerModeIcon", "Speaker Mode"),
        kAmpModeButton(1, 4, "Controls/AmpModeButton", "AmpModeIcon", "Amp Mode"),
        kClimbModeButton(2, 4, "Controls/ClimbModeButton", "ClimbModeIcon", "Climb Mode"),
        kExtendWinchButton(0, 2, "Controls/ExtendWinchButton", "ExtendWinchIcon", "Extend Winch"),
        kRetractWinchButton(2, 2, "Controls/RetractWinchButton", "RetractWinchIcon", "Retract Winch");

        private final int index;
        private final String key;
        private final String icon;
        private final String label;

        private StreamDeckButton(int row, int col, String key, String icon, String label) {
            index = row * 5 + col % 5;
            this.key = key;
            this.icon = icon;
            this.label = label;
        }
    }

    public class ButtonConfiguration {
        private final Map<StreamDeckButton, Optional<BooleanSupplier>> buttonConfigurations = new HashMap<>();

        private ButtonConfiguration() {}

        public ButtonConfiguration addDefault(StreamDeckButton button) {
            buttonConfigurations.put(button, Optional.empty());
            return this;
        }

        public ButtonConfiguration add(StreamDeckButton button, BooleanSupplier selected) {
            buttonConfigurations.put(button, Optional.of(selected));
            return this;
        }
    }

    public class ButtonGroup {
        private final Map<StreamDeckButton, Trigger> triggers = new HashMap<>();
        private Optional<StreamDeckButton> selected = Optional.empty();

        private ButtonGroup() {}

        public ButtonGroup option(StreamDeckButton button) {
            initButton(button);
            return this;
        }

        public ButtonGroup option(StreamDeckButton button, Consumer<Trigger> trigger) {
            var buttonTrigger = initButton(button);
            trigger.accept(buttonTrigger);
            return this;
        }

        public ButtonGroup select(StreamDeckButton button) {
            selected = Optional.of(button);
            return this;
        }

        public ButtonGroup clear() {
            selected = Optional.empty();
            return this;
        }

        public Optional<StreamDeckButton> getSelected() {
            return selected;
        }

        public boolean isSelected(StreamDeckButton button) {
            return selected.isPresent() && selected.get() == button;
        }

        public Trigger trigger(StreamDeckButton button) {
            if (triggers.containsKey(button)) {
                return triggers.get(button);
            }

            return initButton(button);
        }

        private Trigger initButton(StreamDeckButton button) {
            button(button)
                    .onTrue(Commands.runOnce(() -> selected = Optional.of(button))
                            .ignoringDisable(true)
                            .withName("UpdateStreamDeckButtonGroupActiveIndex"));

            var trigger = new Trigger(() -> isSelected(button));
            triggers.put(button, trigger);
            return trigger;
        }
    }
}
