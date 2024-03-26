package com.team1701.robot.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RumbleController extends SubsystemBase {
    private static final double kRumblePulseDurationSeconds = 0.1;
    private static final double kRumblePulsePauseSeconds = 0.1;

    private final GenericHID mController;
    private double mRumbleIntensity;

    public RumbleController(GenericHID controller) {
        mController = controller;

        setDefaultCommand(startEnd(
                        () -> {
                            controller.setRumble(RumbleType.kBothRumble, 0);
                            mRumbleIntensity = 0;
                        },
                        () -> {})
                .ignoringDisable(true)
                .withName("IdleRumble"));
    }

    @Override
    public void simulationPeriodic() {
        Logger.recordOutput("RumbleController/Intensity", mRumbleIntensity);
    }

    public Command rumble() {
        return rumble(RumbleIntensity.kDefault);
    }

    public Command rumble(RumbleIntensity intensity) {
        return startEnd(
                        () -> {
                            mController.setRumble(RumbleType.kBothRumble, intensity.value);
                            mRumbleIntensity = intensity.value;
                        },
                        () -> {
                            mController.setRumble(RumbleType.kBothRumble, 0);
                            mRumbleIntensity = 0;
                        })
                .ignoringDisable(true)
                .withName("Rumble");
    }

    public Command rumbleSeconds(double seconds) {
        return rumbleSeconds(seconds, RumbleIntensity.kDefault);
    }

    public Command rumbleSeconds(double seconds, RumbleIntensity intensity) {
        return rumble(intensity).withTimeout(seconds).withName("RumbleSeconds");
    }

    public Command rumblePulses(int pulses) {
        return rumblePulses(pulses, RumbleIntensity.kDefault);
    }

    public Command rumblePulses(int pulses, RumbleIntensity intensity) {
        if (pulses < 1) {
            return Commands.none();
        }

        var commands = new Command[pulses * 2 - 1];
        commands[0] = rumbleSeconds(kRumblePulseDurationSeconds);

        for (var i = 1; i < pulses; i++) {
            commands[i * 2 - 1] = Commands.waitSeconds(kRumblePulsePauseSeconds);
            commands[i * 2] = rumbleSeconds(kRumblePulseDurationSeconds);
        }

        return Commands.sequence(commands).withName("RumblePulses");
    }

    public static class RumbleIntensity {
        public static final RumbleIntensity kDefault = new RumbleIntensity(0.5);
        public static final RumbleIntensity kLight = new RumbleIntensity(0.25);
        public static final RumbleIntensity kStrong = new RumbleIntensity(1.0);

        private final double value;

        private RumbleIntensity(double value) {
            this.value = value;
        }
    }
}
