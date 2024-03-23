package com.team1701.robot.commands;

import java.util.ArrayList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.team1701.lib.util.PolynomialRegression;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CharacterizationCommands {
    private static final double kStartDelaySeconds = 2.0;
    private static final double kDriveRampAmpsPerSecond = 0.2;
    private static final double kShooterRampVoltsPerSecond = 0.1;

    public static Command runDriveCharacterization(Drive drive) {
        return runFeedforwardCharacterization(
                "Drive",
                kDriveRampAmpsPerSecond,
                drive::runCharacterization,
                drive::getAverageModuleSpeedRadiansPerSecond,
                drive);
    }

    public static Command runShooterCharacterization(Shooter shooter) {
        return Commands.parallel(
                runFeedforwardCharacterization(
                        "Upper Shooter Roller",
                        kShooterRampVoltsPerSecond,
                        shooter::runUpperRollerCharacterization,
                        () -> shooter.getRollerSpeedsRadiansPerSecond().upperSpeed()),
                runFeedforwardCharacterization(
                        "Lower Shooter Roller",
                        kShooterRampVoltsPerSecond,
                        shooter::runLowerRollerCharacterization,
                        () -> shooter.getRollerSpeedsRadiansPerSecond().lowerSpeed()),
                Commands.idle(shooter)); // You can't have two commands require the same subsystem
    }

    private static Command runFeedforwardCharacterization(
            String identifier,
            double inputRampRate,
            DoubleConsumer inputConsumer,
            DoubleSupplier velocitySupplier,
            Subsystem... subsystems) {
        var velocityData = new ArrayList<Double>();
        var voltageData = new ArrayList<Double>();
        var timer = new Timer();

        return new FunctionalCommand(
                        () -> {
                            velocityData.clear();
                            voltageData.clear();

                            timer.reset();
                            timer.start();
                        },
                        () -> {
                            if (timer.get() < kStartDelaySeconds) {
                                inputConsumer.accept(0.0);
                            } else {
                                var voltage = timer.get() * inputRampRate;
                                inputConsumer.accept(voltage);
                                voltageData.add(voltage);
                                velocityData.add(velocitySupplier.getAsDouble());
                            }
                        },
                        (var interrupted) -> {
                            inputConsumer.accept(0.0);
                            timer.stop();

                            if (velocityData.size() == 0) {
                                return;
                            }

                            var regression = new PolynomialRegression(
                                    velocityData.stream()
                                            .mapToDouble(Double::doubleValue)
                                            .toArray(),
                                    voltageData.stream()
                                            .mapToDouble(Double::doubleValue)
                                            .toArray(),
                                    1);

                            System.out.println("FF Characterization Results (" + identifier + "):");
                            System.out.println("\tCount=" + Integer.toString(velocityData.size()));
                            System.out.println(String.format("\tR2=%.5f", regression.R2()));
                            System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
                            System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
                        },
                        () -> false,
                        subsystems)
                .withName("FeedforwardCharacterization");
    }
}
