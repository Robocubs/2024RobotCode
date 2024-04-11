package com.team1701.robot.commands;

import java.util.ArrayList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.team1701.lib.util.PolynomialRegression;
import com.team1701.robot.Constants;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

public class CharacterizationCommands {
    private static final double kStartDelaySeconds = 2.0;
    private static final double kDriveRampAmpsPerSecond = 0.2;
    private static final double kShooterRampVoltsPerSecond = 0.1;
    private static final double kWheelCharacterizationVelocity = 1.0;

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

    private static class WheelRadiusCharacterizationData {
        private double lastGyroYawRadians = 0.0;
        private double cumulativeGyroYawRadians = 0.0;
        private SwerveModulePosition[] startSwerveModulePosition;
        private double currentEffectiveWheelRadius = 0.0;
    }

    public static Command runWheelRadiusCharacterization(Drive drive) {
        var rotationLimiter = new SlewRateLimiter(1.0);
        var data = new WheelRadiusCharacterizationData();
        return new FunctionalCommand(
                        () -> {
                            data.lastGyroYawRadians =
                                    drive.getFieldRelativeHeading().getRadians();
                            data.cumulativeGyroYawRadians = 0.0;
                            data.startSwerveModulePosition = drive.getMeasuredModulePositions();
                            rotationLimiter.reset(0);
                        },
                        () -> {
                            drive.runWheelCharacterization(rotationLimiter.calculate(kWheelCharacterizationVelocity));

                            var gyroYawRadians = drive.getFieldRelativeHeading().getRadians();
                            data.cumulativeGyroYawRadians +=
                                    MathUtil.angleModulus(gyroYawRadians - data.lastGyroYawRadians);
                            data.lastGyroYawRadians = gyroYawRadians;
                            var averageWheelPosition = 0.0;
                            var modulePositions = drive.getMeasuredModulePositions();
                            for (int i = 0; i < 4; i++) {
                                averageWheelPosition += Math.abs(modulePositions[i].distanceMeters
                                                - data.startSwerveModulePosition[i].distanceMeters)
                                        / Constants.Drive.kWheelRadiusMeters;
                            }
                            averageWheelPosition /= 4.0;

                            data.currentEffectiveWheelRadius =
                                    (data.cumulativeGyroYawRadians * Constants.Drive.kModuleRadius)
                                            / averageWheelPosition;
                            Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
                            Logger.recordOutput(
                                    "Drive/RadiusCharacterization/AccumGyroYawRads", data.cumulativeGyroYawRadians);
                            Logger.recordOutput(
                                    "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
                                    Units.metersToInches(data.currentEffectiveWheelRadius));
                        },
                        (interrupted) -> {
                            drive.stop();
                            if (data.cumulativeGyroYawRadians <= Math.PI * 2.0) {
                                System.out.println("Not enough data for characterization");
                            } else {
                                System.out.println("Effective Wheel Radius: "
                                        + Units.metersToInches(data.currentEffectiveWheelRadius) + " inches");
                            }
                        },
                        () -> false,
                        drive)
                .withName("DriveWheelRadiusCharacterization");
    }
}
