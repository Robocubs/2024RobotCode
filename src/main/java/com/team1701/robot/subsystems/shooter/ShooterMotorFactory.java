package com.team1701.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.team1701.lib.alerts.REVAlert;
import com.team1701.lib.drivers.motors.MotorIOSparkFlex;
import com.team1701.robot.Constants;

public class ShooterMotorFactory {
    public static MotorIOSparkFlex createShooterMotorIOSparkFlex(int deviceId, ShooterMotorUsage motorUse) {
        var motor = new CANSparkFlex(deviceId, MotorType.kBrushless);
        var encoder = motor.getEncoder();
        var controller = motor.getPIDController();
        var errorAlert = new REVAlert(motor, deviceId);

        motor.setCANTimeout(200);

        // TODO: Update values for actual shooter
        configureWithRetry(() -> motor.restoreFactoryDefaults(), errorAlert);

        configureWithRetry(() -> motor.setSmartCurrentLimit(80), errorAlert);
        configureWithRetry(() -> motor.enableVoltageCompensation(12), errorAlert);

        configureWithRetry(() -> encoder.setPosition(0), errorAlert);
        configureWithRetry(() -> encoder.setMeasurementPeriod(10), errorAlert);
        configureWithRetry(() -> encoder.setAverageDepth(2), errorAlert);

        switch (motorUse) {
            case ROLLER:
                configureWithRetry(() -> controller.setP(Constants.Shooter.kRollerKp.get()), errorAlert);
                configureWithRetry(() -> controller.setD(Constants.Shooter.kRollerKd.get()), errorAlert);
                configureWithRetry(() -> controller.setFF(Constants.Shooter.kRollerKff.get()), errorAlert);
                break;
            case ROTATION:
                configureWithRetry(() -> controller.setP(Constants.Shooter.kRotationKp.get()), errorAlert);
                configureWithRetry(() -> controller.setD(Constants.Shooter.kRotationKd.get()), errorAlert);
                configureWithRetry(() -> controller.setFF(Constants.Shooter.kRotationKff.get()), errorAlert);
                break;
            default:
                break;
        }

        configureWithRetry(() -> motor.burnFlash(), errorAlert);

        motor.setCANTimeout(0);

        return new MotorIOSparkFlex(motor, Constants.Shooter.kShooterReduction);
    }

    private static void configureWithRetry(Supplier<REVLibError> config, REVAlert failureAlert) {
        REVLibError error = REVLibError.kUnknown;
        for (var i = 0; i < 4; i++) {
            error = config.get();
            if (error == REVLibError.kOk) {
                return;
            }
        }

        failureAlert.enable(error);
    }

    public enum ShooterMotorUsage {
        ROLLER,
        ROTATION
    }
}
