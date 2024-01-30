package com.team1701.robot.subsystems.indexer;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.team1701.lib.alerts.REVAlert;
import com.team1701.lib.drivers.motors.MotorIOSparkFlex;
import com.team1701.robot.Constants;

public class IndexerMotorFactory {
    public static MotorIOSparkFlex createDriveMotorIOSparkFlex(int deviceId) {
        var motor = new CANSparkFlex(deviceId, MotorType.kBrushless);
        var encoder = motor.getEncoder();
        var controller = motor.getPIDController();
        var errorAlert = new REVAlert(motor, deviceId);

        motor.setCANTimeout(200);

        configureWithRetry(() -> motor.restoreFactoryDefaults(), errorAlert);

        configureWithRetry(() -> motor.setSmartCurrentLimit(20), errorAlert);
        configureWithRetry(() -> motor.enableVoltageCompensation(12), errorAlert);

        configureWithRetry(() -> encoder.setPosition(0), errorAlert);
        configureWithRetry(() -> encoder.setMeasurementPeriod(10), errorAlert);
        configureWithRetry(() -> encoder.setAverageDepth(2), errorAlert);

        configureWithRetry(() -> controller.setP(Constants.Indexer.kIndexerKp.get()), errorAlert);
        configureWithRetry(() -> controller.setD(Constants.Indexer.kIndexerKd.get()), errorAlert);
        configureWithRetry(() -> controller.setFF(Constants.Indexer.kIndexerKff.get()), errorAlert);

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
}
