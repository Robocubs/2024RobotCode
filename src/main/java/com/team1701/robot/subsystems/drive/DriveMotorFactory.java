package com.team1701.robot.subsystems.drive;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.team1701.lib.alerts.REVAlert;
import com.team1701.lib.drivers.motors.MotorIOSparkMax;
import com.team1701.robot.Constants;

public final class DriveMotorFactory {
    public static MotorIOSparkMax createDriveMotorIOSparkMax(int deviceId) {
        var motor = new CANSparkMax(deviceId, MotorType.kBrushless);
        var encoder = motor.getEncoder();
        var controller = motor.getPIDController();
        var errorAlert = new REVAlert(motor, deviceId);

        motor.setCANTimeout(200);

        configureWithRetry(() -> motor.restoreFactoryDefaults(), errorAlert);

        configureWithRetry(() -> motor.setSmartCurrentLimit(80), errorAlert);
        configureWithRetry(() -> motor.enableVoltageCompensation(12), errorAlert);

        configureWithRetry(() -> encoder.setPosition(0), errorAlert);
        configureWithRetry(() -> encoder.setMeasurementPeriod(10), errorAlert);
        configureWithRetry(() -> encoder.setAverageDepth(2), errorAlert);

        configureWithRetry(() -> controller.setP(Constants.Drive.kDriveKp.get()), errorAlert);
        configureWithRetry(() -> controller.setD(Constants.Drive.kDriveKd.get()), errorAlert);
        configureWithRetry(() -> controller.setFF(Constants.Drive.kDriveKff.get()), errorAlert);

        configureWithRetry(() -> motor.burnFlash(), errorAlert);

        motor.setInverted(Constants.Drive.kDriveMotorsInverted);
        motor.setCANTimeout(0);

        return new MotorIOSparkMax(motor, Constants.Drive.kDriveReduction);
    }

    public static MotorIOSparkMax createSteerMotorIOSparkMax(int deviceId) {
        var motor = new CANSparkMax(deviceId, MotorType.kBrushless);
        var encoder = motor.getEncoder();
        var controller = motor.getPIDController();
        var errorAlert = new REVAlert(motor, deviceId);

        motor.setCANTimeout(200);

        configureWithRetry(() -> motor.restoreFactoryDefaults(), errorAlert);

        configureWithRetry(() -> motor.setSmartCurrentLimit(30), errorAlert);
        configureWithRetry(() -> motor.enableVoltageCompensation(12.0), errorAlert);

        configureWithRetry(() -> encoder.setPosition(0), errorAlert);
        configureWithRetry(() -> encoder.setMeasurementPeriod(10), errorAlert);
        configureWithRetry(() -> encoder.setAverageDepth(2), errorAlert);

        configureWithRetry(() -> controller.setP(Constants.Drive.kSteerKp.get()), errorAlert);
        configureWithRetry(() -> controller.setD(Constants.Drive.kSteerKd.get()), errorAlert);

        configureWithRetry(() -> controller.setPositionPIDWrappingEnabled(true), errorAlert);
        configureWithRetry(() -> controller.setPositionPIDWrappingMinInput(0), errorAlert);
        configureWithRetry(
                () -> controller.setPositionPIDWrappingMaxInput(1.0 / Constants.Drive.kSteerReduction), errorAlert);

        configureWithRetry(() -> motor.burnFlash(), errorAlert);

        motor.setInverted(Constants.Drive.kSteerMotorsInverted);
        motor.setCANTimeout(0);

        return new MotorIOSparkMax(motor, Constants.Drive.kSteerReduction);
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
