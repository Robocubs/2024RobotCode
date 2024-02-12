package com.team1701.robot.util;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.team1701.lib.alerts.REVAlert;
import com.team1701.lib.drivers.motors.MotorIOSparkFlex;
import com.team1701.lib.drivers.motors.MotorIOSparkMax;
import com.team1701.robot.Constants;

public class SparkMotorFactory {
    public static MotorIOSparkFlex createShooterMotorIOSparkFlex(
            int deviceId, ShooterMotorUsage motorUse, boolean inverted) {
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

        double reduction = 1.0;
        switch (motorUse) {
            case ROLLER:
                configureWithRetry(() -> controller.setP(Constants.Shooter.kRollerKp.get()), errorAlert);
                configureWithRetry(() -> controller.setD(Constants.Shooter.kRollerKd.get()), errorAlert);
                configureWithRetry(() -> controller.setFF(Constants.Shooter.kRollerKff.get()), errorAlert);
                reduction = Constants.Shooter.kRollerReduction;
                break;
            case ROTATION:
                configureWithRetry(() -> controller.setP(Constants.Shooter.kRotationKp.get()), errorAlert);
                configureWithRetry(() -> controller.setD(Constants.Shooter.kRotationKd.get()), errorAlert);
                configureWithRetry(() -> controller.setFF(Constants.Shooter.kRotationKff.get()), errorAlert);
                configureWithRetry(
                        () -> motor.setSoftLimit(
                                SoftLimitDirection.kForward, (float) Constants.Shooter.kShooterUpperLimitRotations),
                        errorAlert);
                reduction = Constants.Shooter.kAngleReduction;
                break;
            default:
                break;
        }

        configureWithRetry(() -> motor.burnFlash(), errorAlert);

        motor.setCANTimeout(0);

        return new MotorIOSparkFlex(motor, reduction);
    }

    public static MotorIOSparkFlex createShooterMotorIOSparkFlex(
            int deviceId, ShooterMotorUsage motorUse, boolean inverted, int idToFollow) {
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

        double reduction = 1.0;
        switch (motorUse) {
            case ROLLER:
                configureWithRetry(() -> controller.setP(Constants.Shooter.kRollerKp.get()), errorAlert);
                configureWithRetry(() -> controller.setD(Constants.Shooter.kRollerKd.get()), errorAlert);
                configureWithRetry(() -> controller.setFF(Constants.Shooter.kRollerKff.get()), errorAlert);
                reduction = Constants.Shooter.kRollerReduction;
                break;
            case ROTATION:
                configureWithRetry(() -> controller.setP(Constants.Shooter.kRotationKp.get()), errorAlert);
                configureWithRetry(() -> controller.setD(Constants.Shooter.kRotationKd.get()), errorAlert);
                configureWithRetry(() -> controller.setFF(Constants.Shooter.kRotationKff.get()), errorAlert);
                reduction = Constants.Shooter.kAngleReduction;
                break;
            default:
                break;
        }

        motor.follow(ExternalFollower.kFollowerSpark, idToFollow, inverted);

        configureWithRetry(() -> motor.burnFlash(), errorAlert);

        motor.setCANTimeout(0);

        return new MotorIOSparkFlex(motor, reduction);
    }

    public static MotorIOSparkFlex createIndexerMotorIOSparkFlex(int deviceId) {
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

        return new MotorIOSparkFlex(motor, Constants.Indexer.kIndexerReduction);
    }

    public static MotorIOSparkFlex createIntakeMotorIOSparkFlex(int deviceId) {
        var motor = new CANSparkFlex(deviceId, MotorType.kBrushless);
        var encoder = motor.getEncoder();
        var errorAlert = new REVAlert(motor, deviceId);

        motor.setCANTimeout(200);

        configureWithRetry(() -> motor.restoreFactoryDefaults(), errorAlert);

        configureWithRetry(() -> motor.setSmartCurrentLimit(20), errorAlert);
        configureWithRetry(() -> motor.enableVoltageCompensation(12), errorAlert);

        configureWithRetry(() -> encoder.setPosition(0), errorAlert);
        configureWithRetry(() -> encoder.setMeasurementPeriod(10), errorAlert);
        configureWithRetry(() -> encoder.setAverageDepth(2), errorAlert);

        configureWithRetry(() -> motor.burnFlash(), errorAlert);

        motor.setCANTimeout(0);

        return new MotorIOSparkFlex(motor, Constants.Indexer.kIntakeReduction);
    }

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

    public enum ShooterMotorUsage {
        ROLLER,
        ROTATION
    }
}
