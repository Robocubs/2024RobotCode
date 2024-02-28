package com.team1701.robot.util;

import java.util.function.Supplier;

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
    public static MotorIOSparkFlex createShooterMotorIOSparkFlex(int deviceId, MotorUsage motorUse, boolean inverted) {
        var motor = new CANSparkFlex(deviceId, MotorType.kBrushless);
        var encoder = motor.getEncoder();
        var controller = motor.getPIDController();
        var errorAlert = new REVAlert(motor, deviceId);

        motor.setCANTimeout(200);

        // TODO: Update values for actual shooter
        configureWithRetry(() -> motor.restoreFactoryDefaults(), errorAlert);

        configureWithRetry(() -> motor.enableVoltageCompensation(12), errorAlert);

        configureWithRetry(() -> encoder.setPosition(0), errorAlert);
        configureWithRetry(() -> encoder.setMeasurementPeriod(10), errorAlert);
        configureWithRetry(() -> encoder.setAverageDepth(2), errorAlert);

        double reduction = 1.0;
        switch (motorUse) {
            case SHOOTER_ROLLER:
                configureWithRetry(() -> motor.setSmartCurrentLimit(80), errorAlert);
                configureWithRetry(() -> motor.setClosedLoopRampRate(0.2), errorAlert);
                configureWithRetry(() -> controller.setP(/*Constants.Shooter.kRollerKp.get()*/ 0), errorAlert);
                configureWithRetry(() -> controller.setD(/*Constants.Shooter.kRollerKd.get()*/ 0), errorAlert);
                configureWithRetry(() -> controller.setFF(/*Constants.Shooter.kRollerKff.get()*/ 0), errorAlert);
                reduction = Constants.Shooter.kRollerReduction;
                break;
            case ROTATION:
                configureWithRetry(() -> motor.setSmartCurrentLimit(40), errorAlert);
                configureWithRetry(() -> motor.setClosedLoopRampRate(0.2), errorAlert);
                configureWithRetry(() -> controller.setP(Constants.Shooter.kRotationKp.get()), errorAlert);
                configureWithRetry(() -> controller.setD(Constants.Shooter.kRotationKd.get()), errorAlert);
                configureWithRetry(() -> controller.setFF(0), errorAlert);
                configureWithRetry(
                        () -> motor.setSoftLimit(SoftLimitDirection.kForward, (float)
                                (Constants.Shooter.kShooterUpperLimitRotations / Constants.Shooter.kAngleReduction)),
                        errorAlert);
                configureWithRetry(
                        () -> motor.setSoftLimit(SoftLimitDirection.kReverse, (float)
                                (Constants.Shooter.kShooterLowerLimitRotations / Constants.Shooter.kAngleReduction)),
                        errorAlert);
                configureWithRetry(() -> motor.enableSoftLimit(SoftLimitDirection.kForward, true), errorAlert);
                configureWithRetry(() -> motor.enableSoftLimit(SoftLimitDirection.kReverse, true), errorAlert);
                reduction = Constants.Shooter.kAngleReduction;

                break;
            default:
                break;
        }

        invertWithRetry(motor, inverted);

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

        configureWithRetry(() -> motor.setSmartCurrentLimit(40), errorAlert);
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

        configureWithRetry(() -> motor.setSmartCurrentLimit(40), errorAlert);
        configureWithRetry(() -> motor.setOpenLoopRampRate(0.2), errorAlert);
        configureWithRetry(() -> motor.enableVoltageCompensation(12), errorAlert);

        configureWithRetry(() -> encoder.setPosition(0), errorAlert);
        configureWithRetry(() -> encoder.setMeasurementPeriod(10), errorAlert);
        configureWithRetry(() -> encoder.setAverageDepth(2), errorAlert);

        configureWithRetry(() -> motor.burnFlash(), errorAlert);

        motor.setCANTimeout(0);

        return new MotorIOSparkFlex(motor, Constants.Indexer.kIntakeReduction);
    }

    public static MotorIOSparkFlex createArmClimbMotorIOSparkFlex(int deviceId, MotorUsage motorUse, boolean inverted) {
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
            case WINCH:
                configureWithRetry(() -> controller.setP(Constants.Climb.kWinchKp.get()), errorAlert);
                configureWithRetry(() -> controller.setD(Constants.Climb.kWinchKd.get()), errorAlert);
                configureWithRetry(() -> controller.setFF(Constants.Climb.kWinchKff.get()), errorAlert);
                reduction = Constants.Climb.kWinchReduction;
                break;
            case ROTATION:
                configureWithRetry(() -> controller.setP(Constants.Arm.kArmRotationKp.get()), errorAlert);
                configureWithRetry(() -> controller.setD(Constants.Arm.kArmRotationKd.get()), errorAlert);
                configureWithRetry(() -> controller.setFF(Constants.Arm.kArmRotationKff.get()), errorAlert);
                configureWithRetry(
                        () -> motor.setSoftLimit(
                                SoftLimitDirection.kForward, (float) (Constants.Arm.kArmUpperLimitRotations)),
                        errorAlert);
                configureWithRetry(
                        () -> motor.setSoftLimit(
                                SoftLimitDirection.kReverse, (float) (Constants.Arm.kArmLowerLimitRotations)),
                        errorAlert);
                configureWithRetry(() -> motor.enableSoftLimit(SoftLimitDirection.kForward, true), errorAlert);
                configureWithRetry(() -> motor.enableSoftLimit(SoftLimitDirection.kReverse, true), errorAlert);
                reduction = Constants.Arm.kRotationReduction;
                break;
            default:
                break;
        }

        invertWithRetry(motor, inverted);

        configureWithRetry(() -> motor.burnFlash(), errorAlert);

        motor.setCANTimeout(0);

        return new MotorIOSparkFlex(motor, reduction);
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

    private static void invertWithRetry(CANSparkFlex motor, boolean inverted) {
        for (int i = 0; i < 4; i++) {
            motor.setInverted(inverted);
            if (motor.getInverted() == inverted) {
                break;
            }
        }
    }

    public enum MotorUsage {
        SHOOTER_ROLLER,
        ROTATION,
        WINCH
    }
}
