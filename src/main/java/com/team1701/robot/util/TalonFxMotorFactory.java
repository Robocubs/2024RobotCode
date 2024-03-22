package com.team1701.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1701.lib.drivers.motors.MotorIOTalonFX;
import com.team1701.lib.drivers.motors.MotorIOTalonFXFOC;
import com.team1701.robot.Constants;

public class TalonFxMotorFactory {
    private static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;

    public static MotorIOTalonFX createDriveMotorIOTalonFx(int deviceId) {
        var motor = new TalonFX(deviceId, "canivore1");

        var config = new TalonFXConfiguration();
        config.MotorOutput.withNeutralMode(NeutralModeValue.Coast).withInverted(kInverted);
        config.Feedback.withSensorToMechanismRatio(1 / Constants.Drive.kDriveReduction);
        config.CurrentLimits.withSupplyCurrentLimit(80).withSupplyCurrentLimitEnable(true);
        config.Slot0.withKS(Constants.Drive.kDriveKs.get())
                .withKV(Constants.Drive.kDriveKv.get())
                .withKA(Constants.Drive.kDriveKa.get())
                .withKP(Constants.Drive.kDriveKp.get())
                .withKD(Constants.Drive.kDriveKd.get());

        configureWithRetry(motor, config);

        motor.setPosition(0);

        return new MotorIOTalonFX(motor).withVelocityControlFrequency(0);
    }

    public static MotorIOTalonFXFOC createDriveMotorIOTalonFxFoc(int deviceId) {
        var motor = new TalonFX(deviceId, "canivore1");

        var config = new TalonFXConfiguration();
        config.MotorOutput.withNeutralMode(NeutralModeValue.Coast).withInverted(kInverted);
        config.Feedback.withSensorToMechanismRatio(1 / Constants.Drive.kDriveReduction);
        config.CurrentLimits.withSupplyCurrentLimit(120).withSupplyCurrentLimitEnable(true);
        config.TorqueCurrent.withPeakForwardTorqueCurrent(80).withPeakReverseTorqueCurrent(-80);
        config.ClosedLoopRamps.withTorqueClosedLoopRampPeriod(0.02);
        config.Slot0.withKS(Constants.Drive.kDriveKs.get())
                .withKV(Constants.Drive.kDriveKv.get())
                .withKA(Constants.Drive.kDriveKa.get())
                .withKP(Constants.Drive.kDriveKp.get())
                .withKD(Constants.Drive.kDriveKd.get());

        configureWithRetry(motor, config);

        motor.setPosition(0);

        return new MotorIOTalonFXFOC(motor).withVelocityControlFrequency(0);
    }

    public static MotorIOTalonFX createSteerMotorIOTalonFx(int deviceId) {
        var motor = new TalonFX(deviceId, "canivore1");

        var config = new TalonFXConfiguration();
        config.MotorOutput.withNeutralMode(NeutralModeValue.Coast).withInverted(kInverted);
        config.Feedback.withSensorToMechanismRatio(1 / Constants.Drive.kSteerReduction);
        config.CurrentLimits.withSupplyCurrentLimit(60).withSupplyCurrentLimitEnable(true);
        config.Slot0.withKP(Constants.Drive.kSteerKp.get()).withKD(Constants.Drive.kSteerKd.get());
        config.ClosedLoopGeneral.ContinuousWrap = true;

        configureWithRetry(motor, config);

        motor.setPosition(0);

        return new MotorIOTalonFX(motor).withPositionControlFrequency(0);
    }

    public static MotorIOTalonFXFOC createSteerMotorIOTalonFxFoc(int deviceId) {
        var motor = new TalonFX(deviceId, "canivore1");

        var config = new TalonFXConfiguration();
        config.MotorOutput.withNeutralMode(NeutralModeValue.Coast).withInverted(kInverted);
        config.Feedback.withSensorToMechanismRatio(1 / Constants.Drive.kSteerReduction);
        config.CurrentLimits.withSupplyCurrentLimit(60).withSupplyCurrentLimitEnable(true);
        config.TorqueCurrent.withPeakForwardTorqueCurrent(40).withPeakReverseTorqueCurrent(-40);
        config.Slot0.withKP(Constants.Drive.kSteerKp.get()).withKD(Constants.Drive.kSteerKd.get());
        config.ClosedLoopGeneral.ContinuousWrap = true;

        configureWithRetry(motor, config);

        motor.setPosition(0);

        return new MotorIOTalonFXFOC(motor).withPositionControlFrequency(0);
    }

    private static void configureWithRetry(TalonFX motor, TalonFXConfiguration config) {
        for (int i = 0; i < 4; i++) {
            if (motor.getConfigurator().apply(config, 0.1) == StatusCode.OK) {
                break;
            }
        }
    }
}
