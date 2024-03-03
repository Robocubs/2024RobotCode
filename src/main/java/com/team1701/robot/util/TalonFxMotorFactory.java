package com.team1701.robot.util;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1701.lib.drivers.motors.MotorIOTalonFX;
import com.team1701.robot.Constants;

public class TalonFxMotorFactory {
    public static MotorIOTalonFX createDriveMotorIOTalonFx(int deviceId) {

        var motor = new TalonFX(deviceId, "canivore1");

        var feedbackConfig = new FeedbackConfigs().withSensorToMechanismRatio(1 / Constants.Drive.kDriveReduction);
        var currentConfig =
                new CurrentLimitsConfigs().withSupplyCurrentLimit(80).withSupplyCurrentLimitEnable(true);
        var config = new TalonFXConfiguration().withFeedback(feedbackConfig).withCurrentLimits(currentConfig);

        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);

        motor.setPosition(0);

        var motorIO = new MotorIOTalonFX(motor);

        motorIO.setPID(
                Constants.Drive.kDriveKff.get(), Constants.Drive.kDriveKp.get(), 0, Constants.Drive.kDriveKd.get());

        motor.setInverted(Constants.Drive.kDriveMotorsInverted);

        return motorIO;
    }

    public static MotorIOTalonFX createSteerMotorIOTalonFx(int deviceId) {

        var motor = new TalonFX(deviceId, "canivore1");

        var feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(1 / Constants.Drive.kSteerReduction);
        var currentConfig =
                new CurrentLimitsConfigs().withSupplyCurrentLimit(60).withSupplyCurrentLimitEnable(true);
        var config = new TalonFXConfiguration().withFeedback(feedbackConfigs).withCurrentLimits(currentConfig);
        config.ClosedLoopGeneral.ContinuousWrap = true;

        motor.getConfigurator().apply(config);

        motor.setPosition(0);

        var motorIO = new MotorIOTalonFX(motor);

        motorIO.setPID(0, Constants.Drive.kSteerKp.get(), 0, Constants.Drive.kSteerKd.get());

        motor.setInverted(Constants.Drive.kSteerMotorsInverted);

        return motorIO;
    }
}
