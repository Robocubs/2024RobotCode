package com.team1701.robot.util;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team1701.lib.drivers.motors.MotorIOTalonFX;
import com.team1701.robot.Constants;

public class TalonFxMotorFactory {
    public static final Configuration kMainConfigs = new Configuration();

    public static class Configuration extends TalonFXConfiguration {

        public boolean INVERTED = false;
        public boolean SENSOR_PHASE = false;

        public int CONTROL_FRAME_PERIOD_MS = 10;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        public int GENERAL_STATUS_FRAME_RATE_MS = 10;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    public static MotorIOTalonFX createDriveMotorIOTalonFx(int deviceId) {

        var motor = new TalonFX(deviceId, "canivore1");

        motor.getConfigurator().apply(kMainConfigs);

        motor.setPosition(0);

        var motorIO = new MotorIOTalonFX(motor, Constants.Drive.kDriveReduction);

        motorIO.setPID(
                Constants.Drive.kDriveKff.get(), Constants.Drive.kDriveKp.get(), 0, Constants.Drive.kDriveKd.get());

        motor.setInverted(Constants.Drive.kDriveMotorsInverted);

        return motorIO;
    }

    public static MotorIOTalonFX createSteerMotorIOTalonFx(int deviceId) {

        var motor = new TalonFX(deviceId, "canivore1");

        var rotationConfig = new ClosedLoopGeneralConfigs();
        var reductionConfigs = new FeedbackConfigs();

        reductionConfigs.SensorToMechanismRatio = Constants.Drive.kSteerReduction;
        rotationConfig.ContinuousWrap = true;

        motor.getConfigurator().apply(kMainConfigs);
        motor.getConfigurator().apply(rotationConfig);
        motor.getConfigurator().apply(reductionConfigs);

        motor.setPosition(0);

        var motorIO = new MotorIOTalonFX(motor, Constants.Drive.kSteerReduction);

        motorIO.setPID(0, Constants.Drive.kSteerKp.get(), 0, Constants.Drive.kSteerKd.get());

        motor.setInverted(Constants.Drive.kSteerMotorsInverted);

        return motorIO;
    }
}
