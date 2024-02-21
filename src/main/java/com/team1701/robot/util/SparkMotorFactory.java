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

    public static class SparkMotorConfiguration {
        private final int deviceId;
        private double reduction = 1.0;
        private int idToFollow = 0;
        private int smartCurrentLimit = 40;
        private int voltageCompensation = 12;
        private int position = 0;
        private int measurementPeriod = 10;
        private int averageDepth = 2;
        private double p = 0;
        private double d = 0;
        private double ff = 0;
        private float softLimitForward = 0;
        private float softLimitReverse = 0;
        private boolean positionPIDWrapping = false;
        private double positionPIDWrappingMinInput = 0;
        private double positionPIDWrappingMaxInput = 0;
        private boolean inverted = false;

        public SparkMotorConfiguration(int deviceId) {
            this.deviceId = deviceId;
        }

        public SparkMotorConfiguration reduction(double value) {
            reduction = value;
            return this;
        }

        public SparkMotorConfiguration idToFollow(int value) {
            idToFollow = value;
            return this;
        }

        public SparkMotorConfiguration smartCurrentLimit(int value) {
            smartCurrentLimit = value;
            return this;
        }

        public SparkMotorConfiguration voltageCompensation(int value) {
            voltageCompensation = value;
            return this;
        }

        public SparkMotorConfiguration position(int value) {
            position = value;
            return this;
        }

        public SparkMotorConfiguration measurementPeriod(int value) {
            measurementPeriod = value;
            return this;
        }

        public SparkMotorConfiguration averageDepth(int value) {
            averageDepth = value;
            return this;
        }

        public SparkMotorConfiguration p(double value) {
            p = value;
            return this;
        }

        public SparkMotorConfiguration d(double value) {
            d = value;
            return this;
        }

        public SparkMotorConfiguration ff(double value) {
            ff = value;
            return this;
        }

        public SparkMotorConfiguration softLimitForward(float value) {
            softLimitForward = value;
            return this;
        }

        public SparkMotorConfiguration softLimitReverse(float value) {
            softLimitReverse = value;
            return this;
        }

        public SparkMotorConfiguration positionPIDWrapping(boolean value) {
            positionPIDWrapping = value;
            return this;
        }

        public SparkMotorConfiguration positionPIDWrappingMinInput(double value) {
            positionPIDWrappingMinInput = value;
            return this;
        }

        public SparkMotorConfiguration positionPIDWrappingMaxInput(double value) {
            positionPIDWrappingMaxInput = value;
            return this;
        }

        public SparkMotorConfiguration inverted(boolean value) {
            inverted = value;
            return this;
        }
    }

    public static MotorIOSparkFlex createMotorSparkFlex(SparkMotorConfiguration configuration) {
        var motor = new CANSparkFlex(configuration.deviceId, MotorType.kBrushless);
        var encoder = motor.getEncoder();
        var controller = motor.getPIDController();
        var errorAlert = new REVAlert(motor, configuration.deviceId);

        motor.setCANTimeout(200);

        configureWithRetry(() -> motor.restoreFactoryDefaults(), errorAlert);

        configureWithRetry(() -> motor.setSmartCurrentLimit(configuration.smartCurrentLimit), errorAlert);
        configureWithRetry(() -> motor.enableVoltageCompensation(configuration.voltageCompensation), errorAlert);

        configureWithRetry(() -> encoder.setPosition(configuration.position), errorAlert);
        configureWithRetry(() -> encoder.setMeasurementPeriod(configuration.measurementPeriod), errorAlert);
        configureWithRetry(() -> encoder.setAverageDepth(configuration.averageDepth), errorAlert);

        configureWithRetry(() -> controller.setP(configuration.p), errorAlert);
        configureWithRetry(() -> controller.setD(configuration.d), errorAlert);
        configureWithRetry(() -> controller.setFF(configuration.ff), errorAlert);

        configureWithRetry(
                () -> motor.setSoftLimit(SoftLimitDirection.kForward, configuration.softLimitForward), errorAlert);
        configureWithRetry(
                () -> motor.setSoftLimit(SoftLimitDirection.kReverse, configuration.softLimitReverse), errorAlert);

        configureWithRetry(
                () -> controller.setPositionPIDWrappingEnabled(configuration.positionPIDWrapping), errorAlert);
        configureWithRetry(
                () -> controller.setPositionPIDWrappingMinInput(configuration.positionPIDWrappingMinInput), errorAlert);
        configureWithRetry(
                () -> controller.setPositionPIDWrappingMaxInput(configuration.positionPIDWrappingMaxInput), errorAlert);

        motor.follow(ExternalFollower.kFollowerSpark, configuration.idToFollow, configuration.inverted);

        configureWithRetry(() -> motor.burnFlash(), errorAlert);

        motor.setInverted(configuration.inverted);

        motor.setCANTimeout(0);

        return new MotorIOSparkFlex(motor, configuration.reduction);
    }

    public static MotorIOSparkMax createMotorSparkMax(SparkMotorConfiguration configuration) {
        var motor = new CANSparkMax(configuration.deviceId, MotorType.kBrushless);
        var encoder = motor.getEncoder();
        var controller = motor.getPIDController();
        var errorAlert = new REVAlert(motor, configuration.deviceId);

        motor.setCANTimeout(200);

        configureWithRetry(() -> motor.restoreFactoryDefaults(), errorAlert);

        configureWithRetry(() -> motor.setSmartCurrentLimit(configuration.smartCurrentLimit), errorAlert);
        configureWithRetry(() -> motor.enableVoltageCompensation(configuration.voltageCompensation), errorAlert);

        configureWithRetry(() -> encoder.setPosition(configuration.position), errorAlert);
        configureWithRetry(() -> encoder.setMeasurementPeriod(configuration.measurementPeriod), errorAlert);
        configureWithRetry(() -> encoder.setAverageDepth(configuration.averageDepth), errorAlert);

        configureWithRetry(() -> controller.setP(configuration.p), errorAlert);
        configureWithRetry(() -> controller.setD(configuration.d), errorAlert);
        configureWithRetry(() -> controller.setFF(configuration.ff), errorAlert);

        configureWithRetry(
                () -> motor.setSoftLimit(SoftLimitDirection.kForward, configuration.softLimitForward), errorAlert);
        configureWithRetry(
                () -> motor.setSoftLimit(SoftLimitDirection.kReverse, configuration.softLimitReverse), errorAlert);

        configureWithRetry(
                () -> controller.setPositionPIDWrappingEnabled(configuration.positionPIDWrapping), errorAlert);
        configureWithRetry(
                () -> controller.setPositionPIDWrappingMinInput(configuration.positionPIDWrappingMinInput), errorAlert);
        configureWithRetry(
                () -> controller.setPositionPIDWrappingMaxInput(configuration.positionPIDWrappingMaxInput), errorAlert);

        motor.follow(ExternalFollower.kFollowerSpark, configuration.idToFollow, configuration.inverted);

        configureWithRetry(() -> motor.burnFlash(), errorAlert);

        motor.setInverted(configuration.inverted);

        motor.setCANTimeout(0);

        return new MotorIOSparkMax(motor, configuration.reduction);
    }

    public static MotorIOSparkFlex createShooterRollerSparkFlex(int deviceId, boolean inverted) {
        return createMotorSparkFlex(new SparkMotorConfiguration(deviceId)
                .smartCurrentLimit(80)
                .p(Constants.Shooter.kRollerKp.get())
                .d(Constants.Shooter.kRollerKd.get())
                .ff(Constants.Shooter.kRollerKff.get())
                .reduction(Constants.Shooter.kRollerReduction)
                .inverted(inverted));
    }

    public static MotorIOSparkFlex createShooterRotationSparkFlex(int deviceId, boolean inverted) {
        return createMotorSparkFlex(new SparkMotorConfiguration(deviceId)
                .smartCurrentLimit(80)
                .p(Constants.Shooter.kRotationKp.get())
                .d(Constants.Shooter.kRotationKd.get())
                .ff(Constants.Shooter.kRotationKff.get())
                .softLimitForward((float) Constants.Shooter.kShooterUpperLimitRotations)
                .softLimitReverse((float) Constants.Shooter.kShooterLowerLimitRotations)
                .reduction(Constants.Shooter.kAngleReduction)
                .inverted(inverted));
    }
    // TODO: Update values for actual shooter
    public static MotorIOSparkFlex createShooterRollerSparkFlex(int deviceId, boolean inverted, int idToFollow) {
        return createMotorSparkFlex(new SparkMotorConfiguration(deviceId)
                .smartCurrentLimit(80)
                .p(Constants.Shooter.kRollerKp.get())
                .d(Constants.Shooter.kRollerKd.get())
                .ff(Constants.Shooter.kRollerKff.get())
                .reduction(Constants.Shooter.kRollerReduction)
                .inverted(inverted)
                .idToFollow(idToFollow));
    }
    // TODO: Update values for actual shooter
    public static MotorIOSparkFlex createShooterRotationSparkFlex(int deviceId, boolean inverted, int idToFollow) {
        return createMotorSparkFlex(new SparkMotorConfiguration(deviceId)
                .smartCurrentLimit(80)
                .p(Constants.Shooter.kRotationKp.get())
                .d(Constants.Shooter.kRotationKd.get())
                .ff(Constants.Shooter.kRotationKff.get())
                .reduction(Constants.Shooter.kAngleReduction)
                .inverted(inverted)
                .idToFollow(idToFollow));
    }

    public static MotorIOSparkFlex createIntakeMotorIOSparkFlex(int deviceId) {
        return createMotorSparkFlex(new SparkMotorConfiguration(deviceId)
                .smartCurrentLimit(20)
                .reduction(Constants.Intake.kReduction));
    }

    public static MotorIOSparkMax createDriveMotorIOSparkMax(int deviceId) {
        return createMotorSparkMax(new SparkMotorConfiguration(deviceId)
                .smartCurrentLimit(80)
                .p(Constants.Drive.kDriveKp.get())
                .d(Constants.Drive.kDriveKd.get())
                .ff(Constants.Drive.kDriveKff.get())
                .inverted(Constants.Drive.kDriveMotorsInverted)
                .reduction(Constants.Drive.kDriveReduction));
    }

    public static MotorIOSparkMax createSteerMotorIOSparkMax(int deviceId) {
        return createMotorSparkMax(new SparkMotorConfiguration(deviceId)
                .smartCurrentLimit(30)
                .p(Constants.Drive.kDriveKp.get())
                .d(Constants.Drive.kDriveKd.get())
                .positionPIDWrapping(true)
                .positionPIDWrappingMinInput(0)
                .positionPIDWrappingMaxInput(1 / Constants.Drive.kSteerReduction)
                .inverted(Constants.Drive.kDriveMotorsInverted)
                .reduction(Constants.Drive.kDriveReduction));
    }

    public static MotorIOSparkFlex createIndexerMotorIOSparkFlex(int deviceId) {
        return createMotorSparkFlex(new SparkMotorConfiguration(deviceId));
    }

    // public static MotorIOSparkFlex createShooterMotorIOSparkFlex(
    //         int deviceId, ShooterMotorUsage motorUse, boolean inverted) {
    // var motor = new CANSparkFlex(deviceId, MotorType.kBrushless);
    // var encoder = motor.getEncoder();
    // var controller = motor.getPIDController();
    // var errorAlert = new REVAlert(motor, deviceId);

    // motor.setCANTimeout(200);

    // // TODO: Update values for actual shooter
    // configureWithRetry(() -> motor.restoreFactoryDefaults(), errorAlert);

    // configureWithRetry(() -> motor.setSmartCurrentLimit(80), errorAlert);
    // configureWithRetry(() -> motor.enableVoltageCompensation(12), errorAlert);

    // configureWithRetry(() -> encoder.setPosition(0), errorAlert);
    // configureWithRetry(() -> encoder.setMeasurementPeriod(10), errorAlert);
    // configureWithRetry(() -> encoder.setAverageDepth(2), errorAlert);

    // double reduction = 1.0;
    // switch (motorUse) {
    //     case ROLLER:
    //         configureWithRetry(() -> controller.setP(Constants.Shooter.kRollerKp.get()), errorAlert);
    //         configureWithRetry(() -> controller.setD(Constants.Shooter.kRollerKd.get()), errorAlert);
    //         configureWithRetry(() -> controller.setFF(Constants.Shooter.kRollerKff.get()), errorAlert);
    //         reduction = Constants.Shooter.kRollerReduction;
    //         break;
    //     case ROTATION:
    //         configureWithRetry(() -> controller.setP(Constants.Shooter.kRotationKp.get()), errorAlert);
    //         configureWithRetry(() -> controller.setD(Constants.Shooter.kRotationKd.get()), errorAlert);
    //         configureWithRetry(() -> controller.setFF(Constants.Shooter.kRotationKff.get()), errorAlert);
    //         configureWithRetry(
    //                 () -> motor.setSoftLimit(
    //                         SoftLimitDirection.kForward, (float) Constants.Shooter.kShooterUpperLimitRotations),
    //                 errorAlert);
    //         configureWithRetry(
    //                 () -> motor.setSoftLimit(
    //                         SoftLimitDirection.kReverse, (float) Constants.Shooter.kShooterLowerLimitRotations),
    //                 errorAlert);
    //         reduction = Constants.Shooter.kAngleReduction;
    //         break;
    //     default:
    //         break;
    // }

    // configureWithRetry(() -> motor.burnFlash(), errorAlert);

    // motor.setCANTimeout(0);

    // return new MotorIOSparkFlex(motor, reduction);
    //  }
    /*
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
    */
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
