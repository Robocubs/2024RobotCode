package com.team1701.lib.drivers.motors;

import java.util.Optional;
import java.util.Queue;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class MotorIOSparkFlex implements MotorIO {
    private final CANSparkFlex mMotor;
    private final RelativeEncoder mEncoder;
    private final SparkPIDController mController;
    private final double mReduction;

    private SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    private Optional<Queue<Double>> mPositionSamples = Optional.empty();
    private Optional<Queue<Double>> mVelocitySamples = Optional.empty();

    public MotorIOSparkFlex(CANSparkFlex motor, double reduction) {
        this(motor, reduction, 0, 0);
    }

    public MotorIOSparkFlex(
            CANSparkFlex motor, double reduction, double velocityConstraint, double accelerationConstraint) {
        mMotor = motor;
        mEncoder = motor.getEncoder();
        mController = motor.getPIDController();
        mReduction = reduction;
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        inputs.positionRadians = Units.rotationsToRadians(mEncoder.getPosition()) * mReduction;
        inputs.velocityRadiansPerSecond =
                Units.rotationsPerMinuteToRadiansPerSecond(mEncoder.getVelocity()) * mReduction;
        mPositionSamples.ifPresentOrElse(
                samples -> {
                    inputs.positionRadiansSamples =
                            samples.stream().mapToDouble(this::toReducedRadians).toArray();
                    inputs.positionRadians = inputs.positionRadiansSamples.length > 0
                            ? inputs.positionRadiansSamples[inputs.positionRadiansSamples.length - 1]
                            : toReducedRadians(mEncoder.getPosition());
                    samples.clear();
                },
                () -> {
                    inputs.positionRadians = toReducedRadians(mEncoder.getPosition());
                    inputs.positionRadiansSamples = kEmptySamples;
                });
        mVelocitySamples.ifPresentOrElse(
                samples -> {
                    inputs.velocityRadiansPerSecondSamples = samples.stream()
                            .mapToDouble(this::toReducedRadiansPerSecond)
                            .toArray();
                    inputs.velocityRadiansPerSecond = inputs.velocityRadiansPerSecondSamples.length > 0
                            ? inputs.velocityRadiansPerSecondSamples[inputs.velocityRadiansPerSecondSamples.length - 1]
                            : toReducedRadiansPerSecond(mEncoder.getVelocity());
                    samples.clear();
                },
                () -> {
                    inputs.velocityRadiansPerSecond = toReducedRadiansPerSecond(mEncoder.getVelocity());
                    inputs.velocityRadiansPerSecondSamples = kEmptySamples;
                });
        inputs.appliedVoltage = mMotor.getBusVoltage() * mMotor.getAppliedOutput();
        inputs.outputCurrent = mMotor.getOutputCurrent();
    }

    private double toReducedRadians(double value) {
        return Units.rotationsToRadians(value) * mReduction;
    }

    private double toReducedRadiansPerSecond(double value) {
        return Units.rotationsPerMinuteToRadiansPerSecond(value) * mReduction;
    }

    @Override
    public void setPositionControl(Rotation2d position) {
        mController.setReference(
                position.getRotations() / mReduction,
                CANSparkFlex.ControlType.kPosition,
                0,
                mFeedforward.calculate(position.getRadians() / mReduction));
    }

    @Override
    public void setVelocityControl(double velocityRadiansPerSecond) {
        mController.setReference(
                Units.radiansPerSecondToRotationsPerMinute(velocityRadiansPerSecond / mReduction),
                CANSparkFlex.ControlType.kVelocity,
                0,
                mFeedforward.calculate(velocityRadiansPerSecond / mReduction));
    }

    @Override
    public void setPercentOutput(double percentage) {
        mController.setReference(percentage, CANSparkFlex.ControlType.kDutyCycle);
    }

    @Override
    public void setVoltageOutput(double volts) {
        mController.setReference(volts, CANSparkFlex.ControlType.kVoltage);
    }

    @Override
    public void runCharacterization(double input) {
        setVoltageOutput(input);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        mMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setFeedforward(double kS, double kV, double kA) {
        mFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void setPID(double p, double i, double d) {
        mController.setP(p);
        mController.setI(i);
        mController.setD(d);
    }

    @Override
    public synchronized void enablePositionSampling(SignalSamplingThread samplingThread) {
        if (mPositionSamples.isPresent()) {
            throw new IllegalStateException("Position sampling already enabled");
        }

        var queue = samplingThread.addSignal(mEncoder::getPosition);
        mPositionSamples = Optional.of(queue);
    }

    @Override
    public synchronized void enableVelocitySampling(SignalSamplingThread samplingThread) {
        if (mVelocitySamples.isPresent()) {
            throw new IllegalStateException("Velocity sampling already enabled");
        }

        var queue = samplingThread.addSignal(mEncoder::getVelocity);
        mVelocitySamples = Optional.of(queue);
    }

    @Override
    public void setPosition(Rotation2d position) {
        mEncoder.setPosition(position.getRotations() / mReduction);
    }

    public void stopMotor() {
        mMotor.stopMotor();
    }
}
