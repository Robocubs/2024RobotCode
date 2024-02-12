package com.team1701.lib.drivers.motors;

import java.util.Optional;
import java.util.Queue;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class MotorIOSparkMax implements MotorIO {
    private final CANSparkMax mMotor;
    private final RelativeEncoder mEncoder;
    private final SparkPIDController mController;
    private final double mReduction;

    private Optional<Queue<Double>> mPositionSamples = Optional.empty();
    private Optional<Queue<Double>> mVelocitySamples = Optional.empty();

    public MotorIOSparkMax(CANSparkMax motor, double reduction) {
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
                () -> inputs.positionRadians = toReducedRadians(mEncoder.getPosition()));
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
                () -> inputs.velocityRadiansPerSecond = toReducedRadiansPerSecond(mEncoder.getVelocity()));
    }

    private double toReducedRadians(double value) {
        return Units.rotationsToRadians(value) * mReduction;
    }

    private double toReducedRadiansPerSecond(double value) {
        return Units.rotationsPerMinuteToRadiansPerSecond(value) * mReduction;
    }

    @Override
    public void setPositionControl(Rotation2d position) {
        mController.setReference(position.getRotations(), CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setVelocityControl(double velocityRadiansPerSecond) {
        mController.setReference(
                Units.radiansPerSecondToRotationsPerMinute(velocityRadiansPerSecond),
                CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setPercentOutput(double percentage) {
        mController.setReference(percentage, CANSparkMax.ControlType.kDutyCycle);
    }

    @Override
    public void setVoltageOutput(double volts) {
        mController.setReference(volts, CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        mMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setPID(double ff, double p, double i, double d) {
        mController.setFF(ff);
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
}
