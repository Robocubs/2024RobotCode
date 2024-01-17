package com.team1701.lib.drivers.motors;

import java.util.Optional;
import java.util.Queue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class MotorIOTalonFX implements MotorIO {
    private final TalonFX mMotor;
    private final double mReduction;
    private final PositionDutyCycle mPositionDutyCycle;
    private final VelocityDutyCycle mVelocityDutyCycle;
    private final Slot0Configs configs;
    private final StatusSignal<Double> mPositionSignal;
    private final StatusSignal<Double> mVelocitySignal;

    private final double kMaxKrakenVoltage = 24;

    private Optional<Queue<Double>> mPositionSamples = Optional.empty();
    private Optional<Queue<Double>> mVelocitySamples = Optional.empty();

    public MotorIOTalonFX(TalonFX motor, double reduction) {
        super();
        mMotor = motor;
        mReduction = reduction;
        mPositionDutyCycle = new PositionDutyCycle(0);
        mVelocityDutyCycle = new VelocityDutyCycle(0);
        configs = new Slot0Configs();
        mPositionSignal = mMotor.getPosition();
        mVelocitySignal = mMotor.getVelocity();
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        inputs.positionRadians = Units.rotationsToRadians(mPositionSignal.getValue()) * mReduction;
        inputs.velocityRadiansPerSecond =
                Units.rotationsPerMinuteToRadiansPerSecond(mVelocitySignal.getValue()) * mReduction;
        mPositionSamples.ifPresent(samples -> {
            inputs.positionRadiansSamples = samples.stream()
                    .mapToDouble((position) -> Units.rotationsToRadians(position))
                    .toArray();
            samples.clear();
        });
        mVelocitySamples.ifPresent(samples -> {
            inputs.positionRadiansSamples = samples.stream()
                    .mapToDouble((velocity) -> Units.rotationsToRadians(velocity))
                    .toArray();
            samples.clear();
        });
    }

    @Override
    public void setPositionControl(Rotation2d position) {
        mMotor.setControl(mPositionDutyCycle.withPosition(position.getRotations()));
    }

    @Override
    public void setVelocityControl(double velocityRadiansPerSecond) {
        mMotor.setControl(mVelocityDutyCycle.withVelocity(Units.radiansToRotations(velocityRadiansPerSecond)));
    }

    /**
     * This method sets the percent of max voltage to output to a Kraken X60.
     * Phoenix 6 has shifted away from arbitrary max voltages and percent outputs,
     * so either DutyCycle or Direct Voltage controls will be a better method of controlling output.
     *
     * This uses 24 volts as the max voltage for a Kraken X60.
     *
     * @param percentage The percent, as a decimal, of voltage to output
     */
    @Override
    public void setPercentOutput(double percentage) {
        mMotor.setVoltage(percentage < 1 ? percentage * kMaxKrakenVoltage : kMaxKrakenVoltage);
    }

    @Override
    public void setVoltageOutput(double volts) {
        mMotor.setVoltage(volts);
    }

    @Override
    public void setBreakMode(boolean enable) {
        mMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPID(double ff, double p, double i, double d) {
        configs.kV = ff;
        configs.kP = p;
        configs.kI = i;
        configs.kD = d;

        mMotor.getConfigurator().apply(configs);
    }

    @Override
    public void enablePositionSampling(SignalSamplingThread samplingThread) {
        if (mPositionSamples.isPresent()) {
            throw new IllegalStateException("Position sampling already enabled");
        }

        var queue = samplingThread.addSignal(
                () -> Units.rotationsToRadians(mMotor.getPosition().getValue()));
        mPositionSamples = Optional.of(queue);
    }

    @Override
    public synchronized void enableVelocitySampling(SignalSamplingThread samplingThread) {
        if (mVelocitySamples.isPresent()) {
            throw new IllegalStateException("Velocity sampling already enabled");
        }

        var queue = samplingThread.addSignal(() ->
                Units.rotationsPerMinuteToRadiansPerSecond(mMotor.getVelocity().getValue()));
        mVelocitySamples = Optional.of(queue);
    }

    public void setSignalUpdateFrequency(double frequencyHz) {
        updatePositionFrequency(frequencyHz);
        updateVelocityFrequency(frequencyHz);
    }

    public void updatePositionFrequency(double frequencyHz) {
        mPositionSignal.setUpdateFrequency(frequencyHz);
    }

    public void updateVelocityFrequency(double frequencyHz) {
        mVelocitySignal.setUpdateFrequency(frequencyHz);
    }
}
