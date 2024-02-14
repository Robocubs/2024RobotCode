package com.team1701.lib.drivers.motors;

import java.util.Optional;
import java.util.Queue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
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
    private final DutyCycleOut mDutyCycleOut;
    private final VoltageOut mVoltageOut;
    private final StatusSignal<Double> mPositionSignal;
    private final StatusSignal<Double> mVelocitySignal;

    private Optional<Queue<Double>> mPositionSamples = Optional.empty();
    private Optional<Queue<Double>> mVelocitySamples = Optional.empty();

    public MotorIOTalonFX(TalonFX motor, double reduction) {
        super();
        mMotor = motor;
        mReduction = reduction;
        mPositionDutyCycle = new PositionDutyCycle(0);
        mVelocityDutyCycle = new VelocityDutyCycle(0);
        mDutyCycleOut = new DutyCycleOut(0);
        mVoltageOut = new VoltageOut(0);
        mPositionSignal = mMotor.getPosition();
        mVelocitySignal = mMotor.getVelocity();
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        mPositionSamples.ifPresentOrElse(
                samples -> {
                    inputs.positionRadiansSamples = samples.stream()
                            .mapToDouble(this::encoderUnitsToReducedUnits)
                            .toArray();
                    samples.clear();
                },
                () -> {
                    mPositionSignal.refresh();
                    inputs.positionRadiansSamples = new double[] {};
                });
        mPositionSignal.refresh();
        inputs.positionRadians = encoderUnitsToReducedUnits(mPositionSignal.getValue());

        mVelocitySamples.ifPresentOrElse(
                samples -> {
                    inputs.velocityRadiansPerSecondSamples = samples.stream()
                            .mapToDouble(this::encoderUnitsToReducedUnits)
                            .toArray();
                    samples.clear();
                },
                () -> {
                    mVelocitySignal.refresh();
                    inputs.velocityRadiansPerSecondSamples = new double[] {};
                });

        inputs.velocityRadiansPerSecond = encoderUnitsToReducedUnits(mVelocitySignal.getValue());
    }

    private double encoderUnitsToReducedUnits(double encoderUnits) {
        return Units.rotationsToRadians(encoderUnits) * mReduction;
    }

    @Override
    public void setPositionControl(Rotation2d position) {
        mMotor.setControl(mPositionDutyCycle.withPosition(position.getRotations() / mReduction));
    }

    @Override
    public void setVelocityControl(double velocityRadiansPerSecond) {
        mMotor.setControl(
                mVelocityDutyCycle.withVelocity(Units.radiansToRotations(velocityRadiansPerSecond) / mReduction));
    }

    /**
     * @param percentage Percent between -1 & 1
     */
    @Override
    public void setPercentOutput(double percentage) {
        mMotor.setControl(mDutyCycleOut.withOutput(percentage));
    }

    @Override
    public void setVoltageOutput(double volts) {
        mMotor.setControl(mVoltageOut.withOutput(volts));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        mMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPID(double ff, double p, double i, double d) {
        var configs = new Slot0Configs();
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

        var queue = samplingThread.addSignal(mMotor, mPositionSignal);
        mPositionSignal.setUpdateFrequency(samplingThread.getFrequency());
        mPositionSamples = Optional.of(queue);
    }

    @Override
    public synchronized void enableVelocitySampling(SignalSamplingThread samplingThread) {
        if (mVelocitySamples.isPresent()) {
            throw new IllegalStateException("Velocity sampling already enabled");
        }

        var queue = samplingThread.addSignal(mMotor, mVelocitySignal);
        mVelocitySignal.setUpdateFrequency(samplingThread.getFrequency());
        mVelocitySamples = Optional.of(queue);
    }
}
