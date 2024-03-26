package com.team1701.lib.drivers.motors;

import java.util.Optional;
import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class MotorIOTalonFXFOC implements MotorIO {
    private final TalonFX mMotor;
    private final PositionTorqueCurrentFOC mPositionTorqueCurrentFOC;
    private final VelocityTorqueCurrentFOC mVelocityTorqueCurrentFOC;
    private final DutyCycleOut mDutyCycleOut;
    private final VoltageOut mVoltageOut;
    private final TorqueCurrentFOC mTorqueCurrentFOC;
    private final StatusSignal<Double> mPositionSignal;
    private final StatusSignal<Double> mVelocitySignal;
    private final StatusSignal<Double> mMotorVoltageSignal;
    private final StatusSignal<Double> mTorqueCurrentSignal;
    private final Slot0Configs mSlot0Configs;

    private Optional<Queue<Double>> mPositionSamples = Optional.empty();
    private Optional<Queue<Double>> mVelocitySamples = Optional.empty();

    public MotorIOTalonFXFOC(TalonFX motor) {
        mMotor = motor;
        mPositionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);
        mVelocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        mDutyCycleOut = new DutyCycleOut(0);
        mVoltageOut = new VoltageOut(0);
        mTorqueCurrentFOC = new TorqueCurrentFOC(0);
        mPositionSignal = mMotor.getPosition();
        mVelocitySignal = mMotor.getVelocity();
        mMotorVoltageSignal = mMotor.getMotorVoltage();
        mTorqueCurrentSignal = mMotor.getTorqueCurrent();
        mSlot0Configs = new Slot0Configs();
        mMotor.getConfigurator().refresh(mSlot0Configs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100, mPositionSignal, mVelocitySignal, mMotorVoltageSignal, mTorqueCurrentSignal);
        mMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        BaseStatusSignal.refreshAll(mPositionSignal, mVelocitySignal, mMotorVoltageSignal, mTorqueCurrentSignal);

        mPositionSamples.ifPresentOrElse(
                samples -> {
                    inputs.positionRadiansSamples = samples.stream()
                            .mapToDouble(this::encoderUnitsToReducedUnits)
                            .toArray();
                    samples.clear();
                },
                () -> inputs.positionRadiansSamples = kEmptySamples);

        inputs.positionRadians = encoderUnitsToReducedUnits(mPositionSignal.getValue());

        mVelocitySamples.ifPresentOrElse(
                samples -> {
                    inputs.velocityRadiansPerSecondSamples = samples.stream()
                            .mapToDouble(this::encoderUnitsToReducedUnits)
                            .toArray();
                    samples.clear();
                },
                () -> inputs.velocityRadiansPerSecondSamples = kEmptySamples);

        inputs.velocityRadiansPerSecond = encoderUnitsToReducedUnits(mVelocitySignal.getValue());

        inputs.appliedVoltage = mMotorVoltageSignal.getValue();
        inputs.outputCurrent = mTorqueCurrentSignal.getValue();
    }

    private double encoderUnitsToReducedUnits(double encoderUnits) {
        return Units.rotationsToRadians(encoderUnits);
    }

    @Override
    public void setPositionControl(Rotation2d position) {
        mMotor.setControl(mPositionTorqueCurrentFOC.withPosition(position.getRotations()));
    }

    @Override
    public void setVelocityControl(double velocityRadiansPerSecond) {
        mMotor.setControl(mVelocityTorqueCurrentFOC.withVelocity(Units.radiansToRotations(velocityRadiansPerSecond)));
    }

    @Override
    public void setPercentOutput(double percentage) {
        mMotor.setControl(mDutyCycleOut.withOutput(percentage));
    }

    @Override
    public void setVoltageOutput(double volts) {
        mMotor.setControl(mVoltageOut.withOutput(volts));
    }

    @Override
    public void runCharacterization(double input) {
        mMotor.setControl(mTorqueCurrentFOC.withOutput(input));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        mMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setFeedforward(double kS, double kV, double kA) {
        mSlot0Configs.kS = kS;
        mSlot0Configs.kV = kV;
        mSlot0Configs.kA = kA;

        mMotor.getConfigurator().apply(mSlot0Configs);
    }

    @Override
    public void setPID(double p, double i, double d) {
        mSlot0Configs.kP = p;
        mSlot0Configs.kI = i;
        mSlot0Configs.kD = d;

        mMotor.getConfigurator().apply(mSlot0Configs);
    }

    @Override
    public void enablePositionSampling(SignalSamplingThread samplingThread) {
        if (mPositionSamples.isPresent()) {
            throw new IllegalStateException("Position sampling already enabled");
        }

        mPositionSignal.setUpdateFrequency(samplingThread.getFrequency());
        var queue = samplingThread.addSignal(mMotor, mPositionSignal);
        mPositionSamples = Optional.of(queue);
    }

    @Override
    public synchronized void enableVelocitySampling(SignalSamplingThread samplingThread) {
        if (mVelocitySamples.isPresent()) {
            throw new IllegalStateException("Velocity sampling already enabled");
        }

        mVelocitySignal.setUpdateFrequency(samplingThread.getFrequency());
        var queue = samplingThread.addSignal(mMotor, mVelocitySignal);
        mVelocitySamples = Optional.of(queue);
    }

    public MotorIOTalonFXFOC withPositionControlFrequency(double frequency) {
        mPositionTorqueCurrentFOC.withUpdateFreqHz(frequency);
        return this;
    }

    public MotorIOTalonFXFOC withVelocityControlFrequency(double frequency) {
        mVelocityTorqueCurrentFOC.withUpdateFreqHz(frequency);
        return this;
    }
}
