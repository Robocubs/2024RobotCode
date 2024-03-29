package com.team1701.lib.drivers.motors;

import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorIOSim implements MotorIO {
    private final DCMotorSim mSim;
    private final PIDController mController;
    private final double mLoopPeriodSeconds;
    private SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    private double mAppliedVoltage;
    private double mVelocityRadiansPerSecond;
    private double mPositionRadians;
    private boolean mPositionSamplingEnabled;
    private boolean mVelocitySamplingEnabled;
    private int mPositionSamples = 0;
    private int mVelocitySamples = 0;

    public MotorIOSim(DCMotor motor, double reduction, double jKgMetersSquared, double loopPeriodSeconds) {
        mSim = new DCMotorSim(motor, 1.0 / reduction, jKgMetersSquared);
        mController = new PIDController(0.0, 0.0, 0.0, loopPeriodSeconds);
        mLoopPeriodSeconds = loopPeriodSeconds;
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        if (DriverStation.isDisabled()) {
            mSim.setInputVoltage(0);
        }

        mSim.update(mLoopPeriodSeconds);

        inputs.velocityRadiansPerSecond = mSim.getAngularVelocityRadPerSec();
        inputs.positionRadians = mPositionRadians + inputs.velocityRadiansPerSecond * mLoopPeriodSeconds;

        if (mPositionSamplingEnabled) {
            var samples = mPositionSamples;
            inputs.positionRadiansSamples = new double[samples];
            var lerp = (inputs.positionRadians - mPositionRadians) / samples;
            for (int i = 0; i < samples; i++) {
                inputs.positionRadiansSamples[i] = mPositionRadians + lerp * (i + 1);
            }
        }

        if (mVelocitySamplingEnabled) {
            var samples = mVelocitySamples;
            inputs.velocityRadiansPerSecondSamples = new double[samples];
            var lerp = (inputs.velocityRadiansPerSecond - mVelocityRadiansPerSecond) / samples;
            for (int i = 0; i < samples; i++) {
                inputs.velocityRadiansPerSecondSamples[i] = mVelocityRadiansPerSecond + lerp * (i + 1);
            }
        }

        inputs.appliedVoltage = mAppliedVoltage;
        inputs.outputCurrent = mSim.getCurrentDrawAmps();

        mPositionRadians = inputs.positionRadians;
        mVelocityRadiansPerSecond = inputs.velocityRadiansPerSecond;
        mPositionSamples = 0;
        mVelocitySamples = 0;
    }

    @Override
    public void setPositionControl(Rotation2d position) {
        mController.setSetpoint(position.getRadians());
        setVoltageOutput(mController.calculate(mPositionRadians) + mFeedforward.calculate(position.getRadians()));
    }

    @Override
    public void setVelocityControl(double velocityRadiansPerSecond) {
        mController.setSetpoint(velocityRadiansPerSecond);
        setVoltageOutput(
                mController.calculate(mVelocityRadiansPerSecond) + mFeedforward.calculate(velocityRadiansPerSecond));
    }

    @Override
    public void setPercentOutput(double percentage) {
        setVoltageOutput(percentage * 12.0);
    }

    @Override
    public void setVoltageOutput(double volts) {
        mAppliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        mSim.setInputVoltage(mAppliedVoltage);
    }

    @Override
    public void runCharacterization(double input) {
        setVoltageOutput(input);
    }

    @Override
    public void setBrakeMode(boolean enable) {}

    @Override
    public void setFeedforward(double kS, double kV, double kA) {
        mFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void setPID(double p, double i, double d) {
        mController.setPID(p, i, d);
    }

    @Override
    public synchronized void enablePositionSampling(SignalSamplingThread samplingThread) {
        if (mPositionSamplingEnabled) {
            throw new IllegalStateException("Position sampling already enabled");
        }

        samplingThread.addSignal(() -> {
            mPositionSamples++;
            return 0.0; // We will interpolate in updateInputs
        });

        mPositionSamplingEnabled = true;
    }

    @Override
    public synchronized void enableVelocitySampling(SignalSamplingThread samplingThread) {
        if (mVelocitySamplingEnabled) {
            throw new IllegalStateException("Velocity sampling already enabled");
        }

        samplingThread.addSignal(() -> {
            mVelocitySamples++;
            return 0.0; // We will interpolate in updateInputs
        });

        mVelocitySamplingEnabled = true;
    }

    @Override
    public void setPosition(Rotation2d position) {
        mPositionRadians = position.getRadians();
    }

    @Override
    public void stopMotor() {
        setVoltageOutput(0);
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        mController.enableContinuousInput(minimumInput, maximumInput);
    }

    public void disableContinuousInput() {
        mController.disableContinuousInput();
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromRadians(mPositionRadians);
    }

    public double getVelocityRotationsPerSecond() {
        return mVelocityRadiansPerSecond;
    }
}
