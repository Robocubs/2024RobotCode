package com.team1701.lib.drivers.gyros;

import java.util.function.Supplier;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    private Supplier<Rotation2d> mYawSupplier;
    private boolean mYawSamplingEnabled;

    public GyroIOSim() {
        mYawSupplier = () -> GeometryUtil.kRotationIdentity;
    }

    public GyroIOSim(Supplier<Rotation2d> yawSupplier) {
        mYawSupplier = yawSupplier;
    }

    public GyroIOSim(Supplier<Double> yawVelocityRadiansPerSecondSupplier, double loopPeriodSeconds) {
        setYawSupplier(yawVelocityRadiansPerSecondSupplier, loopPeriodSeconds);
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.connected = true;
        inputs.yaw = mYawSupplier.get();
        if (mYawSamplingEnabled) {
            inputs.yawSamples = new Rotation2d[] {inputs.yaw};
        }
    }

    @Override
    public synchronized void enableYawSampling(SignalSamplingThread samplingThread) {
        if (mYawSamplingEnabled) {
            throw new IllegalStateException("Yaw sampling already enabled");
        }

        mYawSamplingEnabled = true;
    }

    public void setYawSupplier(Supplier<Rotation2d> yawSupplier) {
        mYawSupplier = yawSupplier;
    }

    public void setYawSupplier(Supplier<Double> yawVelocityRadiansPerSecondSupplier, double loopPeriodSeconds) {
        mYawSupplier = new Supplier<Rotation2d>() {
            private double yawRadians;

            @Override
            public Rotation2d get() {
                yawRadians += yawVelocityRadiansPerSecondSupplier.get() * loopPeriodSeconds;
                return Rotation2d.fromRadians(yawRadians);
            }
        };
    }
}
