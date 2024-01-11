package com.team1701.lib.drivers.gyros;

import java.util.Optional;
import java.util.Queue;
import java.util.function.Consumer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 mPigeon;
    private final StatusSignal<Double> mYawSignal;
    private final StatusSignal<Double> mPitchSignal;
    private final StatusSignal<Double> mRollSignal;

    private Optional<Queue<Double>> mYawDegreesSamples = Optional.empty();

    public GyroIOPigeon2(int pigeonID) {
        mPigeon = new Pigeon2(pigeonID);
        mYawSignal = mPigeon.getYaw();
        mPitchSignal = mPigeon.getPitch();
        mRollSignal = mPigeon.getRoll();
    }

    public GyroIOPigeon2(int pigeonID, Consumer<Pigeon2Configurator> configure) {
        this(pigeonID);
    }

    public void updateInputs(GyroInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(mYawSignal, mPitchSignal, mRollSignal)
                .equals(StatusCode.OK);
        inputs.yaw = Rotation2d.fromDegrees(mYawSignal.getValue());
        inputs.pitch = Rotation2d.fromDegrees(mPitchSignal.getValue());
        inputs.roll = Rotation2d.fromDegrees(mRollSignal.getValue());

        mYawDegreesSamples.ifPresent(samples -> {
            inputs.yawSamples = samples.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
            samples.clear();
        });
    }

    @Override
    public synchronized void enableYawSampling(SignalSamplingThread samplingThread) {
        if (mYawDegreesSamples.isPresent()) {
            throw new IllegalStateException("Yaw sampling already enabled");
        }

        mYawSignal.setUpdateFrequency(samplingThread.getFrequency());
        mYawDegreesSamples = Optional.of(samplingThread.addSignal(mYawSignal));
    }
}
