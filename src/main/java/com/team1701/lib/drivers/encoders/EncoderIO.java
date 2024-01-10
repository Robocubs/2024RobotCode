package com.team1701.lib.drivers.encoders;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {
    @AutoLog
    public static class EncoderInputs {
        public Rotation2d position = GeometryUtil.kRotationIdentity;
    }

    public default void updateInputs(EncoderInputs inputs) {}

    public default void enableYawSampling(SignalSamplingThread samplingThread) {}
}
