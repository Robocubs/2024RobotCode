package com.team1701.lib.drivers.gyros;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroInputs {
        public boolean connected;
        public Rotation2d yaw = GeometryUtil.kRotationIdentity;
        public Rotation2d pitch = GeometryUtil.kRotationIdentity;
        public Rotation2d roll = GeometryUtil.kRotationIdentity;
        public Rotation2d[] yawSamples = new Rotation2d[] {};
    }

    public default void updateInputs(GyroInputs inputs) {}

    public default void enableYawSampling(SignalSamplingThread samplingThread) {}
}
