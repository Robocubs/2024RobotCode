package com.team1701.lib.drivers.motors;

import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
    static final double[] kEmptySamples = new double[] {};

    @AutoLog
    public static class MotorInputs {
        public double positionRadians;
        public double velocityRadiansPerSecond;
        public double[] positionRadiansSamples = kEmptySamples;
        public double[] velocityRadiansPerSecondSamples = kEmptySamples;
        public double appliedVoltage;
        public double outputCurrent;
    }

    public default void updateInputs(MotorInputs inputs) {}

    public default void setPositionControl(Rotation2d position) {}

    public default void setVelocityControl(double velocityRadiansPerSecond) {}

    public default void setPercentOutput(double percentage) {}

    public default void setVoltageOutput(double volts) {}

    public default void runCharacterization(double input) {}

    public default void setBrakeMode(boolean enable) {}

    public default void setFeedforward(double kS, double kV, double kA) {}

    public default void setPID(double p, double i, double d) {}

    public default void enablePositionSampling(SignalSamplingThread samplingThread) {}

    public default void enableVelocitySampling(SignalSamplingThread samplingThread) {}

    public default void setPosition(Rotation2d position) {}

    public default void stopMotor() {}
}
