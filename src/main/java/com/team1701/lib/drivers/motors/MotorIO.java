package com.team1701.lib.drivers.motors;

import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
    @AutoLog
    public static class MotorInputs {
        public double positionRadians;
        public double velocityRadiansPerSecond;
        public double[] positionRadiansSamples = new double[] {};
        public double[] velocityRadiansPerSecondSamples = new double[] {};
    }

    public default void updateInputs(MotorInputs inputs) {}

    public default void setPositionControl(Rotation2d position) {}

    public default void setSmoothPositionControl(
            Rotation2d position, double maxVelocityRadiansPerSecond, double maxAccelerationRadiansPerSecond) {}

    public default void setVelocityControl(double velocityRadiansPerSecond) {}

    public default void setSmoothVelocityControl(
            double velocityRadiansPerSecond, double maxAccelerationRadiansPerSecondSquared) {}

    public default void setPercentOutput(double percentage) {}

    public default void setVoltageOutput(double volts) {}

    public default void setBrakeMode(boolean enable) {}

    public default void setPID(double ff, double p, double i, double d) {}

    public default void enablePositionSampling(SignalSamplingThread samplingThread) {}

    public default void enableVelocitySampling(SignalSamplingThread samplingThread) {}

    public default void setPosition(Rotation2d position) {}

    public default void stopMotor() {}

    public default double getAppliedVoltage() {
        return 0;
    }

    public default double getOutputCurrent() {
        return 0;
    }
}
