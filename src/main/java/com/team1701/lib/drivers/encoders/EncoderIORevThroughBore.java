package com.team1701.lib.drivers.encoders;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class EncoderIORevThroughBore implements EncoderIO {
    public final DutyCycleEncoder mEncoder;

    public EncoderIORevThroughBore(int channel) {
        mEncoder = new DutyCycleEncoder(channel);
        mEncoder.setDistancePerRotation(360);
    }

    @Override
    public void updateInputs(EncoderInputs inputs) {
        var rotations = mEncoder.get();
        inputs.position = Rotation2d.fromRotations(
                rotations == Double.NaN || rotations == Double.POSITIVE_INFINITY ? 0.0 : rotations);
    }

    public void setPositionOffset(Rotation2d offset) {
        mEncoder.setPositionOffset(MathUtil.inputModulus(offset.getRotations(), 0.0, 1.0));
    }

    public void setDistancePerRotation(double distancePerRotation) {
        mEncoder.setDistancePerRotation(distancePerRotation);
    }
}
