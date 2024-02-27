package com.team1701.lib.drivers.encoders;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class EncoderIORevThroughBore implements EncoderIO {
    public final DutyCycleEncoder mEncoder;
    private boolean mIsInverted;

    public EncoderIORevThroughBore(int channel, boolean isInverted) {
        mEncoder = new DutyCycleEncoder(channel);
        mIsInverted = isInverted;
    }

    @Override
    public void updateInputs(EncoderInputs inputs) {
        var rotations = mIsInverted ? -mEncoder.get() : mEncoder.get();
        inputs.position = Rotation2d.fromRotations(
                rotations == Double.NaN || rotations == Double.POSITIVE_INFINITY ? 0.0 : rotations);
    }

    public void setPositionOffset(Rotation2d offset) {
        mEncoder.setPositionOffset(MathUtil.inputModulus(offset.getRotations(), 0.0, 1.0));
    }
}
