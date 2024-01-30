package com.team1701.lib.drivers.encoders;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class EncoderIOSim implements EncoderIO {
    private final Supplier<Rotation2d> mRotationSupplier;

    public EncoderIOSim(Supplier<Rotation2d> rotationSupplier) {
        mRotationSupplier = rotationSupplier;
    }

    @Override
    public void updateInputs(EncoderInputs inputs) {
        inputs.position = mRotationSupplier.get();
    }
}
