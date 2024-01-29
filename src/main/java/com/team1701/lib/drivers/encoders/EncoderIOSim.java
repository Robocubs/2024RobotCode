package com.team1701.lib.drivers.encoders;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class EncoderIOSim implements EncoderIO {
    public final Supplier<Rotation2d> mRotationSupplier;
    public double rotations;

    public EncoderIOSim(Supplier<Rotation2d> rotationSupplier) {
        mRotationSupplier = rotationSupplier;
        rotations = mRotationSupplier.get().getRotations();
    }

    @Override
    public void updateInputs(EncoderInputs inputs) {
        inputs.position = mRotationSupplier.get();
    }
}
