package com.team1701.lib.drivers.digitalinputs;

import java.util.function.Supplier;

public class DigitalIOSim implements DigitalIO {
    private final Supplier<Boolean> mBlockedSupplier;

    public DigitalIOSim(Supplier<Boolean> blockedSupplier) {
        mBlockedSupplier = blockedSupplier;
    }

    @Override
    public void updateInputs(DigitalInputs inputs) {
        inputs.blocked = mBlockedSupplier.get();
    }
}
