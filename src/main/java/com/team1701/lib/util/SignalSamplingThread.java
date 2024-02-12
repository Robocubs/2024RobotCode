package com.team1701.lib.util;

import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;

public interface SignalSamplingThread {
    public Lock getLock();

    public double getFrequency();

    public Queue<Double> addSignal(ParentDevice device, StatusSignal<Double> signal);

    public Queue<Double> addSignal(DoubleSupplier signal);
}
