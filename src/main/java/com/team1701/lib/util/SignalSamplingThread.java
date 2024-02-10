package com.team1701.lib.util;

import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;

public interface SignalSamplingThread {
    public Lock getLock();

    public double getFrequency();

    public Queue<Double> addSignal(StatusSignal<Double> signal);

    public Queue<Double> addSignal(DoubleSupplier signal);
}
