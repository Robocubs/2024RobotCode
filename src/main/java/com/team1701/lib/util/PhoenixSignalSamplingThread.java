package com.team1701.lib.util;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.team1701.lib.alerts.Alert;

public class PhoenixSignalSamplingThread implements SignalSamplingThread {
    private final Lock mSignalsLock = new ReentrantLock();
    private final Lock mPhoenixSignalsLock = new ReentrantLock();
    private final List<Runnable> mSignalQueues = new ArrayList<>();
    private final double mFrequency;
    private static final Alert mAlert = Alert.error("Added unsupported signal type to PhoenixSignalSamplingThread");

    private boolean mIsCANFD;

    private BaseStatusSignal[] mPhoenixSignals = new BaseStatusSignal[] {};

    public PhoenixSignalSamplingThread(double frequency) {
        var thread = new Thread(this::periodic);
        thread.start();
        mFrequency = frequency;
        mIsCANFD = true;
    }

    public Lock getLock() {
        return mSignalsLock;
    }

    public double getFrequency() {
        return mFrequency;
    }

    public Queue<Double> addSignal(ParentDevice device, StatusSignal<Double> signal) {
        // 100 samples should be plenty for any use cases
        var queue = new ArrayBlockingQueue<Double>(100);

        mPhoenixSignalsLock.lock();
        try {
            mSignalsLock.lock();
            try {
                mSignalQueues.add(new PhoenixSignalQueue(queue, signal));
                if (!CANBus.isNetworkFD(device.getNetwork())) {
                    mIsCANFD = false;
                }
                var newSignals = new BaseStatusSignal[mPhoenixSignals.length + 1];
                System.arraycopy(mPhoenixSignals, 0, newSignals, 0, mPhoenixSignals.length);
                newSignals[mPhoenixSignals.length] = signal;
                mPhoenixSignals = newSignals;
            } finally {
                mSignalsLock.unlock();
            }
        } finally {
            mPhoenixSignalsLock.unlock();
        }

        return queue;
    }

    private void periodic() {
        mPhoenixSignalsLock.lock();
        try {
            if (mIsCANFD && mPhoenixSignals.length > 0) {
                BaseStatusSignal.waitForAll(2 / mFrequency, mPhoenixSignals);
                BaseStatusSignal.refreshAll(mPhoenixSignals);
            } else {
                Thread.sleep((long) (1000.0 / mFrequency));
                if (mPhoenixSignals.length > 0) {
                    BaseStatusSignal.refreshAll(mPhoenixSignals);
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            mPhoenixSignalsLock.unlock();
        }

        mSignalsLock.lock();
        try {
            mSignalQueues.forEach(Runnable::run);
        } finally {
            mSignalsLock.unlock();
        }
    }

    private class PhoenixSignalQueue implements Runnable {
        private final Queue<Double> queue;
        private final StatusSignal<Double> signal;

        public PhoenixSignalQueue(Queue<Double> queue, StatusSignal<Double> signal) {
            this.queue = queue;
            this.signal = signal;
        }

        public void run() {
            queue.offer(signal.getValue());
        }
    }

    @Override
    public Queue<Double> addSignal(DoubleSupplier signal) {
        mAlert.enable();
        return new ArrayDeque<>();
    }
}
